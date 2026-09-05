#include "hw/arm/ipod_touch_sdio.h"
#include <string.h>
#include "net/eth.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/arm/ipod_touch_debug.h"
#include "qemu/error-report.h"

/* SDIO/BCM4325 bring-up trace. The device is being implemented, so its
 * command log stays on independently of IPOD_TOUCH_DEBUG; it is a few lines
 * per second at most, unlike the per-page device chatter. */
/* Command-level SDIO/BCM4325 trace. A few lines per second, so it is not
 * gated behind IPOD_TOUCH_DEBUG - the WLAN device is still being brought
 * up and this is what shows how far the driver gets. */
#define SDTRACE(fmt, ...) do { \
    fprintf(stderr, "sdio: " fmt "\n", ##__VA_ARGS__); \
    fflush(stderr); } while (0)

static void bcm4325_queue_event(IPodTouchSDIOState *s, uint32_t event_type,
                                uint32_t status, uint32_t flags);

/* The one network this device reports; the scan result and the association
 * queries both use it, so they tell the same story. */
static const uint8_t bcm4325_bssid[6] = { 0x02, 0x00, 0x5e, 0x10, 0x00, 0x01 };

/* Value the backplane returns for a 32-bit read at a windowed F1 address.
 *
 * The host identifies the chip from the id at the enumeration base, then walks
 * a fixed list of cores, reading each one's SB config space (the top 0x100
 * bytes of its 4KB window) for a core id and reset/clock state. Reporting the
 * cores as present and out of reset is what lets it proceed to loading
 * firmware; an unknown address reads as zero rather than leaving the host's
 * buffer untouched, which previously returned stale guest memory.
 */
static uint32_t bcm4325_backplane_read(IPodTouchSDIOState *s, uint32_t addr)
{
    uint32_t win = addr & ~0xFFFu;      /* which core's 4KB window */
    uint32_t off = addr & 0xFFFu;       /* offset within it */

    /* The chip id lives at offset 0 of the enumeration base. */
    if (addr == 0) {
        return BCM4325_CHIPID_VALUE;
    }

    if (off >= SB_CONFIG_OFF) {
        uint32_t core_id = 0;
        switch (win) {
        case 0x00000: core_id = CORE_ID_CHIPCOMMON;   break;
        case 0x01000: core_id = CORE_ID_80211;        break;
        case 0x02000: core_id = CORE_ID_ARM_CM3;      break;
        case 0x03000: core_id = CORE_ID_INTERNAL_MEM; break;
        case 0x11000: core_id = CORE_ID_SDIO_DEV;     break;
        default: return 0;
        }
        switch (off) {
        case SB_IDHIGH:
            /* core id in bits 15..4, revision 0 */
            return core_id << SB_IDHIGH_CC_SHIFT;
        case SB_TMSTATELOW:
            /* clock enabled, not held in reset */
            return SB_TML_CLK;
        case SB_TMSTATEHIGH:
            /* not busy */
            return 0;
        default:
            return 0;
        }
    }
    return 0;
}


/* ---- CDC control channel ------------------------------------------------
 * Control traffic on function 2 is a length tag, an SDPCM header, then a CDC
 * command header. The device must answer each command with a response whose
 * header echoes the command id, the data length and the request id (the top
 * 16 bits of flags) - Apple's driver asserts on all three in
 * AppleBCM4325CmdManager, and retries forever when they do not match.
 * Layout and semantics follow Linux's brcmfmac (bcdc.c), which speaks the
 * same protocol to this family of parts.
 */
static void bcm4325_handle_control_write(IPodTouchSDIOState *s, uint32_t len)
{
    uint8_t frame[2048];
    if (len > sizeof(frame)) { len = sizeof(frame); }
    cpu_physical_memory_read(s->baddr, frame, len);


    if (len < SDPCM_HEADER_LEN + sizeof(BCM4325CdcHeader)) {
        return;
    }

    BCM4325SdpcmHeader *sdpcm =
        (BCM4325SdpcmHeader *)(frame + sizeof(BCM4325FrameHeaderPacket));
    uint32_t off = sdpcm->data_offset ? sdpcm->data_offset : SDPCM_HEADER_LEN;

    /* A data frame is an Ethernet packet the guest is transmitting: hand it
     * to the host network rather than parsing it as a command. */
    if (sdpcm->channel == SDPCM_DATA_CHANNEL) {
        BCM4325FrameHeaderPacket *tag = (BCM4325FrameHeaderPacket *)frame;
        uint32_t flen = tag->frame_length;
        if (flen > off && flen <= len && s->nic) {
            qemu_send_packet(qemu_get_queue(s->nic), frame + off, flen - off);
            SDTRACE("  tx %u bytes to the network", flen - off);
        } else {
            SDTRACE("  tx dropped: frame_length=%u off=%u len=%u", flen, off, len);
        }
        return;
    }
    if (off + sizeof(BCM4325CdcHeader) > len) {
        return;
    }
    BCM4325CdcHeader *cdc = (BCM4325CdcHeader *)(frame + off);

    /* GET_VAR/SET_VAR carry the variable name as a NUL-terminated string at
     * the start of the payload; log it, since it says what the driver wants. */
    const char *iovar = "";
    char namebuf[64];
    if ((cdc->cmd == 262 || cdc->cmd == 263) &&
        off + CDC_REQUEST_HEADER_LEN < len) {
        /* dump the 8 bytes around the assumed payload start, to place it */
        const char *p = (const char *)(frame + off + CDC_REQUEST_HEADER_LEN);
        uint32_t avail = len - off - CDC_REQUEST_HEADER_LEN;
        uint32_t n = 0;
        while (n < avail && n < sizeof(namebuf) - 1 && p[n] >= 0x20 && p[n] < 0x7f) {
            namebuf[n] = p[n]; n++;
        }
        namebuf[n] = 0;
        iovar = namebuf;
    }
    BCM4325PendingResponse *resp = g_malloc0(sizeof(*resp));
    resp->channel = SDPCM_CONTROL_CHANNEL;
    resp->cmd    = cdc->cmd;
    resp->flags  = cdc->flags;      /* carries the request id the host checks */
    resp->status = 0;               /* success */

    /* Association queries answer with the network the scan reports, so the
     * two agree. A zeroed BSSID reads as "associated to nothing", which is
     * the contradiction that drives configd into the Ask To Join panel. */
    if (cdc->cmd == CDC_CMD_GET_BSSID && resp->payload_len >= 6) {
        memcpy(resp->payload, bcm4325_bssid, sizeof(bcm4325_bssid));
    }

    /* A scan request is answered with results, but not here: queueing an event
     * straight after the reply puts it in the middle of the command/response
     * cycle, and the driver reads it where it expects the next reply. Note the
     * request and deliver it from the idle poll, when nothing is outstanding. */
    if (cdc->cmd == CDC_CMD_SET_VAR && !strcmp(iovar, "iscan")) {
        s->scan_pending = true;
    }

    /* The host checks the response's len against the length it sent, so echo
     * that back whatever we actually return. A get carries that many bytes of
     * payload (zeroed - what each command should report is not yet modelled);
     * a set carries none, but still reports the length it was given. */
    resp->len = cdc->len;

    resp->payload_len = (cdc->flags & CDC_DCMD_SET) ? 0 : (cdc->len & 0xFFFF);
    if (resp->payload_len > sizeof(resp->payload)) {
        resp->payload_len = sizeof(resp->payload);
    }

    /* Answer the queries whose value the driver actually acts on. Everything
     * else still returns zeros, which reads as "feature absent / disabled"
     * and keeps initialisation moving. */
    if (cdc->cmd == CDC_CMD_GET_VAR && resp->payload_len >= 6 &&
        (!strcmp(iovar, "cur_etheraddr") || !strcmp(iovar, "perm_etheraddr"))) {
        /* Locally-administered address; the OTP would hold this on real
         * hardware. Without it the interface has no identity and the stack
         * will not bring it up. */
        memcpy(resp->payload, s->conf.macaddr.a, 6);
    } else if (cdc->cmd == CDC_CMD_GET_VAR && resp->payload_len >= 4 &&
               !strcmp(iovar, "ver")) {
        /* A printable firmware version string; the driver logs it. */
        snprintf((char *)resp->payload, sizeof(resp->payload),
                 "wl0: emulated BCM4325 (qemu-ios)");
    }

    SDTRACE("  CDC cmd=%u %s len=%u id=%u %s", cdc->cmd,
            (cdc->flags & CDC_DCMD_SET) ? "set" : "get",
            cdc->len & 0xFFFF, cdc->flags >> 16, iovar);

    g_queue_push_tail(s->rx_fifo, resp);

    /* Events are not injected yet. The wire format and the delivery channel
     * are right - sent as data frames they no longer draw "WTF?? Got an event
     * packet!!!" - but pushing three of them straight after the iscan reply
     * lands them in the middle of the driver's command/response cycle: it
     * reads the first event where it expects the next command's reply, and
     * AppleBCM4325CmdManager then fails its cmd/len/transaction-id asserts and
     * blocks, which is the spinner. Association needs the driver to be idle
     * between commands before an event can be delivered safely. */
}

/* Build the frame the host reads back for a queued control response. */
static uint32_t bcm4325_build_control_frame(BCM4325PendingResponse *resp,
                                            uint8_t *out, uint32_t max)
{
    uint32_t total = SDPCM_HEADER_LEN + resp->payload_len +
                     (resp->channel == SDPCM_CONTROL_CHANNEL
                          ? sizeof(BCM4325CdcHeader) : 0);
    if (total > max) { return 0; }
    memset(out, 0, total);

    BCM4325FrameHeaderPacket *tag = (BCM4325FrameHeaderPacket *)out;
    tag->frame_length = total;
    tag->checksum = ~total;

    BCM4325SdpcmHeader *sdpcm =
        (BCM4325SdpcmHeader *)(out + sizeof(BCM4325FrameHeaderPacket));
    sdpcm->channel = resp->channel;
    sdpcm->data_offset = SDPCM_HEADER_LEN;
    sdpcm->max_sequence = 4;      /* credit for further transmissions */

    if (resp->channel == SDPCM_CONTROL_CHANNEL) {
        BCM4325CdcHeader *cdc = (BCM4325CdcHeader *)(out + SDPCM_HEADER_LEN);
        cdc->cmd    = resp->cmd;
        cdc->len    = resp->len;
        cdc->flags  = resp->flags;
        cdc->status = resp->status;
        if (resp->payload_len) {
            memcpy(out + SDPCM_HEADER_LEN + sizeof(BCM4325CdcHeader),
                   resp->payload, resp->payload_len);
        }
    } else {
        /* Event and data frames carry their payload directly. */
        memcpy(out + SDPCM_HEADER_LEN, resp->payload, resp->payload_len);
    }
    return total;
}


/* Queue a firmware event. The driver subscribes to these with event_msgs and
 * waits for them; a link-up event is what tells the stack it is associated. */

/* The one network this device reports. Its BSSID is what the association
 * queries answer with, so the scan result and the association state tell the
 * same story - a scan that finds nothing, or a BSSID of all zeros, leaves
 * configd asking SpringBoard to present the Ask To Join panel. */

static void bcm4325_fill_bss(BCM4325BssInfo *bss)
{
    memset(bss, 0, sizeof(*bss));
    bss->version = BCM4325_BSS_VERSION;
    bss->length = sizeof(*bss);
    memcpy(bss->bssid, bcm4325_bssid, sizeof(bcm4325_bssid));
    bss->beacon_period = 100;
    bss->capability = 0x0001;          /* ESS, no privacy - an open network */
    bss->ssid_len = strlen(BCM4325_FAKE_SSID);
    memcpy(bss->ssid, BCM4325_FAKE_SSID, bss->ssid_len);
    bss->rateset.count = 4;
    bss->rateset.rates[0] = 0x82;      /* 1Mbps basic */
    bss->rateset.rates[1] = 0x84;      /* 2Mbps basic */
    bss->rateset.rates[2] = 0x8b;      /* 5.5Mbps basic */
    bss->rateset.rates[3] = 0x96;      /* 11Mbps basic */
    bss->chanspec = BCM4325_FAKE_CHANNEL;
    bss->dtim_period = 1;
    bss->rssi = (uint16_t)(int16_t)-40; /* a strong signal */
    bss->phy_noise = -90;
    bss->ctl_ch = 6;
    bss->ie_offset = sizeof(*bss);
    bss->ie_length = 0;
    bss->snr = 50;
}

static void bcm4325_queue_event_data(IPodTouchSDIOState *s, uint32_t event_type,
                                     uint32_t status, uint32_t flags,
                                     const void *data, uint32_t datalen);

static void bcm4325_queue_event(IPodTouchSDIOState *s, uint32_t event_type,
                                uint32_t status, uint32_t flags)
{
    bcm4325_queue_event_data(s, event_type, status, flags, NULL, 0);
}

static void bcm4325_queue_event_data(IPodTouchSDIOState *s, uint32_t event_type,
                                     uint32_t status, uint32_t flags,
                                     const void *data, uint32_t datalen)
{
    BCM4325PendingResponse *resp = g_malloc0(sizeof(*resp));
    const uint8_t *mac = s->conf.macaddr.a;
    BCM4325EventHeader *ev = (BCM4325EventHeader *)resp->payload;

    /* Real firmware delivers events to the host as ordinary data frames that
     * the stack recognises by their 0x886C ethertype - not on a separate
     * channel. Marking them as channel 1 makes AppleBCM4325 report "WTF?? Got
     * an event packet!!!" from its data-frame path. */
    resp->channel = SDPCM_DATA_CHANNEL;

    memcpy(ev->dest, mac, 6);
    memcpy(ev->src, mac, 6);
    ev->ethertype   = cpu_to_be16(0x886C);
    ev->subtype     = cpu_to_be16(BCMILCP_SUBTYPE_VENDOR_LONG);
    ev->length      = cpu_to_be16(sizeof(*ev) - 12);
    ev->version     = 0;
    ev->oui[0] = BRCM_OUI_0; ev->oui[1] = BRCM_OUI_1; ev->oui[2] = BRCM_OUI_2;
    ev->usr_subtype = cpu_to_be16(BCMILCP_BCM_SUBTYPE_EVENT);
    ev->msg_version = cpu_to_be16(2);
    ev->flags       = cpu_to_be16(flags);
    ev->event_type  = cpu_to_be32(event_type);
    ev->status      = cpu_to_be32(status);
    ev->reason      = 0;
    ev->auth_type   = 0;
    memcpy(ev->addr, mac, 6);
    g_strlcpy(ev->ifname, "eth0", sizeof(ev->ifname));
    ev->ifidx = 0;
    ev->bsscfgidx = 0;

    ev->datalen = cpu_to_be32(datalen);
    resp->payload_len = sizeof(*ev);
    if (data && datalen && sizeof(*ev) + datalen <= sizeof(resp->payload)) {
        memcpy(resp->payload + sizeof(*ev), data, datalen);
        resp->payload_len += datalen;
    }
    SDTRACE("  event type=%u status=%u datalen=%u queued", event_type, status,
            datalen);
    g_queue_push_tail(s->event_fifo, resp);
}

/* A frame arriving from the host network. Hand it to the guest on the data
 * channel, wrapped the same way a real device would. */
static ssize_t bcm4325_receive(NetClientState *nc, const uint8_t *buf,
                               size_t size)
{
    IPodTouchSDIOState *s = qemu_get_nic_opaque(nc);
    BCM4325PendingResponse *resp;

    if (size > sizeof(resp->payload)) {
        return size;   /* drop oversized frames rather than truncate */
    }
    resp = g_malloc0(sizeof(*resp));
    resp->channel = SDPCM_DATA_CHANNEL;
    memcpy(resp->payload, buf, size);
    resp->payload_len = size;
    SDTRACE("  rx %u bytes from the network: ethertype %02x%02x",
            (unsigned)size, size > 13 ? buf[12] : 0, size > 13 ? buf[13] : 0);

    /* Network receive callbacks already run with the big QEMU lock held, so
     * this must not take it again - doing so deadlocks the emulator and the
     * guest then panics with "Unable to communicate with SDIO device". */
    g_queue_push_tail(s->event_fifo, resp);
    s->irq_reg |= 0x1;
    qemu_irq_raise(s->irq);
    return size;
}

static NetClientInfo net_bcm4325_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = bcm4325_receive,
};

static void trigger_irq(void *opaque)
{
    IPodTouchSDIOState *s = (IPodTouchSDIOState *)opaque;
    /* Set the data-ready bit without clearing transfer-complete: the two are
     * independent causes sharing one register, and this timer fires 10ms after
     * a write. Overwriting the register loses the completion the host is still
     * waiting for, and it then blocks until the SDIO layer reports a DMA
     * timeout. */
    s->irq_reg |= 0x2;
    qemu_irq_raise(s->irq);
}

void sdio_exec_cmd(IPodTouchSDIOState *s)
{
    uint32_t cmd_type = s->cmd & 0x3f;
    uint32_t addr = (s->arg >> 9) & 0x1ffff;
    uint32_t func = (s->arg >> 28) & 0x7;
    SDTRACE("CMD%-2d addr=0x%05x func=%d arg=0x%08x", cmd_type, addr, func, s->arg);
    if(cmd_type == 0x3) {
        // RCA request - ignore
    }
    else if(cmd_type == 0x5) {
        /* CMD5 (IO_SEND_OP_COND) - the R4 response is what tells the host a
         * card exists at all: without it the driver just repeats
         * "Searching for SDIO device in slot: 0" forever.
         *
         *   bit 31      card ready (initialisation complete)
         *   bits 30..28 number of I/O functions besides function 0
         *   bit 27      memory present (0 - this is I/O only)
         *   bits 23..0  OCR voltage window; report the usual 2.0-3.6V range,
         *               since a host that finds no supported voltage treats
         *               the card as unusable.
         */
        s->resp0 = (1u << 31)
                 | (BCM4325_FUNCTIONS << CMD5_FUNC_OFFSET)
                 | 0x00FF8000;
    }
    else if(cmd_type == 0x7) {
        // select card - ignore
    }
    else if(cmd_type == 0x34) {
        // CMD52 - read/write from a register
        bool is_write = (s->arg >> 31) != 0;
        if(is_write) {
            uint8_t data = s->arg & 0xFF;
            s->registers[addr] = data;
            if(addr == 0x2) { s->registers[0x3] = data; } // if we write to register 2, we also write the same result to register 3 (this is the enabled functions register)
            SDTRACE("  CMD52 write 0x%02x -> reg 0x%05x", data, addr);
        } else {
            if(addr == 0x1000e) {
                // misc register
                s->resp0 = (1 << 6) /* enable ALP clock */ | (1 << 7); /* enable HT clock */
            }
            else if(addr == 0x2020) {
                // some indication that packets are ready??
                s->resp0 = (1 << 6);
            }
            else {
                SDTRACE("  CMD52 read  reg 0x%05x -> 0x%02x", addr, s->registers[addr]);
                s->resp0 = s->registers[addr];
            }
        }
    }
    else if(cmd_type == 0x35) {
        // CMD53 - block transfer
        addr = addr & 0x7fff;
        bool is_write = (s->arg >> 31) != 0;
        SDTRACE("  CMD53 func=%d addr=0x%05x %s %d x %dB -> mem 0x%08x", func, addr,
                is_write ? "write" : "read", s->numblk, s->blklen, s->baddr);
        
        if(is_write) {
            if(func == 0x1) {
                cpu_physical_memory_read(s->baddr, &s->registers[addr], s->blklen * s->numblk);
            }
            else if(func == 0x2) {
                /* The host has sent a frame. Parse the control command it
                 * carries and queue the matching response, then raise the
                 * interrupt so it comes back to collect it. */
                bcm4325_handle_control_write(s, s->blklen * s->numblk);
                timer_mod(s->irq_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND / 100);
            }
        } else {
            if(func == 0x1) {
                /* Backplane read: answer from the modelled core layout rather
                 * than leaving the host's buffer as it was. */
                uint32_t len = s->blklen * s->numblk;
                for (uint32_t i = 0; i + 4 <= len; i += 4) {
                    uint32_t v = bcm4325_backplane_read(s, addr + i);
                    cpu_physical_memory_write(s->baddr + i, &v, 4);
                }
                if (len < 4) {
                    uint32_t v = bcm4325_backplane_read(s, addr);
                    cpu_physical_memory_write(s->baddr, &v, len);
                }
            }
            else if(func == 0x2) {
                /* The host reads a frame in two steps: first the length tag
                 * and headers, then the whole frame. Serve a queued control
                 * response if there is one; otherwise report an empty frame,
                 * which the host treats as "nothing to collect". */
                uint32_t want = s->blklen * s->numblk;
                uint8_t frame[2048] = { 0 };
                uint32_t total = 0;

                GQueue *q = !g_queue_is_empty(s->rx_fifo) ? s->rx_fifo
                                                          : s->event_fifo;
                if (!g_queue_is_empty(q)) {
                    BCM4325PendingResponse *resp =
                        (BCM4325PendingResponse *)g_queue_peek_head(q);
                    total = bcm4325_build_control_frame(resp, frame, sizeof(frame));
                    /* Only consume it once the host asks for the full frame;
                     * the initial short read is just peeking at the header. */
                    if (total && want >= total) {
                        g_free(g_queue_pop_head(q));
                    }
                }
                /* Idle: nothing is outstanding, so this is the safe moment to
                 * deliver an unsolicited event. */
                if (!total && s->scan_pending) {
                    BCM4325EscanResult r;
                    s->scan_pending = false;
                    memset(&r, 0, sizeof(r));
                    r.buflen = sizeof(r);
                    r.version = BCM4325_ESCAN_VERSION;
                    r.sync_id = 1;
                    r.bss_count = 1;
                    bcm4325_fill_bss(&r.bss);
                    /* This driver scans with iscan, the API that predates
                     * escan, so the completion it waits for is SCAN_COMPLETE
                     * (26) rather than ESCAN_RESULT (69). */
                    bcm4325_queue_event_data(s, BRCMF_E_SCAN_COMPLETE,
                                             BRCMF_E_STATUS_SUCCESS, 0,
                                             &r, sizeof(r));
                    if (!g_queue_is_empty(s->event_fifo)) {
                        BCM4325PendingResponse *ev =
                            (BCM4325PendingResponse *)g_queue_peek_head(s->event_fifo);
                        total = bcm4325_build_control_frame(ev, frame, sizeof(frame));
                        if (total && want >= total) {
                            g_free(g_queue_pop_head(s->event_fifo));
                        }
                    }
                }
                if (!total) {
                    /* Nothing queued. After collecting a response the host
                     * always polls once more for a follow-on frame; answering
                     * with a header-only frame makes it report "a packet
                     * fragment, flags(00)". Leave the buffer as it is and
                     * report no length at all - the tag then reads as zero,
                     * which is how an idle device answers a poll. */
                    total = 0;
                }
                if (total >= SDPCM_HEADER_LEN + sizeof(BCM4325CdcHeader)) {
                    BCM4325CdcHeader *dbg =
                        (BCM4325CdcHeader *)(frame + SDPCM_HEADER_LEN);
                    SDTRACE("  F2 read want=%u -> %u bytes  reply cmd=%u len=%u "
                            "id=%u status=%u", want, total, dbg->cmd, dbg->len,
                            dbg->flags >> 16, dbg->status);
                } else {
                    SDTRACE("  F2 read want=%u -> %u bytes%s", want, total,
                            total ? "" : " (no frame)");
                }
                /* Write the whole length the host asked for, not just the
                 * frame. A short write leaves the tail of its buffer holding
                 * the previous response, and it then parses a stale CDC header
                 * - which is what the driver reports as three failed asserts on
                 * cmd, len and transaction id. frame[] is zeroed, so the
                 * padding reads as zeros. */
                /* Never write more than the host asked for: its 12-byte peek
                 * buffer is 12 bytes, and overrunning it corrupts guest kernel
                 * memory - the SDIO controller then wedges with "timed out on
                 * DMA transaction" and the filesystem is damaged enough to drop
                 * the device into recovery. Pad with zeros when the frame is
                 * shorter than the request. */
                uint32_t fill = want < sizeof(frame) ? want : sizeof(frame);
                cpu_physical_memory_write(s->baddr, frame, fill);
            }
            
        }

        /* Every transfer completes with an interrupt: the host waits for it
         * during the firmware upload too, long before any frame exists, so
         * this cannot be made conditional on the receive queue. */
        s->irq_reg |= 0x1;
        qemu_irq_raise(s->irq);
    }
    else {
        hw_error("Unknown SDIO command %d", cmd_type);
    }
}

static void ipod_touch_sdio_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    printf("%s: writing 0x%08x to 0x%08x\n", __func__, value, addr);
    
    IPodTouchSDIOState *s = (struct IPodTouchSDIOState *) opaque;

    switch(addr) {
        case SDIO_CMD:
            s->cmd = value;
            if(value & (1 << 31)) { // execute bit is set
                sdio_exec_cmd(s);
            }
            break;
        case SDIO_ARGU:
            s->arg = value;
            break;
        case SDIO_STATE:
            s->state = value;
            break;
        case SDIO_STAC:
            s->stac = value;
            break;
        case SDIO_CSR:
            s->csr = value;
            break;
        case SDIO_IRQ:
            /* Write-one-to-clear: the host acknowledges the causes it has
             * handled. The bits must be cleared here - they accumulate as
             * separate causes, so leaving them set holds the interrupt
             * asserted and the driver polls forever. */
            s->irq_reg &= ~(uint32_t)value;
            if (!s->irq_reg) {
                qemu_irq_lower(s->irq);
            }
            break;
        case SDIO_IRQMASK:
            s->irq_mask = value;
            break;
        case SDIO_BADDR:
            s->baddr = value;
            break;
        case SDIO_BLKLEN:
            s->blklen = value;
            break;
        case SDIO_NUMBLK:
            s->numblk = value;
            break;
        default:
            break;
    }
}

static uint64_t ipod_touch_sdio_read(void *opaque, hwaddr addr, unsigned size)
{
    printf("%s: offset = 0x%08x\n", __func__, addr);

    IPodTouchSDIOState *s = (struct IPodTouchSDIOState *) opaque;

    switch (addr) {
        case SDIO_CMD:
            return s->cmd;
        case SDIO_ARGU:
            return s->arg;
        case SDIO_STATE:
            return s->state;
        case SDIO_STAC:
            return s->stac;
        case SDIO_DSTA:
            return (1 << 0) | (1 << 4) ; // 0x1 indicates that the SDIO is ready for a CMD, (1 << 4) that the command is complete
        case SDIO_RESP0:
            return s->resp0;
        case SDIO_RESP1:
            return s->resp1 == 0 ? 0x7465 : s->resp1;
        case SDIO_RESP2:
            return s->resp2 == 0 ? 0x7374 : s->resp2;
        case SDIO_RESP3:
            return s->resp3 == 0 ? 0xFFFF : s->resp3;
        case SDIO_CSR:
            return s->csr;
        case SDIO_IRQ:
            return s->irq_reg;
        case SDIO_IRQMASK:
            return s->irq_mask;
        case SDIO_BADDR:
            return s->baddr;
        case SDIO_BLKLEN:
            return s->blklen;
        case SDIO_NUMBLK:
            return s->numblk;
        default:
            break;
    }

    return 0;
}

static const MemoryRegionOps ipod_touch_sdio_ops = {
    .read = ipod_touch_sdio_read,
    .write = ipod_touch_sdio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void ipod_touch_sdio_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    IPodTouchSDIOState *s = IPOD_TOUCH_SDIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->registers[0x9] = CIS_OFFSET; // registers 0x9 - 0xB contain the relative address offset, which we set to 0xC8 (200)
    s->registers[19] = 0x1; // enable support for high speed mode

    // set the vendor information
    s->registers[CIS_OFFSET] = CIS_MANUFACTURER_ID;
    s->registers[CIS_OFFSET + 1] = 0x4;
    s->registers[CIS_OFFSET + 2] = BCM4325_MANUFACTURER & 0xFF;
    s->registers[CIS_OFFSET + 3] = (BCM4325_MANUFACTURER >> 8) & 0xFF;
    s->registers[CIS_OFFSET + 4] = BCM4325_PRODUCT_ID & 0xFF;
    s->registers[CIS_OFFSET + 5] = (BCM4325_PRODUCT_ID >> 8) & 0xFF;

    // set the MAC address (00:23:32:6E:AA:10)
    s->registers[CIS_OFFSET + 6] = CIS_FUNCTION_EXTENSION;
    s->registers[CIS_OFFSET + 8] = 0x4; // unknown
    s->registers[CIS_OFFSET + 9] = 0x6; // the length of the MAC address
    s->registers[CIS_OFFSET + 10] = 0x0;
    s->registers[CIS_OFFSET + 11] = 0x23;
    s->registers[CIS_OFFSET + 12] = 0x32;
    s->registers[CIS_OFFSET + 13] = 0x6E;
    s->registers[CIS_OFFSET + 14] = 0xAA;
    s->registers[CIS_OFFSET + 15] = 0x10;

    memory_region_init_io(&s->iomem, obj, &ipod_touch_sdio_ops, s, TYPE_IPOD_TOUCH_SDIO, 4096);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, trigger_irq, s);

    s->rx_fifo = g_queue_new();
    s->event_fifo = g_queue_new();
}

static void ipod_touch_sdio_realize(DeviceState *dev, Error **errp)
{
    IPodTouchSDIOState *s = IPOD_TOUCH_SDIO(dev);

    /* The MAC the CIS advertises, unless one was given on the command line. */
    static const uint8_t default_mac[6] = { 0x00, 0x23, 0x32, 0x6E, 0xAA, 0x10 };
    bool have_mac = false;
    for (int i = 0; i < 6; i++) {
        if (s->conf.macaddr.a[i]) { have_mac = true; break; }
    }
    if (!have_mac) {
        memcpy(s->conf.macaddr.a, default_mac, sizeof(default_mac));
    }
    for (int i = 0; i < 6; i++) {
        s->registers[CIS_OFFSET + 10 + i] = s->conf.macaddr.a[i];
    }

    s->nic = qemu_new_nic(&net_bcm4325_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static Property ipod_touch_sdio_properties[] = {
    DEFINE_NIC_PROPERTIES(IPodTouchSDIOState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void ipod_touch_sdio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ipod_touch_sdio_realize;
    device_class_set_props(dc, ipod_touch_sdio_properties);
}

static const TypeInfo ipod_touch_sdio_type_info = {
    .name = TYPE_IPOD_TOUCH_SDIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodTouchSDIOState),
    .instance_init = ipod_touch_sdio_init,
    .class_init = ipod_touch_sdio_class_init,
};

static void ipod_touch_sdio_register_types(void)
{
    type_register_static(&ipod_touch_sdio_type_info);
}

type_init(ipod_touch_sdio_register_types)
