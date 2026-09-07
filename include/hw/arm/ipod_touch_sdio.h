#ifndef IPOD_TOUCH_SDIO_H
#define IPOD_TOUCH_SDIO_H

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "hw/hw.h"
#include "hw/irq.h"

#define TYPE_IPOD_TOUCH_SDIO                "ipodtouch.sdio"
OBJECT_DECLARE_SIMPLE_TYPE(IPodTouchSDIOState, IPOD_TOUCH_SDIO)

#define SDIO_CMD        0x8
#define SDIO_ARGU       0xC
#define SDIO_STATE      0x10
#define SDIO_STAC       0x14
#define SDIO_DSTA       0x18
#define SDIO_RESP0      0x20
#define SDIO_RESP1      0x24
#define SDIO_RESP2      0x28
#define SDIO_RESP3      0x2C
#define SDIO_CSR        0x34
#define SDIO_IRQ        0x38
#define SDIO_IRQMASK    0x3C
#define SDIO_BADDR      0x44
#define SDIO_BLKLEN     0x48
#define SDIO_NUMBLK     0x4C

#define CMD5_FUNC_OFFSET 28
#define CIS_OFFSET 0xC8
#define CIS_MANUFACTURER_ID 0x20
#define CIS_FUNCTION_EXTENSION 0x22

/* I/O functions besides function 0: F1 backplane (SoC address space)
 * and F2 WLAN packet transfer - see BCM4325 datasheet, "SDIO V1.2". */
#define BCM4325_FUNCTIONS 0x2
#define BCM4325_MANUFACTURER 0x4D50
#define BCM4325_PRODUCT_ID 0x4D48

/* ---- BCM4325 backplane (SDIO function 1) --------------------------------
 * F1 windows the chip's SiliconBackplane address space. The host reads the
 * chip id at offset 0 of the enumeration base to identify the part, then
 * talks to individual cores. Register meanings follow Linux's brcmfmac
 * (drivers/net/wireless/broadcom/brcm80211), which drives the same family.
 *
 *   chipid: [15:0] chip id, [19:16] revision, [31:28] backplane type
 * The 4325 is a SiliconBackplane (SB) part, not the newer AXI/AI type.
 */
#define SI_ENUM_BASE          0x18000000   /* backplane enumeration base */
#define BCM4325_CHIP_ID       0x4325
#define BCM4325_CHIP_REV      0x0
#define SOCI_SB               0x0          /* SiliconBackplane */
/* The value the driver accepts here is 0x00050000 - chip id 0, revision 5.
 * Reporting id 0x4325 instead makes AppleBCM4325 abandon initialisation
 * before it uploads firmware, so this field evidently does not carry the
 * part number the way brcmfmac's newer parts do. Keep the value that works
 * and leave the decode documented for whoever revisits it. */
#define BCM4325_CHIPID_VALUE  0x00050000u

/* SB core base addresses, in the fixed layout the host expects for this
 * generation (mirrors BCM4329_CORE_*_BASE in brcmfmac's chip.c). */
#define BCM4325_CORE_CHIPCOMMON  0x18000000
#define BCM4325_CORE_80211       0x18001000
#define BCM4325_CORE_SDIO_DEV    0x18011000
#define BCM4325_CORE_SOCRAM      0x18003000
#define BCM4325_CORE_ARM_CM3     0x18002000

/* SB config space sits in the top 0x100 bytes of each 4KB core window. */
#define SB_CONFIG_OFF         0xF00
#define SB_IDHIGH             (SB_CONFIG_OFF + 0xFC)  /* core id / revision */
#define SB_TMSTATELOW         (SB_CONFIG_OFF + 0x98)
#define SB_TMSTATEHIGH        (SB_CONFIG_OFF + 0x9C)
#define SB_IDHIGH_CC_SHIFT    4                       /* core id field */
#define SB_TML_RESET          0x0001
#define SB_TML_CLK            0x10000
#define SB_TMH_BUSY           0x0004

/* Core ids, as reported in SB_IDHIGH (brcmfmac: BCMA_CORE_*). */
#define CORE_ID_CHIPCOMMON    0x800
#define CORE_ID_80211         0x812
#define CORE_ID_SDIO_DEV      0x829
#define CORE_ID_INTERNAL_MEM  0x80E
#define CORE_ID_ARM_CM3       0x82A

/* Frames on function 2 begin with a 4-byte length tag (see the BCM4325
 * datasheet, "Device Software Architecture"): the length, then its one's
 * complement as a check. */
typedef struct BCM4325FrameHeaderPacket
{
    uint16_t frame_length;
    uint16_t checksum;
} __attribute__((__packed__)) BCM4325FrameHeaderPacket;

/* SDPCM software header, following the 4-byte length tag.
 *
 * Byte offsets here are from the start of the frame, which is how
 * AppleBCM4325DeviceInterfaceSdio::rxPacket indexes them:
 *
 *   [4] sequence      this frame's sequence number
 *   [5] channel       low nibble only: 0 control, 1 event, 2 data
 *   [7] data offset   bytes from the frame start to the payload (0x0c)
 *   [8] flow control  zero means the host may transmit  -> its +0x3a
 *   [9] credit        highest sequence the host may use -> its +0x39
 */
typedef struct BCM4325SdpcmHeader
{
    uint8_t  sequence;       /* frame byte 4 */
    uint8_t  channel;        /* frame byte 5, low nibble */
    uint8_t  next_length;    /* frame byte 6 */
    uint8_t  data_offset;    /* frame byte 7 */
    uint8_t  flow_control;   /* frame byte 8 */
    uint8_t  credit;         /* frame byte 9 */
    uint8_t  reserved[2];
} __attribute__((__packed__)) BCM4325SdpcmHeader;

/* CDC control header: three words, in both directions.
 *
 * A request carries its iovar name at frame offset 24 = 4 (tag) + 8 (SDPCM)
 * + 12 (this). AppleBCM4325CmdManager reads a response's payload at the same
 * +12 (`addeq sl, r5, #0xc`); with a fourth word here it parsed the header
 * itself as the data - "Scan results: status(262), version(...), buflen(2024)"
 * is cmd, flags and len being read as the result structure. An error is
 * reported in flags bit 0, which the driver tests before touching the payload.
 * The device echoes cmd, len and the request id (flags bits 31..16). */
typedef struct BCM4325CdcHeader
{
    uint32_t cmd;
    uint32_t len;
    uint32_t flags;
} __attribute__((__packed__)) BCM4325CdcHeader;
#define CDC_DCMD_ERROR         0x01

#define SDPCM_CONTROL_CHANNEL  0
#define SDPCM_EVENT_CHANNEL    1
#define SDPCM_DATA_CHANNEL     2
#define SDPCM_HEADER_LEN       (sizeof(BCM4325FrameHeaderPacket) +                                 sizeof(BCM4325SdpcmHeader))
#define CDC_DCMD_SET           0x02   /* 0 = get, 1 = set */
#define CDC_REQUEST_HEADER_LEN sizeof(BCM4325CdcHeader)

/* CDC command numbers used by the driver (brcmfmac: BRCMF_C_*). */
#define CDC_CMD_UP             2
#define CDC_CMD_SET_INFRA     38
#define CDC_CMD_GET_MAGIC     83
#define CDC_CMD_GET_VERSION   84
#define CDC_CMD_GET_RATE      12
#define CDC_CMD_GET_SSID      25
#define CDC_CMD_SET_SSID      26
#define CDC_CMD_GET_CHANNEL   29
#define CDC_CMD_GET_BSSID     86
#define CDC_CMD_GET_RSSI     127
#define CDC_CMD_GET_VAR      262
#define CDC_CMD_SET_VAR      263

/* ---- firmware events ----------------------------------------------------
 * The driver subscribes to asynchronous events with the event_msgs iovar and
 * then waits for them: without a link-up event nothing tells the stack it is
 * associated, and configd retries auto-join forever. Events arrive on SDPCM
 * channel 1 shaped as an Ethernet frame carrying a Broadcom-specific header
 * and a big-endian message (brcmfmac: struct brcmf_event).
 */
#define BRCM_OUI_0             0x00
#define BRCM_OUI_1             0x10
#define BRCM_OUI_2             0x18
#define BCMILCP_SUBTYPE_VENDOR_LONG  32769
#define BCMILCP_BCM_SUBTYPE_EVENT    1
#define BRCMF_E_AUTH            3     /* 802.11 authentication done */
#define BRCMF_E_ASSOC           7     /* association done */
#define BRCMF_E_LINK           16     /* link up/down: flags bit 0 = up */
#define BRCMF_E_SET_SSID       0      /* join completed */
#define BRCMF_E_ESCAN_RESULT   69     /* scan results ready (newer escan API) */
#define BRCMF_E_SCAN_COMPLETE  26     /* scan finished (the iscan API this driver uses) */
#define BRCMF_E_STATUS_SUCCESS 0
#define BRCMF_E_STATUS_PARTIAL 8
#define BRCMF_EVENT_MSG_LINK   0x01

/* Data-channel frames carry a 6-byte BDC header before the Ethernet frame.
 *
 * AppleBCM4325::handleDataPacket logs flags from [r4] and priority from
 * [r4+1], then takes the Ethernet frame at `r4 + 6` (`add r6, r4, #6`) and
 * reads the ethertype at r4+18 - which is 12 bytes into that frame, past the
 * two MAC addresses. Six, not four. */
typedef struct BCM4325BdcHeader
{
    uint8_t  flags;
    uint8_t  priority;
    uint8_t  flags2;
    uint8_t  data_offset;    /* in 4-byte words, beyond this header */
    uint8_t  reserved[2];
} __attribute__((__packed__)) BCM4325BdcHeader;

#define BDC_PROTO_VERSION      2
#define BDC_FLAG_VER_SHIFT     4

typedef struct BCM4325EventHeader
{
    uint8_t  dest[6];
    uint8_t  src[6];
    uint16_t ethertype;      /* big-endian 0x886C */
    /* Broadcom header */
    uint16_t subtype;        /* big-endian */
    uint16_t length;         /* big-endian */
    uint8_t  version;
    uint8_t  oui[3];
    uint16_t usr_subtype;    /* big-endian */
    /* event message, all big-endian */
    uint16_t msg_version;
    uint16_t flags;
    uint32_t event_type;
    uint32_t status;
    uint32_t reason;
    uint32_t auth_type;
    uint32_t datalen;
    uint8_t  addr[6];
    char     ifname[16];
    uint8_t  ifidx;
    uint8_t  bsscfgidx;
} __attribute__((__packed__)) BCM4325EventHeader;

/* ---- scan results -------------------------------------------------------
 * The driver scans with the iscan iovar, the interface that predates escan.
 * A scan-complete event carries no payload; the driver then reads the results
 * back with a GET_VAR on "iscanresults", which returns a status word followed
 * by wl_scan_results: buflen, version, count, then the BSS entries.
 *
 * The BSS layout is the pre-802.11n one (LEGACY_WL_BSS_INFO_VERSION, 107) -
 * a single channel byte where later revisions have a chanspec, and none of
 * the n/ac capability fields. Getting this wrong is invisible: the driver
 * checks version and length, ignores what it does not recognise, and asks
 * again.
 */
/* One scan result, laid out exactly as AppleBCM4325ScanManager reads it
 * (the field dump at 0xc033c068 and the beacon accessors). Offsets are from
 * the entry start; processScanResults steps to the next entry by `length`.
 *
 *   0x00 version   0x04 length   0x08 BSSID   0x0e beacon_period
 *   0x10 capability   0x12 SSID_len   0x13 SSID[32]
 *   0x34 rateset.count   0x38 rates[16]
 *   0x48 chanspec (u16)   0x4a atim_window   0x4c dtim_period
 *   0x4e RSSI (s16)   0x50 phy_noise (s8)   0x51 n_cap
 *   0x54 nbss_cap   0x58 ctl_ch   0x74 ie_offset (u16)   0x78 ie_length
 *
 * This is the 802.11n-era wl_bss_info (chanspec, not a channel byte); the
 * driver never compares `version`. Natural alignment produces these offsets,
 * so the struct is deliberately not packed, and the static assertions below
 * hold it to the disassembly. */
typedef struct BCM4325BssInfo
{
    uint32_t version;
    uint32_t length;
    uint8_t  bssid[6];
    uint16_t beacon_period;
    uint16_t capability;
    uint8_t  ssid_len;
    uint8_t  ssid[32];
    struct {
        uint32_t count;
        uint8_t  rates[16];
    } rateset;
    uint16_t chanspec;
    uint16_t atim_window;
    uint8_t  dtim_period;
    int16_t  rssi;
    int8_t   phy_noise;
    uint8_t  n_cap;
    uint32_t nbss_cap;
    uint8_t  ctl_ch;
    uint32_t reserved32[1];
    uint8_t  flags;
    uint8_t  reserved[3];
    uint8_t  basic_mcs[16];
    uint16_t ie_offset;
    uint32_t ie_length;
    int16_t  snr;
} BCM4325BssInfo;

QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, bssid)       != 0x08);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, capability)  != 0x10);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, ssid_len)    != 0x12);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, rateset)     != 0x34);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, chanspec)    != 0x48);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, dtim_period) != 0x4c);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, rssi)        != 0x4e);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, phy_noise)   != 0x50);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, nbss_cap)    != 0x54);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, ctl_ch)      != 0x58);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, ie_offset)   != 0x74);
QEMU_BUILD_BUG_ON(offsetof(BCM4325BssInfo, ie_length)   != 0x78);
QEMU_BUILD_BUG_ON(sizeof(BCM4325BssInfo) != 0x80);

/* handleIScanResult reads status at +0, buflen at +4, version at +8, count
 * at +12, and hands processScanResults the entries from +16. */
typedef struct BCM4325ScanResults
{
    uint32_t buflen;
    uint32_t version;
    uint32_t count;
    BCM4325BssInfo bss[1];
} BCM4325ScanResults;

typedef struct BCM4325IscanResults
{
    uint32_t status;
    BCM4325ScanResults results;
} BCM4325IscanResults;

QEMU_BUILD_BUG_ON(offsetof(BCM4325IscanResults, results.bss) != 16);

#define BCM4325_BSS_VERSION      109
#define WL_SCAN_RESULTS_SUCCESS  0     /* handleIScanResult: 0 = done */
#define WL_SCAN_RESULTS_PARTIAL  1     /* 1 = "More scan results pending" */
#define BCM4325_FAKE_SSID        "qemu-ios"
#define BCM4325_FAKE_CHANNEL     6
#define BCM4325_SCAN_MS          400   /* virtual ms from "iscan" to completion */
#define BCM4325_JOIN_MS          100   /* virtual ms from WLC_SET_SSID to link-up */
/* chanspec as the driver decodes it: band bits 15..12 (0x2000 = 2.4 GHz),
 * bandwidth bits 11..10 (0x800 = 20 MHz), sideband bits 9..8 (0x300 = none),
 * channel in the low byte. */
#define BCM4325_FAKE_CHANSPEC    (0x2000 | 0x0800 | 0x0300 | BCM4325_FAKE_CHANNEL)

/* A control response waiting to be collected by the host. */
typedef struct BCM4325PendingResponse
{
    uint8_t  channel;       /* SDPCM channel: control, event or data */
    uint8_t  sequence;      /* frame sequence, fixed when queued */
    uint32_t cmd;
    uint32_t len;
    uint32_t flags;
    uint32_t status;
    uint8_t  payload[1600];   /* BDC header + a full Ethernet frame: slirp's DHCP replies are 590 bytes */
    uint32_t payload_len;
} BCM4325PendingResponse;

typedef struct IPodTouchSDIOState
{
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t cmd;
    uint32_t arg;
    uint32_t state;
    uint32_t stac;
    uint32_t csr;
    uint32_t resp0;
    uint32_t resp1;
    uint32_t resp2;
    uint32_t resp3;
    uint32_t irq_reg;
    uint32_t irq_mask;
    uint32_t baddr;
    uint32_t blklen;
    uint32_t numblk;
    QEMUTimer *irq_timer;
    qemu_irq irq;
    qemu_irq irq2;
    GQueue *rx_fifo;        /* control responses awaiting collection */
    GQueue *event_fifo;     /* events and received frames: a separate queue, or
                             * an event queued mid-sequence is collected as if
                             * it were the answer to the next command */
    NICState *nic;          /* host network backend */
    NICConf conf;
    uint8_t tx_sequence;    /* next frame sequence to hand the host */
    QEMUTimer *scan_timer;  /* fires the scan-complete event after "iscan" */
    QEMUTimer *join_timer;  /* fires the association events after WLC_SET_SSID */
    bool associated;
    bool link_up_sent;      /* the association event is sent once */
    uint8_t registers[0x10000];
} IPodTouchSDIOState;

#endif