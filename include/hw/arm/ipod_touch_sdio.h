#ifndef IPOD_TOUCH_SDIO_H
#define IPOD_TOUCH_SDIO_H

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
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

/* SDPCM software header, following the length tag. */
typedef struct BCM4325SdpcmHeader
{
    uint8_t  sequence;
    uint8_t  channel;        /* 0 = control, 1 = event, 2 = data */
    uint8_t  next_length;
    uint8_t  data_offset;    /* bytes from frame start to the payload */
    uint8_t  flow_control;
    uint8_t  max_sequence;   /* highest sequence the device will accept */
    uint8_t  reserved[2];
} __attribute__((__packed__)) BCM4325SdpcmHeader;

/* CDC control message header. The device echoes cmd, len and the request id
 * (top 16 bits of flags) back to the host, which asserts on all three. */
typedef struct BCM4325CdcHeader
{
    uint32_t cmd;
    uint32_t len;
    uint32_t flags;
    uint32_t status;
} __attribute__((__packed__)) BCM4325CdcHeader;

#define SDPCM_CONTROL_CHANNEL  0
#define SDPCM_EVENT_CHANNEL    1
#define SDPCM_DATA_CHANNEL     2
#define SDPCM_HEADER_LEN       (sizeof(BCM4325FrameHeaderPacket) +                                 sizeof(BCM4325SdpcmHeader))
#define CDC_DCMD_SET           0x02   /* 0 = get, 1 = set */
/* A request header carries cmd, len and flags; status is present only in the
 * response, so an iovar name begins 12 bytes into the command, not 16. */
#define CDC_REQUEST_HEADER_LEN 12

/* CDC command numbers used by the driver (brcmfmac: BRCMF_C_*). */
#define CDC_CMD_UP             2
#define CDC_CMD_SET_INFRA     38
#define CDC_CMD_GET_MAGIC     83
#define CDC_CMD_GET_VERSION   84
#define CDC_CMD_GET_BSSID     86
#define CDC_CMD_GET_VAR      262
#define CDC_CMD_SET_VAR      263

/* A control response waiting to be collected by the host. */
typedef struct BCM4325PendingResponse
{
    uint32_t cmd;
    uint32_t len;
    uint32_t flags;
    uint32_t status;
    uint8_t  payload[512];
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
    GQueue *rx_fifo;
    uint8_t registers[0x10000];
} IPodTouchSDIOState;

#endif