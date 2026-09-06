#ifndef IPOD_TOUCH_FMSS_H
#define IPOD_TOUCH_FMSS_H

#include <math.h>
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/irq.h"

#define TYPE_IPOD_TOUCH_FMSS                "ipodtouch.fmss"
OBJECT_DECLARE_SIMPLE_TYPE(IPodTouchFMSSState, IPOD_TOUCH_FMSS)

#define NAND_BYTES_PER_PAGE 4096
#define NAND_BYTES_PER_SPARE 64

/* Flat NAND image support.
 * The original backing store is one file per page (nand/cs<N>/<page>.page),
 * which means ~262k tiny files: slow to generate, slow to copy, and very slow
 * over WSL/network filesystems. A flat image stores the same data as a single
 * file of fixed-size records, indexed by (cs, page).
 */
#define NAND_BYTES_PER_RECORD (NAND_BYTES_PER_PAGE + NAND_BYTES_PER_SPARE)
#define NAND_PAGES_PER_BLOCK  128
#define NAND_BLOCKS_PER_CS    4096
#define NAND_BANKS_PER_CS     1
#define NAND_PAGES_PER_CS     (NAND_BLOCKS_PER_CS * NAND_PAGES_PER_BLOCK * NAND_BANKS_PER_CS)   /* 524288 */
#define NAND_NUM_CS           4
/* Flat image: records, then a presence bitmap (1 bit per (cs,page)). */
#define NAND_IMAGE_RECORD_AREA ((uint64_t)NAND_NUM_CS * NAND_PAGES_PER_CS * NAND_BYTES_PER_RECORD)
#define NAND_IMAGE_BITMAP_LEN  (NAND_NUM_CS * NAND_PAGES_PER_CS / 8)
/* Logical-page area: content filed by logical page number, plus a map from
 * each physical slot to the logical page it last held (0 = none). Both live
 * after the record area and the presence bitmap. */
#define NAND_MAX_LPN           (1u << 21)          /* 2M logical pages = 8 GiB */
#define NAND_IMAGE_LMAP_OFF    (NAND_IMAGE_RECORD_AREA + NAND_IMAGE_BITMAP_LEN)
#define NAND_IMAGE_LMAP_LEN    ((uint64_t)NAND_NUM_CS * NAND_PAGES_PER_CS * 4)
#define NAND_IMAGE_LOGICAL_AREA (NAND_IMAGE_LMAP_OFF + NAND_IMAGE_LMAP_LEN)

#define FMSS__FMCTRL1             0x4
#define FMSS__CS_IRQ              0xC0C
#define FMSS__CS_IRQMASK          0xC10
#define FMSS__CS_BUF_RST_OK       0xC64
#define FMSS_CINFO_TARGET_ADDR    0xD08
#define FMSS_PAGES_IN_ADDR        0xD0C
#define FMSS_CS_BUF_ADDR          0xD10
#define FMSS_NUM_PAGES            0xD18
#define FMSS_PAGE_SPARE_OUT_ADDR  0xD1C
#define FMSS_PAGES_OUT_ADDR       0xD20
#define FMSS_WRITE_NUM_PAGES      0xD28
#define FMSS_WRITE_NUM_PAGES2     0xD2C
#define FMSS_CSGENRC              0xD30

typedef struct IPodTouchFMSSState
{
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint8_t *page_buffer;
    uint8_t *page_spare_buffer;

    uint32_t reg_cs_irq_bit;
    QEMUTimer *complete_timer;   /* asynchronous command completion */
    uint32_t reg_cinfo_target_addr;
    uint32_t reg_pages_in_addr;
    uint32_t reg_cs_buf_addr;
    uint32_t reg_num_pages;
    uint32_t reg_page_spare_out_addr;
    uint32_t reg_pages_out_addr;
    uint32_t reg_csgenrc;
    uint32_t reg_write_num_pages;
    char *nand_path;
    FILE *nand_image;     /* non-NULL when nand_path is a flat image */
    uint8_t *nand_bitmap; /* page presence bitmap, NULL if the image has none */
    bool nand_flat_checked;
#define FMSS_MAX_SEEN_CMDS 16
    uint32_t seen_cmds[FMSS_MAX_SEEN_CMDS];
    int num_seen_cmds;
    bool warned_ro_dir;
    bool warned_oob_read;
} IPodTouchFMSSState;

#endif