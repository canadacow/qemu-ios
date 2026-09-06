#include "hw/arm/ipod_touch_fmss.h"
#include "qemu/error-report.h"
#include "hw/arm/ipod_touch_debug.h"

/* Report registers this model does not implement: they read as zero, and a
 * guest polling one for a ready bit waits forever with nothing in the log. */
#define UNHANDLED_READ(dev, a) do { \
    static uint32_t seen_[64]; static int nseen_; \
    bool dup_ = false; \
    for (int i_ = 0; i_ < nseen_; i_++) { \
        if (seen_[i_] == (uint32_t)(a)) { dup_ = true; break; } } \
    if (!dup_ && nseen_ < 64) { seen_[nseen_++] = (uint32_t)(a); \
        warn_report("%s: unhandled read at 0x%03x -> 0", dev, (uint32_t)(a)); } \
} while (0)

uint64_t temp_storage[512];




/* ---- logical-page store ------------------------------------------------
 * Flash cannot overwrite in place, so the guest's FTL never updates a page
 * where it lies: it writes the new version to a free page and remembers the
 * move in a RAM map it only persists on a clean shutdown - which this driver
 * never performs. Every rewrite is therefore invisible after a reboot.
 *
 * An emulator has no such constraint. Data pages carry their logical page
 * number in the spare (bytes 0..3, type 0x40), so file their contents under
 * that number instead of the physical slot: relocation stops mattering, and a
 * stale map still resolves to the current data. Pages without a logical
 * identity - the FTL's own map and index pages, the VFL context - keep using
 * their physical slot, because for them the slot *is* the identity.
 *
 * lmap[(cs,page)] = lpn + 1   (0 = slot has no logical identity)
 * Page content for a logical page lives at a reserved area of the image.
 */
#define LPN_NONE 0u

/* The logical page number sits in spare word 3, tagged by the magic in word 4.
 * Words 0..2 belong to Apple's format (sequence numbers and the ECC mark at
 * word 2) and must not be disturbed - overwriting the mark makes the FTL
 * reject the page with CHECK_FTL_ECC_MARK. Pages the guest writes carry the
 * FTL's own metadata in words 0..2 and no tag, so they are identified by the
 * user-data spare type instead. */
#define FTL_DATA_SPARE_MAGIC 0x4C504E00u

static inline uint32_t fmss_spare_word(const uint8_t *sp, int w)
{
    return (uint32_t)sp[w * 4] | (uint32_t)sp[w * 4 + 1] << 8 |
           (uint32_t)sp[w * 4 + 2] << 16 | (uint32_t)sp[w * 4 + 3] << 24;
}

static inline bool fmss_spare_is_data(const uint8_t *sp)
{
    return fmss_spare_word(sp, 4) == FTL_DATA_SPARE_MAGIC || sp[9] == 0x40;
}

static inline uint32_t fmss_spare_lpn(const uint8_t *sp)
{
    /* generator page: tagged word 3; guest page: lpn in word 0 (FTL layout) */
    return fmss_spare_word(sp, 4) == FTL_DATA_SPARE_MAGIC
             ? fmss_spare_word(sp, 3) : fmss_spare_word(sp, 0);
}

/* offset of a logical page's record inside the image's logical area */
static inline uint64_t fmss_lpn_offset(uint32_t lpn)
{
    return NAND_IMAGE_LOGICAL_AREA + (uint64_t)lpn * NAND_BYTES_PER_RECORD;
}
static uint8_t find_bit_index(uint8_t num) {
    int index = 0;
    while (num > 1) {
        num >>= 1;
        index++;
    }
    return index;
}

static void write_chip_info(IPodTouchFMSSState *s)
{
    uint32_t chipid[] = { 0xb614d5ad, 0xb614d5ad, 0xb614d5ad, 0xb614d5ad };
    cpu_physical_memory_write(s->reg_cinfo_target_addr, &chipid, 0x10);
}

static void dump_registers(IPodTouchFMSSState *s) {
    printf("FMSS_PAGES_IN_ADDR: 0x%08x\n", s->reg_pages_in_addr);
    printf("FMSS_CS_BUF_ADDR: 0x%08x\n", s->reg_cs_buf_addr);
    printf("FMSS_NUM_PAGES: 0x%08x\n", s->reg_num_pages);
}

static void read_nand_pages(IPodTouchFMSSState *s)
{
    // boot args
    const char *boot_args = "kextlog=0xfff debug=0x8 cpus=1 rd=disk0s1 serial=1 pmu-debug=0x1 io=0xffff8fff debug-usb=0xffffffff amfi_allow_any_signature=1 -v zalloc_debug"; // if not const then overwritten
    cpu_physical_memory_write(0x0ff2a584, boot_args, strlen(boot_args));

    // patch iBoot - we want to inject the bluetooth MAC address which is located as sub-node of uart1 and not uart3 in the device tree...
    const char *chr = "arm-io/uart1/bluetooth";
    cpu_physical_memory_write(0x0ff2206c, chr, strlen(chr));

    int page_out_buf_ind = 0;
    //dump_registers(s);
    //printf("Start CMD...\n");
    for(int page_ind = 0; page_ind < s->reg_num_pages; page_ind++) {
        uint32_t page_nr = 0;
        uint32_t page_out_addr = 0;
        uint32_t cs = 0;
        cpu_physical_memory_read(s->reg_pages_in_addr + (page_ind * sizeof(uint32_t)), &page_nr, sizeof(uint32_t));
        cpu_physical_memory_read(s->reg_cs_buf_addr + (page_ind * sizeof(uint32_t)), &cs, sizeof(uint32_t));
        uint32_t og_cs = cs;
        cs = find_bit_index(cs);

        if(cs > 3) {
            printf("CS %d invalid! (original CS: %d, reading page %d)\n", cs, og_cs, page_nr);
            //dump_registers(s);
            hw_error("CS %d invalid!", cs);
        }

        // prepare the page
        bool page_present = false;

        /* Open a flat NAND image on first use: if nand_path names a regular
         * file (rather than the historical cs<N>/<page>.page directory tree),
         * every page is one fixed-size record inside it. */
        if (!s->nand_image && !s->nand_flat_checked) {
            struct stat nst = {0};
            s->nand_flat_checked = true;
            if (stat(s->nand_path, &nst) == 0 && S_ISREG(nst.st_mode)) {
                /* r+b: the guest programs pages in place; the image must be writable
                 * for settings, keys and caches to survive a reboot. */
                s->nand_image = fopen(s->nand_path, "r+b");
                if (!s->nand_image) {
                    /* fall back to read-only if the image is not writable */
                    s->nand_image = fopen(s->nand_path, "rb");
                    if (s->nand_image) {
                        warn_report("ipod_touch_fmss: %s is read-only; "
                                    "guest writes will not persist", s->nand_path);
                    }
                }
                if (!s->nand_image) {
                    hw_error("Unable to open NAND image %s", s->nand_path);
                }
                /* Presence bitmap, if the packer appended one. */
                if (s->nand_image) {
                    fseeko(s->nand_image, 0, SEEK_END);
                    off_t sz = ftello(s->nand_image);
                    if (sz >= (off_t)(NAND_IMAGE_RECORD_AREA + NAND_IMAGE_BITMAP_LEN)) {
                        s->nand_bitmap = g_malloc(NAND_IMAGE_BITMAP_LEN);
                        if (fseeko(s->nand_image, (off_t)NAND_IMAGE_RECORD_AREA, SEEK_SET) != 0 ||
                            fread(s->nand_bitmap, 1, NAND_IMAGE_BITMAP_LEN, s->nand_image)
                                != NAND_IMAGE_BITMAP_LEN) {
                            warn_report("ipod_touch_fmss: failed to read NAND presence bitmap; "
                                        "treating all pages as present");
                            g_free(s->nand_bitmap);
                            s->nand_bitmap = NULL;
                        }
                    } else {
                        warn_report("ipod_touch_fmss: NAND image has no presence bitmap "
                                    "(old format); never-written pages will read as zero");
                    }
                    if (sz < (off_t)NAND_IMAGE_LOGICAL_AREA) {
                        warn_report("ipod_touch_fmss: image predates the logical-page "
                                    "store; rewrites will not survive a reboot");
                    }
                }
            }
        }

        if (s->nand_image) {
            /* Flat image: one fixed-size record per (cs, page). */
            uint64_t off = ((uint64_t)cs * NAND_PAGES_PER_CS + page_nr) *
                           NAND_BYTES_PER_RECORD;
            /* A never-written page is a hole in the sparse image, which fread()
             * happily returns as zeroes - and content cannot tell a hole from a
             * real page, because the generator writes some pages (NAND
             * signature, bad-block table, FTL maps) with an all-zero spare.
             * The packer records presence in a bitmap after the record area;
             * a clear bit is reported exactly like a missing .page file so the
             * clean-page marker gets synthesised. Images without a bitmap are
             * treated as fully present. */
            uint64_t idx = (uint64_t)cs * NAND_PAGES_PER_CS + page_nr;
            bool present = !s->nand_bitmap ||
                           (s->nand_bitmap[idx >> 3] & (1u << (idx & 7)));
            /* A page beyond the modelled per-CS range must not alias into the
             * next chip select's records (the directory format simply had no
             * such file). Report it once - it means the guest addresses a
             * larger geometry than the image models. */
            if (page_nr >= NAND_PAGES_PER_CS) {
                present = false;
                if (!s->warned_oob_read) {
                    s->warned_oob_read = true;
                    warn_report("ipod_touch_fmss: guest read page %u on cs %u, beyond the "
                                "%u pages/CS the image models (bank 1?)", page_nr, cs,
                                (unsigned)NAND_PAGES_PER_CS);
                }
            }
            /* Read the physical slot first: it is the truth about what the
             * FTL last put here. If that page carries a logical identity, the
             * logical store may hold a NEWER version of the same logical page
             * (the FTL relocated it and only recorded the move in RAM), so
             * prefer the stored copy - but only when the identities match.
             * Trusting a remembered slot->lpn mapping instead would serve
             * stale content once a block is recycled for different data. */
            if (present &&
                fseeko(s->nand_image, (off_t)off, SEEK_SET) == 0 &&
                fread(s->page_buffer, 1, NAND_BYTES_PER_PAGE, s->nand_image)
                    == NAND_BYTES_PER_PAGE &&
                fread(s->page_spare_buffer, 1, NAND_BYTES_PER_SPARE, s->nand_image)
                    == NAND_BYTES_PER_SPARE) {
                page_present = true;

                /* Multi-page reads have never been verified: log what each
                 * entry decodes to and the lpn stamp of the page served. For a
                 * read of consecutive logical pages the stamps must be
                 * consecutive; scattered stamps mean the per-entry decode is
                 * wrong and the guest is receiving the wrong pages. */
                {
                    static unsigned multi_reads;
                    if (s->reg_num_pages > 1 && multi_reads < 14 * 8) {
                        multi_reads++;
                        bool d = fmss_spare_is_data(s->page_spare_buffer);
                        warn_report("fmss: multiread n=%u entry %d: cs=%u page=%u "
                                    "(blk %u pib %u) -> %s lpn=%u", s->reg_num_pages,
                                    page_ind, cs, page_nr, page_nr / 256,
                                    page_nr % 128, d ? "data" : "meta",
                                    d ? fmss_spare_lpn(s->page_spare_buffer) : 0);
                    }
                }

                if (fmss_spare_is_data(s->page_spare_buffer)) {
                    uint32_t lpn = fmss_spare_lpn(s->page_spare_buffer);
                    uint8_t lbuf[NAND_BYTES_PER_PAGE], lspare[NAND_BYTES_PER_SPARE];
                    if (lpn < NAND_MAX_LPN &&
                        fseeko(s->nand_image, (off_t)fmss_lpn_offset(lpn), SEEK_SET) == 0 &&
                        fread(lbuf, 1, NAND_BYTES_PER_PAGE, s->nand_image)
                            == NAND_BYTES_PER_PAGE &&
                        fread(lspare, 1, NAND_BYTES_PER_SPARE, s->nand_image)
                            == NAND_BYTES_PER_SPARE &&
                        fmss_spare_is_data(lspare) &&
                        fmss_spare_lpn(lspare) == lpn) {
                        memcpy(s->page_buffer, lbuf, NAND_BYTES_PER_PAGE);
                        memcpy(s->page_spare_buffer, lspare, NAND_BYTES_PER_SPARE);
                    }
                }
            }
        } else {
            char filename[200];
            sprintf(filename, "%s/cs%d/%d.page", s->nand_path, cs, page_nr);
            struct stat st = {0};
            if (stat(filename, &st) != -1) {
                FILE *f = fopen(filename, "rb");
                if (f == NULL) { hw_error("Unable to read file!"); }
                fread(s->page_buffer, sizeof(char), NAND_BYTES_PER_PAGE, f);
                fread(s->page_spare_buffer, sizeof(char), NAND_BYTES_PER_SPARE, f);
                fclose(f);
                page_present = true;
            }
        }

        if (!page_present) {
            if (s->nand_image) {
                /* Flat image: a never-written page is an ERASED page, and the
                 * FTL identifies erased pages/blocks by an all-0xFF spare (its
                 * restore scan logs anything else as 'unidentified spare'
                 * and never treats a block as free or reusable). Mirror real
                 * NAND: 0xFF data and spare. */
                memset(s->page_buffer, 0xFF, NAND_BYTES_PER_PAGE);
                memset(s->page_spare_buffer, 0xFF, NAND_BYTES_PER_SPARE);
            } else {
                // page storage does not exist - initialize an empty buffer
                memset(s->page_buffer, 0, NAND_BYTES_PER_PAGE);
                memset(s->page_spare_buffer, 0, NAND_BYTES_PER_SPARE);

                uint32_t *buf_cst = (uint32_t *) s->page_spare_buffer;
                buf_cst[2] = 0x00FF00FF;
            }
        }

        // we write away the page in two parts, 2048 bytes first and then the other 2048 bytes.
        int write_buf_size = NAND_BYTES_PER_PAGE / 2;
        for(int i = 0; i < 2; i++) {
            cpu_physical_memory_read(s->reg_pages_out_addr + (page_out_buf_ind * sizeof(uint32_t)), &page_out_addr, sizeof(uint32_t));
            cpu_physical_memory_write(page_out_addr, s->page_buffer + i * write_buf_size, write_buf_size);
            //printf("Will read page %d @ cs %d into address 0x%08x and spare into address 0x%08x\n", page_nr, cs, page_out_addr, s->reg_page_spare_out_addr);
            page_out_buf_ind++;

        }

        // finally, write the spare
        cpu_physical_memory_write(s->reg_page_spare_out_addr + page_ind * 0xc, s->page_spare_buffer, NAND_BYTES_PER_SPARE);
    }
}


/* Program (write) pages.
 *
 * The guest's VFL/FTL does all flash management in software (wear levelling,
 * bad-block remapping, ECC, spare metadata); the controller only has to store
 * bytes at the physical page it is told. Command 0xa02 uses the same buffer
 * registers as the 0xa01 read, but each page-list entry is a packed physical
 * address rather than a bare page number:
 *
 *     bits 31..12  block within the chip select
 *     bits 11..4   page within the block
 *     bits  3..0   chip-select mask, one-hot
 *
 * (observed: 0x801001 = block 0x801, page 0, CS0; 0x801101 = page 16, CS0;
 * 0x801002/4/8 = the same page on CS1/2/3 - an FTL striping a freshly
 * allocated block across the four chip selects.)
 *
 * The page data arrives in the two 2048-byte halves the read path returns it
 * in, and the 12-byte per-page FTL metadata sits at spare_out + idx*12. That
 * metadata is stored verbatim in the first 12 bytes of the spare; the rest is
 * ECC on real hardware and is left zero here. Only the flat image is writable.
 */
#define FMSS_WRITE_MAX_ENTRIES   16
#define FMSS_WRITE_ENTRY_FLAGS   0x00801000u
#define FMSS_ERASE_ENTRY_FLAGS   0x00801100u   /* program flags | bit 8 */



static void write_nand_pages(IPodTouchFMSSState *s)
{
    if (!s->nand_image) {
        if (!s->warned_ro_dir) {
            s->warned_ro_dir = true;
            warn_report("ipod_touch_fmss: NAND is a page directory; guest "
                        "writes are not persisted (use a flat image)");
        }
        return;
    }

    /* Program-command descriptor, at pages_in (cs_buf points at the same
     * buffer): one two-word entry per page,
     *
     *     word 0 = FMSS_WRITE_ENTRY_FLAGS | one-hot chip-select mask
     *     word 1 = chip page number
     *
     * 0xD28/0xD2C hold 2 for every command - words per entry - and 0xD18 is
     * left over from the preceding read, so neither is a page count. The
     * buffer is reused across commands; entries after the last valid one are
     * stale read-list words, so iterate while the flag signature holds. The
     * page data is in the same two 2048-byte halves the read path uses,
     * indexed from pages_out, and the 12 bytes of FTL spare metadata at
     * spare_out + idx*12. */
    static unsigned multi_seen, max_entries;
    int entries = 0;
    for (int idx = 0; idx < FMSS_WRITE_MAX_ENTRIES; idx++) {
        entries = idx + 1;
        uint32_t w0 = 0, w1 = 0, half_addr = 0;
        cpu_physical_memory_read(s->reg_pages_in_addr + (2 * idx) * sizeof(uint32_t),
                                 &w0, sizeof(uint32_t));
        cpu_physical_memory_read(s->reg_pages_in_addr + (2 * idx + 1) * sizeof(uint32_t),
                                 &w1, sizeof(uint32_t));

        uint32_t cs_mask = w0 & 0xF;
        uint32_t flags   = w0 & ~0xFu;
        bool one_hot     = cs_mask && !(cs_mask & (cs_mask - 1));
        bool is_program  = (flags == FMSS_WRITE_ENTRY_FLAGS);
        bool is_erase    = (flags == FMSS_ERASE_ENTRY_FLAGS);
        if (!(is_program || is_erase) || !one_hot) {
            if (idx > 0 && w0 != 0 && w0 != 0xFFFFFFFFu) {
                static unsigned cut_seen;
                if (cut_seen < 12) {
                    cut_seen++;
                    warn_report("fmss: program CUT at entry %d: w0=0x%08x w1=0x%08x "
                                "(d18=%u d28=%u) - pages after this are DROPPED",
                                idx, w0, w1, s->reg_num_pages, s->reg_write_num_pages);
                }
            }
            if (idx == 0) {
                warn_report("ipod_touch_fmss: descriptor word0=0x%08x word1=0x%08x "
                            "is neither program (0x%08x|cs) nor erase (0x%08x|cs) "
                            "- not applied", w0, w1,
                            FMSS_WRITE_ENTRY_FLAGS, FMSS_ERASE_ENTRY_FLAGS);
            }
            break;   /* end of the valid entries */
        }
        uint32_t cs      = find_bit_index(cs_mask);
        uint32_t page_nr = w1;
        if (cs >= NAND_NUM_CS || page_nr >= NAND_PAGES_PER_CS) {
            warn_report("ipod_touch_fmss: refusing out-of-range %s "
                        "(cs=%u page=%u)", is_erase ? "erase" : "program",
                        cs, page_nr);
            break;
        }

        /* Flag bit 8 is not an erase. It is the only form that ever appears
         * in a trace - the plain 0x00801000 program flag is never seen, while
         * pages are demonstrably written - so it is a program variant whose
         * meaning is still unknown. Treating it as an erase clears whole
         * blocks the FTL still needs, including the context blocks at the
         * start of the device, and the volume then fails to mount with
         * "Not HFS+ (signature 0xffff)". */

        int half = NAND_BYTES_PER_PAGE / 2;
        for (int i = 0; i < 2; i++) {
            cpu_physical_memory_read(s->reg_pages_out_addr + (2 * idx + i) * sizeof(uint32_t),
                                     &half_addr, sizeof(uint32_t));
            cpu_physical_memory_read(half_addr, s->page_buffer + i * half, half);
        }
        memset(s->page_spare_buffer, 0, NAND_BYTES_PER_SPARE);
        cpu_physical_memory_read(s->reg_page_spare_out_addr + idx * 0xc,
                                 s->page_spare_buffer, 12);

        uint64_t rec = (uint64_t)cs * NAND_PAGES_PER_CS + page_nr;
        /* A data page is stored under its logical number, so a later read of
         * any slot the FTL maps to that number returns this content. */
        if (fmss_spare_is_data(s->page_spare_buffer)) {
            uint32_t lpn = fmss_spare_lpn(s->page_spare_buffer);
            if (lpn < NAND_MAX_LPN) {
                if (fseeko(s->nand_image, (off_t)fmss_lpn_offset(lpn), SEEK_SET) == 0) {
                    fwrite(s->page_buffer, 1, NAND_BYTES_PER_PAGE, s->nand_image);
                    fwrite(s->page_spare_buffer, 1, NAND_BYTES_PER_SPARE, s->nand_image);
                }
            }
        }

        if (fseeko(s->nand_image, (off_t)(rec * NAND_BYTES_PER_RECORD), SEEK_SET) != 0 ||
            fwrite(s->page_buffer, 1, NAND_BYTES_PER_PAGE, s->nand_image)
                != NAND_BYTES_PER_PAGE ||
            fwrite(s->page_spare_buffer, 1, NAND_BYTES_PER_SPARE, s->nand_image)
                != NAND_BYTES_PER_SPARE) {
            warn_report("ipod_touch_fmss: failed to write page %u @ cs %u",
                        page_nr, cs);
            return;
        }
        if (s->nand_bitmap) {
            uint8_t byte = s->nand_bitmap[rec >> 3] | (1u << (rec & 7));
            if (byte != s->nand_bitmap[rec >> 3]) {
                s->nand_bitmap[rec >> 3] = byte;
                if (fseeko(s->nand_image,
                           (off_t)(NAND_IMAGE_RECORD_AREA + (rec >> 3)), SEEK_SET) == 0) {
                    fwrite(&byte, 1, 1, s->nand_image);
                }
            }
        }
    }
    fflush(s->nand_image);

    if (entries > 1) {
        if (entries > max_entries) { max_entries = entries; }
        if (multi_seen < 30) {
            multi_seen++;
            warn_report("fmss: program with %d entries (max so far %u)", entries, max_entries);
        }
    }
    if (entries >= FMSS_WRITE_MAX_ENTRIES) {
        /* Did the guest supply more than we processed? Peek at the next slot. */
        uint32_t w0n = 0, w1n = 0;
        cpu_physical_memory_read(s->reg_pages_in_addr + (2 * FMSS_WRITE_MAX_ENTRIES) * 4, &w0n, 4);
        cpu_physical_memory_read(s->reg_pages_in_addr + (2 * FMSS_WRITE_MAX_ENTRIES + 1) * 4, &w1n, 4);
        if ((w0n & ~0xFu) == FMSS_WRITE_ENTRY_FLAGS || (w0n & ~0xFu) == FMSS_ERASE_ENTRY_FLAGS) {
            warn_report("fmss: TRUNCATED program - entry %d is valid (w0=0x%08x page=%u) "
                        "but the loop stopped at the cap", FMSS_WRITE_MAX_ENTRIES, w0n, w1n);
        }
    }
}

static uint64_t ipod_touch_fmss_read(void *opaque, hwaddr addr, unsigned size)
{
    //fprintf(stderr, "%s: read from location 0x%08lx\n", __func__, addr);

    IPodTouchFMSSState *s = (IPodTouchFMSSState *)opaque;
    switch(addr)
    {
        case FMSS__CS_BUF_RST_OK:
            return 0x1;
        case FMSS__CS_IRQ:
            return s->reg_cs_irq_bit;
        case FMSS__CS_IRQMASK:
            return 0x1;
        case FMSS__FMCTRL1:
            return (0x1 << 30);
        case 0xD00:
            return 42;
	case 0x00000C30:
	    return 0x1;
        default:
            printf("%s: read invalid location 0x%08x.\n", __func__, addr);
            break;
    }
    UNHANDLED_READ("fmss", addr);
    return 0;
}


static void ipod_touch_fmss_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IPodTouchFMSSState *s = (IPodTouchFMSSState *)opaque;
    //printf("%s: writing 0x%08lx to 0x%08lx\n", __func__, val, addr);

    switch(addr) {
        case 0xC00:
            if(val == 0x0000ffb5) { s->reg_cs_irq_bit = 1; } // TODO ugly and hard-coded
            /* Completion is synchronous. Deferring it by 50us was tried and
             * raised the hang rate from about half of boots to three in four,
             * so the driver's interrupt path is the more fragile one here. */
            if(val == 0xfff5) { s->reg_cs_irq_bit = 1; qemu_set_irq(s->irq, 1); }
            break;
        case FMSS__CS_IRQ:
            s->reg_cs_irq_bit = 0;
            qemu_set_irq(s->irq, 0);
            break;
        case FMSS_CINFO_TARGET_ADDR:
            s->reg_cinfo_target_addr = val;
            write_chip_info(s);
            break;
        case FMSS_PAGES_IN_ADDR:
            s->reg_pages_in_addr = val;
            break;
        case FMSS_CS_BUF_ADDR:
            s->reg_cs_buf_addr = val;
            break;
        case FMSS_NUM_PAGES:
            s->reg_num_pages = val;
            break;
        case FMSS_WRITE_NUM_PAGES:
        case FMSS_WRITE_NUM_PAGES2:
            /* The program command carries its own page count here; 0xD18 is
             * left over from the preceding read and must not be trusted. */
            s->reg_write_num_pages = val;
            break;
        case FMSS_PAGE_SPARE_OUT_ADDR:
            s->reg_page_spare_out_addr = val;
            break;
        case FMSS_PAGES_OUT_ADDR:
            s->reg_pages_out_addr = val;
            break;
        case FMSS_CSGENRC:
            s->reg_csgenrc = val;
            break;
        case 0xD38:
            /* Transfer trigger. csgenrc selects the operation: 0xa01 reads the
             * pages named by the descriptor, 0xa02 programs them. */
            {
                static unsigned ncmd, nrd, nwr;
                if (s->reg_csgenrc == 0xa01) { nrd++; } else if (s->reg_csgenrc == 0xa02) { nwr++; }
                if (++ncmd % 250 == 0) {
                    warn_report("fmss: heartbeat cmds=%u reads=%u programs=%u", ncmd, nrd, nwr);
                }
            }
            if (s->reg_csgenrc == 0xa01) { read_nand_pages(s); }
            else if (s->reg_csgenrc == 0xa02) { write_nand_pages(s); }
            else {
                /* Any other operation is dropped on the floor. The flash layer
                 * advertises WriteMultiple; if that is a distinct command, every
                 * bulk write - a journal replay, say - vanishes here silently. */
                static uint32_t seen[16]; static int nseen;
                bool dup = false;
                for (int i = 0; i < nseen; i++) { if (seen[i] == s->reg_csgenrc) { dup = true; } }
                if (!dup && nseen < 16) {
                    uint32_t w[4] = {0};
                    if (s->reg_pages_in_addr) { cpu_physical_memory_read(s->reg_pages_in_addr, w, 16); }
                    seen[nseen++] = s->reg_csgenrc;
                    warn_report("fmss: UNHANDLED command csgenrc=0x%x num_pages=%u d28=%u "
                                "in[0..3]=%08x %08x %08x %08x - DROPPED",
                                s->reg_csgenrc, s->reg_num_pages, s->reg_write_num_pages,
                                w[0], w[1], w[2], w[3]);
                }
            }
            break;
        default:
            break;
    }
}


static void fmss_complete_cb(void *opaque)
{
    IPodTouchFMSSState *s = (IPodTouchFMSSState *)opaque;
    s->reg_cs_irq_bit = 1;
    qemu_set_irq(s->irq, 1);
}

static const MemoryRegionOps fmss_ops = {
    .read = ipod_touch_fmss_read,
    .write = ipod_touch_fmss_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void ipod_touch_fmss_realize(DeviceState *dev, Error **errp)
{
    
}

static void ipod_touch_fmss_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(sbd);
    IPodTouchFMSSState *s = IPOD_TOUCH_FMSS(dev);

    memory_region_init_io(&s->iomem, obj, &fmss_ops, s, "fmss", 0xF00);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->complete_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fmss_complete_cb, s);

    s->page_buffer = (uint8_t *)malloc(NAND_BYTES_PER_PAGE);
    s->page_spare_buffer = (uint8_t *)malloc(NAND_BYTES_PER_SPARE);
}

static void ipod_touch_fmss_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ipod_touch_fmss_realize;
}

static const TypeInfo ipod_touch_fmss_info = {
    .name          = TYPE_IPOD_TOUCH_FMSS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodTouchFMSSState),
    .instance_init = ipod_touch_fmss_init,
    .class_init    = ipod_touch_fmss_class_init,
};

static void ipod_touch_machine_types(void)
{
    type_register_static(&ipod_touch_fmss_info);
}

type_init(ipod_touch_machine_types)
