#include "hw/arm/ipod_touch_mipi_dsi.h"
#include "hw/arm/ipod_touch_debug.h"
#include "qemu/error-report.h"

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

static uint64_t ipod_touch_mipi_dsi_read(void *opaque, hwaddr addr, unsigned size)
{
    if (addr != 0x00000)
	fprintf(stderr, "%s: read from location 0x%08lx\n", __func__, addr);

    IPodTouchMIPIDSIState *s = (IPodTouchMIPIDSIState *)opaque;
    switch(addr)
    {
        case 0x0:
            return 0x103 | rDSIM_STATUS_TxReadyHsClk;
        case REG_INTSRC:
            return rDSIM_INTSRC_RxDatDone;
        case REG_RXFIFO:
            if(!s->return_panel_id) {
                s->return_panel_id = true;
                // TODO this should be rewritten as a proper queue!
                return DSIM_RSP_LONG_READ | (3 << 8); // the latter part indicates the length of the response (the panel ID)
            } else {
                s->return_panel_id = false;
                return 0x00a1d13c;
            }
            
        case REG_FIFOCTRL:
            return rDSIM_FIFOCTRL_EmptyHSfr;
        default:
            printf("%s: read invalid location 0x%08lx.\n", __func__, addr);
            break;
    }
    UNHANDLED_READ("mipi_dsi", addr);
    return 0;
}

static void ipod_touch_mipi_dsi_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IPodTouchMIPIDSIState *s = (IPodTouchMIPIDSIState *)opaque;
    fprintf(stderr, "%s: writing 0x%08lx to 0x%08lx\n", __func__, val, addr);

    switch(addr)
    {
        case REG_PKTHDR:
            s->pkthdr_reg = val;
            break;
	case 0x00000008:
	    if (val == 0)
		printf("turning off screen\n");
        default:
            break;
    }
}

static const MemoryRegionOps mipi_dsi_ops = {
    .read = ipod_touch_mipi_dsi_read,
    .write = ipod_touch_mipi_dsi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void ipod_touch_mipi_dsi_realize(DeviceState *dev, Error **errp)
{
    
}

static void ipod_touch_mipi_dsi_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(sbd);
    IPodTouchMIPIDSIState *s = IPOD_TOUCH_MIPI_DSI(dev);

    memory_region_init_io(&s->iomem, obj, &mipi_dsi_ops, s, "mipi_dsi", 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->return_panel_id = 0;
}

static void ipod_touch_mipi_dsi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ipod_touch_mipi_dsi_realize;
}

static const TypeInfo ipod_touch_mipi_dsi_info = {
    .name          = TYPE_IPOD_TOUCH_MIPI_DSI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodTouchMIPIDSIState),
    .instance_init = ipod_touch_mipi_dsi_init,
    .class_init    = ipod_touch_mipi_dsi_class_init,
};

static void ipod_touch_machine_types(void)
{
    type_register_static(&ipod_touch_mipi_dsi_info);
}

type_init(ipod_touch_machine_types)
