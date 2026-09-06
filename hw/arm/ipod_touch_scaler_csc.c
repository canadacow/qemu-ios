#include "hw/arm/ipod_touch_scaler_csc.h"
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

static uint64_t ipod_touch_scaler_csc_read(void *opaque, hwaddr addr, unsigned size)
{
    //fprintf(stderr, "%s: offset = 0x%08x\n", __func__, addr);

    switch (addr) {
        default:
            return 0;
    }

    UNHANDLED_READ("scaler_csc", addr);

    return 0;
}

static void ipod_touch_scaler_csc_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    IPodTouchScalerCSCState *s = IPOD_TOUCH_SCALER_CSC(opaque);
    //printf("%s (base %d): writing 0x%08x to 0x%08x\n", __func__, s->base, data, addr);
}

static const MemoryRegionOps ipod_touch_scaler_csc_ops = {
    .read = ipod_touch_scaler_csc_read,
    .write = ipod_touch_scaler_csc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void ipod_touch_scaler_csc_init(Object *obj)
{
    IPodTouchScalerCSCState *s = IPOD_TOUCH_SCALER_CSC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ipod_touch_scaler_csc_ops, s, TYPE_IPOD_TOUCH_SCALER_CSC, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void ipod_touch_scaler_csc_class_init(ObjectClass *klass, void *data)
{
    
}

static const TypeInfo ipod_touch_scaler_csc_type_info = {
    .name = TYPE_IPOD_TOUCH_SCALER_CSC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodTouchScalerCSCState),
    .instance_init = ipod_touch_scaler_csc_init,
    .class_init = ipod_touch_scaler_csc_class_init,
};

static void ipod_touch_scaler_csc_register_types(void)
{
    type_register_static(&ipod_touch_scaler_csc_type_info);
}

type_init(ipod_touch_scaler_csc_register_types)