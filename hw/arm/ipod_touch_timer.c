#include "hw/arm/ipod_touch_timer.h"
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


/* Anomaly detection for the timer model: each of these is a way to lose a
 * wakeup silently. Counted and reported once per virtual second. */
static struct {
    unsigned raises, latches, starts, low_wo_high, double_raise, latch_dropped,
             past_deadline, long_deadline, unh_reads[8]; uint32_t unh_addr[8]; int nunh;
    bool raised, tick_since_latch, high_read; int64_t last_report;
} tdbg;
static Clock *tdbg_sysclk;
static void tdbg_report(const char *why)
{
    char u[160] = ""; int n = 0;
    for (int i = 0; i < tdbg.nunh && n < (int)sizeof(u) - 24; i++)
        n += snprintf(u + n, sizeof(u) - n, " 0x%03x:%u", tdbg.unh_addr[i], tdbg.unh_reads[i]);
    warn_report("timer: t=%.3fs tb=%llu %s raises=%u latches=%u starts=%u low-wo-high=%u double-raise=%u "
                "latch-dropped-tick=%u past-deadline=%u long-deadline=%u unhandled-reads:%s",
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1e9,
                (unsigned long long)(tdbg_sysclk ? clock_ns_to_ticks(tdbg_sysclk, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 2) : 0),
                why, tdbg.raises, tdbg.latches,
                tdbg.starts, tdbg.low_wo_high, tdbg.double_raise, tdbg.latch_dropped,
                tdbg.past_deadline, tdbg.long_deadline, u);
}
static void tdbg_unhandled(uint32_t a)
{
    for (int i = 0; i < tdbg.nunh; i++) if (tdbg.unh_addr[i] == a) { tdbg.unh_reads[i]++; return; }
    if (tdbg.nunh < 8) { tdbg.unh_addr[tdbg.nunh] = a; tdbg.unh_reads[tdbg.nunh++] = 1; }
}
static void s5l8900_st_update(IPodTouchTimerState *s)
{
    /* The deadline timer must run at the same rate as the 64-bit timebase the
     * kernel reads for mach_absolute_time: sysclk (12MHz) halved below, so
     * 6MHz. At the previous hard-coded 10MHz every deadline fired when the
     * timebase had covered only 60% of the interval; the kernel saw "not yet",
     * re-armed for the remainder, and fired early again - a geometric re-arm
     * storm on every wakeup (arms per fire climbed from 1.0 to 3.2 in a hung
     * run) and a system in which no two time sources agreed. */
    s->freq_out = s->sysclk ? clock_get_hz(s->sysclk) / 2 : 6000000;
    s->tick_interval = /* bcount1 * get_ticks / freq  + ((bcount2 * get_ticks / freq)*/
    muldiv64((s->bcount1 < 1000) ? 1000 : s->bcount1, NANOSECONDS_PER_SECOND, s->freq_out);
    s->next_planned_tick = 0;
}

static void s5l8900_st_set_timer(IPodTouchTimerState *s)
{
    uint64_t last = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->base_time;

    s->next_planned_tick = last + (s->tick_interval - last % s->tick_interval);
    timer_mod(s->st_timer, s->next_planned_tick + s->base_time);
    s->last_tick = last;
}

static void s5l8900_st_tick(void *opaque)
{
    IPodTouchTimerState *s = (IPodTouchTimerState *)opaque;

    if (s->status & TIMER_STATE_START) {
        //fprintf(stderr, "%s: Raising irq\n", __func__);
        tdbg.raises++; if (tdbg.raised) tdbg.double_raise++; tdbg.raised = true; tdbg.tick_since_latch = true;
        if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - tdbg.last_report > 1000000000LL) { tdbg.last_report = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); tdbg_sysclk = s->sysclk; tdbg_report("hb"); }
        qemu_irq_raise(s->irq);

        /* schedule next interrupt */
        if(!(s->status & TIMER_STATE_MANUALUPDATE)) {
            s5l8900_st_set_timer(s);
        }
    } else {
        s->next_planned_tick = 0;
        s->last_tick = 0;
        timer_del(s->st_timer);
    }
}

static void s5l8900_timer1_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    //fprintf(stderr, "%s: writing 0x%08x to 0x%08x\n", __func__, value, addr);
    IPodTouchTimerState *s = (struct IPodTouchTimerState *) opaque;

    switch(addr){

        case TIMER_IRQSTAT:
            { static unsigned k; if (k++ < 40) warn_report("timer: t=%.4fs IRQSTAT <- 0x%08x", qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1e9, (uint32_t)value); }
            s->irqstat = value;
            return;
        case TIMER_IRQLATCH:
            //fprintf(stderr, "%s: lowering irq\n", __func__);
            tdbg.latches++; if (!tdbg.raised) tdbg.latch_dropped++; tdbg.raised = false; tdbg.tick_since_latch = false;
            qemu_irq_lower(s->irq);     
            return;
        case TIMER_4 + TIMER_CONFIG:
            s5l8900_st_update(s);
            s->config = value;
            break;
        case TIMER_4 + TIMER_STATE:
            { static unsigned k; if (k++ < 40) warn_report("timer: t=%.4fs T4 STATE <- 0x%x", qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1e9, (uint32_t)value); }
            if (value & TIMER_STATE_START) {
                s->base_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                s5l8900_st_update(s);
                s5l8900_st_set_timer(s);
                { tdbg.starts++; int64_t in_ = (int64_t)(s->next_planned_tick + s->base_time) - qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); if (in_ < 0) tdbg.past_deadline++; if (in_ > 2000000000LL) { tdbg.long_deadline++; warn_report("timer: START deadline %.3fs away (bcount1=%u)", in_/1e9, s->bcount1); } }
            } else if (value == TIMER_STATE_STOP) {
                timer_del(s->st_timer);
            }
            s->status = value;
            break;
        case TIMER_4 + TIMER_COUNT_BUFFER:
            { static unsigned k; if (k++ < 40) warn_report("timer: t=%.4fs T4 COUNT <- %u", qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1e9, (uint32_t)value); }
            s->bcount1 = s->bcreload = value;
            break;
        case TIMER_4 + TIMER_COUNT_BUFFER2:
            s->bcount2 = value;
            break;
      default: {
          static unsigned nw; static uint32_t seen[16]; static int ns; bool first = true;
          for (int i = 0; i < ns; i++) if (seen[i] == (uint32_t)addr) first = false;
          if (first && ns < 16) seen[ns++] = (uint32_t)addr;
          if (nw < 60 || first) { nw++;
              warn_report("timer: t=%.4fs UNHANDLED WRITE 0x%03x <- 0x%08x", qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1e9, (uint32_t)addr, (uint32_t)value); }
          break; }
    }
}

static uint64_t s5l8900_timer1_read(void *opaque, hwaddr addr, unsigned size)
{
    //fprintf(stderr, "%s: read from location 0x%08x\n", __func__, addr);
    IPodTouchTimerState *s = (struct IPodTouchTimerState *) opaque;
    uint64_t elapsed_ns, ticks;

    switch (addr) {
        case TIMER_TICKSHIGH:    // needs to be fixed so that read from low first works as well
            tdbg.high_read = true;

            elapsed_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 2; // the timer ticks twice as slow as the CPU frequency in the kernel
            ticks = clock_ns_to_ticks(s->sysclk, elapsed_ns);
            //printf("TICKS: %lld\n", ticks);
            s->ticks_high = (ticks >> 32);
            s->ticks_low = (ticks & 0xFFFFFFFF);
            return s->ticks_high;
        case TIMER_TICKSLOW:
            if (!tdbg.high_read) tdbg.low_wo_high++; tdbg.high_read = false;
            return s->ticks_low;
        case TIMER_IRQSTAT:
            return s->irqstat; // ~0; // s->irqstat;
        case TIMER_IRQLATCH:
            return 0xffffffff;

      default:
        break;
    }
    tdbg_unhandled((uint32_t)addr);
    UNHANDLED_READ("timer", addr);
    return 0;
}

static const MemoryRegionOps timer1_ops = {
    .read = s5l8900_timer1_read,
    .write = s5l8900_timer1_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void s5l8900_timer_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(sbd);
    IPodTouchTimerState *s = IPOD_TOUCH_TIMER(dev);

    memory_region_init_io(&s->iomem, obj, &timer1_ops, s, "timer1", 0x10001);
    sysbus_init_irq(sbd, &s->irq);

    s->base_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->st_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, s5l8900_st_tick, s);
}

static void s5l8900_timer_class_init(ObjectClass *klass, void *data)
{

}

static const TypeInfo ipod_touch_timer_info = {
    .name          = TYPE_IPOD_TOUCH_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodTouchTimerState),
    .instance_init = s5l8900_timer_init,
    .class_init    = s5l8900_timer_class_init,
};

static void ipod_touch_machine_types(void)
{
    type_register_static(&ipod_touch_timer_info);
}

type_init(ipod_touch_machine_types)
