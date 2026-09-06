#include "hw/arm/ipod_touch_lcd.h"
#include "hw/arm/ipod_touch_bezel.h"
#include "hw/arm/ipod_touch_2g.h"
#include <zlib.h>
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "ui/pixel_ops.h"
#include "ui/console.h"
#include "hw/display/framebuffer.h"
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

int lcd_brightness = 255;

static uint64_t ipod_touch_lcd_read(void *opaque, hwaddr addr, unsigned size)
{
    // printf("%s: read from location 0x%08x\n", __func__, addr);

    IPodTouchLCDState *s = (IPodTouchLCDState *)opaque;
    switch(addr)
    {
        case 0x0:
            return 2;
        case 0x4:
            return s->lcd_con;
        case 0xC:
            return 0x1; //s->unknown1;
        case 0x20:
            return s->w1_display_depth_info;
        case 0x24:
            return s->w1_framebuffer_base;
        case 0x28:
            return s->w1_hspan;
        case 0x30:
            return s->w1_display_resolution_info;
        case 0x1b10:
            return 2;
	case 0x1b14:
	    return 0x3;
        default:
            printf("%s: read invalid location 0x%08x.\n", __func__, addr);
            break;
    }
    UNHANDLED_READ("lcd", addr);
    return 0;
}

static void ipod_touch_lcd_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IPodTouchLCDState *s = (IPodTouchLCDState *)opaque;
    // printf("%s: writing 0x%08x to 0x%08x\n", __func__, val, addr);

    switch(addr) {
        case 0x4:
            s->lcd_con = val;
            break;
        case 0xC:
            s->render = val;
            qemu_irq_lower(s->irq);
            break;
        case 0x20:
            s->w1_display_depth_info = val;
            break;
        case 0x24:
            s->w1_framebuffer_base = val;
            break;
        case 0x28:
            s->w1_hspan = val;
            break;
        case 0x30:
            s->w1_display_resolution_info = val;
            break;
    }
}

void lcd_changebrightness(int brightness)
{
    lcd_brightness = brightness & 0xFF;
}

static void lcd_invalidate(void *opaque)
{
    IPodTouchLCDState *s = opaque;
    s->invalidate = 1;
}

static void draw_line32_32(void *opaque, uint8_t *d, const uint8_t *s, int width, int deststep)
{
    // IPodTouchLCDState *lcd = (IPodTouchLCDState *) opaque;
    uint8_t r, g, b;

    do {
        //v = lduw_le_p((void *) s);
        //printf("V: %d\n", *s);
        b = s[0];
        g = s[1];
        r = s[2];
        // printf("R: %d, G: %d, B: %d, A: %d\n", r, g, b, lcd_brightness);
        ((uint32_t *) d)[0] = rgb_to_pixel32(round((float)r * ((float)lcd_brightness / 255.0f)), round((float)g * ((float)lcd_brightness / 255.0f)), round((float)b * ((float)lcd_brightness / 255.0f)));
        s += 4;
        d += 4;
    } while (-- width != 0);
}


/* ---- bezel (device skin) support ---------------------------------- */

static void bezel_composite(IPodTouchLCDState *lcd)
{
    DisplaySurface *surface = qemu_console_surface(lcd->con);
    uint8_t *dst_base = surface_data(surface);
    int stride = surface_stride(surface);
    const uint8_t *src = lcd->bezel_rgba;
    int x, y;

    if (!src || !dst_base) {
        return;
    }

    for (y = 0; y < BEZEL_HEIGHT; y++) {
        uint8_t *dst = dst_base + y * stride;
        for (x = 0; x < BEZEL_WIDTH; x++) {
            const uint8_t *p = src + (y * BEZEL_WIDTH + x) * 4;
            uint8_t a = p[3];
            uint8_t *d = dst + x * 4;
            bool in_screen = (x >= BEZEL_SCREEN_X && x < BEZEL_SCREEN_X + BEZEL_SCREEN_W &&
                              y >= BEZEL_SCREEN_Y && y < BEZEL_SCREEN_Y + BEZEL_SCREEN_H);
            if (in_screen) {
                continue;
            }
            /* Blend against BEZEL_BG, never against the existing buffer:
             * the surface starts uninitialised, so blending with it would
             * tint the antialiased corners with garbage (white). */
            if (a == 255) {
                d[0] = p[2]; d[1] = p[1]; d[2] = p[0];
            } else if (a == 0) {
                d[0] = BEZEL_BG_B; d[1] = BEZEL_BG_G; d[2] = BEZEL_BG_R;
            } else {
                d[0] = (p[2] * a + BEZEL_BG_B * (255 - a)) / 255;
                d[1] = (p[1] * a + BEZEL_BG_G * (255 - a)) / 255;
                d[2] = (p[0] * a + BEZEL_BG_R * (255 - a)) / 255;
            }
            d[3] = 0xFF;
        }
    }
    dpy_gfx_update(lcd->con, 0, 0, BEZEL_WIDTH, BEZEL_HEIGHT);
}

static int bezel_hit_test(int bx, int by)
{
    int dx = bx - BEZEL_HOME_CX;
    int dy = by - BEZEL_HOME_CY;
    if (dx * dx + dy * dy <= BEZEL_HOME_R * BEZEL_HOME_R) {
        return BEZEL_BTN_HOME;
    }
    if (bx >= BEZEL_VOLUP_X0 && bx <= BEZEL_VOLUP_X1 &&
        by >= BEZEL_VOLUP_Y0 && by <= BEZEL_VOLUP_Y1) {
        return BEZEL_BTN_VOLUP;
    }
    if (bx >= BEZEL_VOLDN_X0 && bx <= BEZEL_VOLDN_X1 &&
        by >= BEZEL_VOLDN_Y0 && by <= BEZEL_VOLDN_Y1) {
        return BEZEL_BTN_VOLDN;
    }
    return BEZEL_BTN_NONE;
}

static void bezel_button_event(IPodTouchLCDState *lcd, int btn, bool down)
{
    int code;
    switch (btn) {
    case BEZEL_BTN_HOME:  code = down ? KEY_H_DOWN    : KEY_H_UP;    break;
    case BEZEL_BTN_VOLUP: code = down ? KEY_PLUS_DOWN : KEY_PLUS_UP; break;
    case BEZEL_BTN_VOLDN: code = down ? KEY_MIN_DOWN  : KEY_MIN_UP;  break;
    default: return;
    }
    ipod_touch_key_event(lcd->mt, code);
}

static void lcd_refresh(void *opaque)
{
    // printf("%s: refreshing LCD screen\n", __func__);

    IPodTouchLCDState *lcd = (IPodTouchLCDState *) opaque;
    DisplaySurface *surface = qemu_console_surface(lcd->con);
    drawfn draw_line;
    int src_width, dest_width;
    int height, first, last;
    int width, linesize;

    if (!lcd || !lcd->con || !surface_bits_per_pixel(surface))
        return;

    dest_width = 4;
    draw_line = draw_line32_32;

    /* Resolution */
    first = last = 0;
    width = 320;
    height = 480;
    lcd->invalidate = 1;

    src_width =  4 * width;
    linesize = surface_stride(surface);

    if(lcd->invalidate) {
        framebuffer_update_memory_section(&lcd->fbsection, lcd->sysmem, lcd->w1_framebuffer_base, height, 4 * width);
    }

    if (lcd->bezel_enabled) {
        /* Paint the static bezel once (and after any full invalidate). */
        if (!lcd->bezel_drawn) {
            bezel_composite(lcd);
            lcd->bezel_drawn = true;
        }
        /* Render the 320x480 framebuffer into a scratch surface, then blit it
         * into the screen cut-out. framebuffer_update_display() always writes
         * at the surface origin, so it cannot target a sub-rectangle. */
        framebuffer_update_display(lcd->fb_surface, &lcd->fbsection,
                                   width, height,
                                   src_width,
                                   surface_stride(lcd->fb_surface),
                                   dest_width,
                                   lcd->invalidate,
                                   draw_line, NULL,
                                   &first, &last);
        if (first >= 0) {
            uint8_t *dst_base = surface_data(surface);
            uint8_t *src_base = surface_data(lcd->fb_surface);
            int dst_stride = surface_stride(surface);
            int src_stride = surface_stride(lcd->fb_surface);
            int row;
            for (row = first; row <= last; row++) {
                memcpy(dst_base + (BEZEL_SCREEN_Y + row) * dst_stride + BEZEL_SCREEN_X * 4,
                       src_base + row * src_stride,
                       width * 4);
            }
            dpy_gfx_update(lcd->con, BEZEL_SCREEN_X, BEZEL_SCREEN_Y + first,
                           width, last - first + 1);
        }
    } else {
        framebuffer_update_display(surface, &lcd->fbsection,
                                   width, height,
                                   src_width,
                                   linesize,
                                   dest_width,
                                   lcd->invalidate,
                                   draw_line, NULL,
                                   &first, &last);
        if (first >= 0) {
            dpy_gfx_update(lcd->con, 0, first, width, last - first + 1);
        }
    }
    lcd->invalidate = 0;
}

static const MemoryRegionOps lcd_ops = {
    .read = ipod_touch_lcd_read,
    .write = ipod_touch_lcd_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const GraphicHwOps gfx_ops = {
    .invalidate  = lcd_invalidate,
    .gfx_update  = lcd_refresh,
};

static void ipod_touch_lcd_mouse_event(void *opaque, int x, int y, int z, int buttons_state)
{
    // printf("x %d y %d z %d state %d\n", x, y, z, buttons_state);

    IPodTouchLCDState *lcd = (IPodTouchLCDState *) opaque;
    float fx, fy;

    if (lcd->bezel_enabled) {
        /* absolute coords are normalised over the whole (bezel-sized) window */
        int bx = (int)((x / pow(2, 15)) * BEZEL_WIDTH);
        int by = (int)((y / pow(2, 15)) * BEZEL_HEIGHT);
        int hit = bezel_hit_test(bx, by);

        /* release a held bezel button when the mouse comes up or leaves it */
        if (lcd->bezel_btn_held != BEZEL_BTN_NONE &&
            (!buttons_state || hit != lcd->bezel_btn_held)) {
            bezel_button_event(lcd, lcd->bezel_btn_held, false);
            lcd->bezel_btn_held = BEZEL_BTN_NONE;
        }
        if (hit != BEZEL_BTN_NONE) {
            if (buttons_state && lcd->bezel_btn_held == BEZEL_BTN_NONE) {
                lcd->bezel_btn_held = hit;
                bezel_button_event(lcd, hit, true);
            }
            return;   /* not a touchscreen event */
        }
        /* re-normalise into the screen cut-out */
        fx = (bx - BEZEL_SCREEN_X) / (float)BEZEL_SCREEN_W;
        fy = 1 - (by - BEZEL_SCREEN_Y) / (float)BEZEL_SCREEN_H;
        if (fx < 0 || fx > 1 || fy < 0 || fy > 1) {
            return;   /* outside the display */
        }
    } else {
        fx = x / pow(2, 15);
        fy = 1 - y / pow(2, 15);
    }

    lcd->mt->prev_touch_x = lcd->mt->touch_x;
    lcd->mt->prev_touch_y = lcd->mt->touch_y;
    lcd->mt->touch_x = fx;
    lcd->mt->touch_y = fy;

    if(buttons_state && !lcd->mt->touch_down) {
        ipod_touch_multitouch_on_touch(lcd->mt);
    }
    else if(!buttons_state && lcd->mt->touch_down) {
        ipod_touch_multitouch_on_release(lcd->mt);
    }
}

static void refresh_timer_tick(void *opaque)
{
    IPodTouchLCDState *s = (IPodTouchLCDState *)opaque;
    /* Raise every frame: gating on the render register loses the interrupt
     * permanently when a framebuffer handover clears it. */
    qemu_irq_raise(s->irq);

    timer_mod(s->refresh_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND / 60);//LCD_REFRESH_RATE_FREQUENCY);
}


static Property ipod_touch_lcd_properties[] = {
    DEFINE_PROP_BOOL("bezel", IPodTouchLCDState, bezel_enabled, true),
    DEFINE_PROP_END_OF_LIST(),
};

static void ipod_touch_lcd_realize(DeviceState *dev, Error **errp)
{
    IPodTouchLCDState *s = IPOD_TOUCH_LCD(dev);
    s->con = graphic_console_init(dev, 0, &gfx_ops, s);

    s->bezel_btn_held = BEZEL_BTN_NONE;
    if (s->bezel_enabled) {
        unsigned long dlen = BEZEL_RGBA_LEN;
        s->bezel_rgba = g_malloc(BEZEL_RGBA_LEN);
        if (uncompress(s->bezel_rgba, &dlen, bezel_rgba_z,
                       BEZEL_RGBA_Z_LEN) != Z_OK || dlen != BEZEL_RGBA_LEN) {
            warn_report("iPod Touch: failed to decompress bezel image, "
                        "falling back to plain display");
            g_free(s->bezel_rgba);
            s->bezel_rgba = NULL;
            s->bezel_enabled = false;
        }
    }

    if (s->bezel_enabled) {
        qemu_console_resize(s->con, BEZEL_WIDTH, BEZEL_HEIGHT);
        s->fb_surface = qemu_create_displaysurface(320, 480);
    } else {
        qemu_console_resize(s->con, 320, 480);
    }

    // add mouse handler
    qemu_add_mouse_event_handler(ipod_touch_lcd_mouse_event, s, 1, "iPod Touch Touchscreen");

    // initialize the refresh timer
    s->refresh_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, refresh_timer_tick, s);
    timer_mod(s->refresh_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND / LCD_REFRESH_RATE_FREQUENCY);
}

static void ipod_touch_lcd_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(sbd);
    IPodTouchLCDState *s = IPOD_TOUCH_LCD(dev);

    memory_region_init_io(&s->iomem, obj, &lcd_ops, s, "lcd", 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void ipod_touch_lcd_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ipod_touch_lcd_realize;
    device_class_set_props(dc, ipod_touch_lcd_properties);
}

static const TypeInfo ipod_touch_lcd_info = {
    .name          = TYPE_IPOD_TOUCH_LCD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodTouchLCDState),
    .instance_init = ipod_touch_lcd_init,
    .class_init    = ipod_touch_lcd_class_init,
};

static void ipod_touch_machine_types(void)
{
    type_register_static(&ipod_touch_lcd_info);
}

type_init(ipod_touch_machine_types)
