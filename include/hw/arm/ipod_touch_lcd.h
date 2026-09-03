#ifndef IPOD_TOUCH_LCD_H
#define IPOD_TOUCH_LCD_H

#include <math.h>
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/arm/ipod_touch_multitouch.h"
#include "hw/arm/ipod_touch_gpio.h"
#include "ui/console.h"

#define TYPE_IPOD_TOUCH_LCD                "ipodtouch.lcd"
OBJECT_DECLARE_SIMPLE_TYPE(IPodTouchLCDState, IPOD_TOUCH_LCD)

#define LCD_REFRESH_RATE_FREQUENCY 10

typedef struct IPodTouchLCDState
{
    SysBusDevice parent_obj;
    MemoryRegion *sysmem;
    MemoryRegion iomem;
    QemuConsole *con;
    IPodTouchMultitouchState *mt;
    int invalidate;
    uint8_t brightness;
    MemoryRegionSection fbsection;
    qemu_irq irq;
    uint32_t lcd_con;

    uint32_t w1_display_resolution_info;
    uint32_t w1_framebuffer_base;
    uint32_t w1_hspan;
    uint32_t w1_display_depth_info;

    uint32_t render;

    QEMUTimer *refresh_timer;

    /* bezel ("device skin") support */
    bool bezel_enabled;
    uint8_t *bezel_rgba;     /* decompressed BEZEL_WIDTH*BEZEL_HEIGHT*4 */
    DisplaySurface *fb_surface;  /* scratch 320x480 render target */
    bool bezel_drawn;        /* bezel composited into the surface */
    IPodTouchGPIOState *gpio_state;
    int bezel_btn_held;      /* which bezel button the mouse is holding, -1 none */
} IPodTouchLCDState;

/* bezel buttons */
#define BEZEL_BTN_NONE  (-1)
#define BEZEL_BTN_HOME  0
#define BEZEL_BTN_VOLUP 1
#define BEZEL_BTN_VOLDN 2

void lcd_changebrightness(int brightness);

#endif
