/* Central switch for the iPod Touch device models' debug chatter.
 *
 * These devices log with bare printf(), which on Windows goes straight to the
 * console and cannot be redirected. Route it through a macro that is silent
 * unless IPOD_TOUCH_DEBUG is defined (or -DIPOD_TOUCH_DEBUG=1 at build time).
 */
#ifndef IPOD_TOUCH_DEBUG_H
#define IPOD_TOUCH_DEBUG_H

#include <stdio.h>

#ifndef IPOD_TOUCH_DEBUG
#define IPOD_TOUCH_DEBUG 0
#endif

#define printf(...) \
    do { if (IPOD_TOUCH_DEBUG) { fprintf(stderr, __VA_ARGS__); } } while (0)

#endif
