#include "led.h"

#define ROM_LED_REPORT  0xFFFF0D0Bu   /* Thumb, never returns */

typedef void (*rom_led_fn)(u32);

void led_code(u32 code)
{
    /*
     * Verified encoding from tmp/ret_test.S: r0 = 0x2000 | code, then a tail
     * call into the BootROM's reporter. The function does not come back, which
     * is why callers only use it for terminal states.
     */
    ((rom_led_fn)ROM_LED_REPORT)(0x2000u | (code & 0xFFu));

    for (;;)
        ;
}

void led_raw(u32 value)
{
    ((rom_led_fn)ROM_LED_REPORT)(value);

    for (;;)
        ;
}

/*
 * Same setup the reporter performs before its first flash:
 *   0xFFFF0D0A  [0xF2910000+0x44]=1  [+0x28]=1  [+0x44]=1
 *               [0xF2912000+0x24]=1  [+0x34]=1  [+0x44]=1
 *               [0xF2910000+0x04]=1
 * Calling this leaves the lamp dark, so the first led_on() is visible.
 *
 * Polarity (corrected 2026-10-22 from a hardware run): writing +0x44 turns
 * the lamp OFF and +0x48 turns it ON.  The first version of this file had the
 * two swapped, which made every "gap" between blinks look like a steady ON --
 * that is exactly what was observed on the bench ("1 short, 1 long" per group
 * instead of "n short, then dark").  The ROM's own codes look right because
 * the reporter also lights the lamp through +0x48.
 */
#define LED_PORT 0xF2910000u
#define LED_AUX  0xF2912000u

void led_prepare(void)
{
    write32(LED_PORT + 0x44u, 1u);
    write32(LED_PORT + 0x28u, 1u);
    write32(LED_PORT + 0x44u, 1u);
    write32(LED_AUX  + 0x24u, 1u);
    write32(LED_AUX  + 0x34u, 1u);
    write32(LED_AUX  + 0x44u, 1u);
    write32(LED_PORT + 0x04u, 1u);
}

void led_on(void)
{
    write32(LED_PORT + 0x48u, 1u);
}

void led_off(void)
{
    write32(LED_PORT + 0x44u, 1u);
}

void led_blink(u32 count)
{
    while (count--) {
        led_on();
        mdelay(100);
        led_off();
        mdelay(100);
    }
}
