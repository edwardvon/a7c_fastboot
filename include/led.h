#ifndef A7C_LED_H
#define A7C_LED_H

#include "io.h"

/*
 * Hand a status code to the BootROM's own LED reporter (0xFFFF0D0B, Thumb).
 * It never returns, so this is only for terminal states: fatal init errors and
 * the halt after `reboot`.
 */
void led_code(u32 code);

/*
 * Same reporter, but the caller supplies the raw argument:
 *
 *   0x2000 | n   1 long  + n short   (what led_code() always produces)
 *   0x3000 | n   2 long  + n short   (the BootROM's own loader errors)
 *   0x4000 | n   3 long  + n short   (free: the ROM never emits these)
 *
 * Disassembly of 0xFFFF0D0A shows the ROM only splits the argument into
 * ((value >> 12) & 0xF) - 1 long flashes and value & 0xFFF short flashes, with
 * no range check, so any 0xNxxx value is legal.
 */
void led_raw(u32 value);

/*
 * Non-blocking LED control, for progress while the payload keeps running.
 * The register sequence is copied verbatim from the reporter at 0xFFFF0D0A
 * (port 0 pin 0 plus the 0xF2912000 auxiliary block), so it drives the same
 * lamp without entering the reporter's endless blink loop.
 */
void led_prepare(void);
void led_on(void);
void led_off(void);
void led_blink(u32 count);

#endif /* A7C_LED_H */
