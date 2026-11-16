#ifndef A7C_TIMER_H
#define A7C_TIMER_H

#include "io.h"

/*
 * Free-running counter inside the A7C timer block, 4 MHz (one tick per
 * 250 ns).  Derived from the ROM carrier timing: the slow block is
 * 909 ticks and lasts 229.2 us, so 1 tick = 0.25 us.  The ROM's own blink
 * code at 0xFFFF0D0A also uses 2,000,000 ticks per long phase, which is the
 * observed 0.5 s flash.
 */
#define TIMER0_BASE    0xF2403000u
#define TIMER0_COUNTER (TIMER0_BASE + 0x0Cu)
#define TIMER0_HZ      4000000u
#define TIMER0_TICKS_PER_US (TIMER0_HZ / 1000000u)

u32 timer_ticks(void);

#endif /* A7C_TIMER_H */
