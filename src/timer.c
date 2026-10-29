#include "timer.h"

u32 timer_ticks(void)
{
    return read32(TIMER0_COUNTER);
}

/*
 * 2026-10-23：udelay 原来只靠计数差，定时器一旦不走就永久自旋。
 *
 * 真机读数（指纹 `1→2→5` 之后灯长亮、拔线不灭、重插无反应）把卡点收窄到
 * XFERNOTREADY 路径上的第一次 `dwc3_depcmd()`，而那条路径里唯一没有上限的
 * 等待就是这里。4 MHz 下 250 ns 才走一格，一次 MMIO 读只有几十纳秒，所以
 * 正常运行时绝不会连续 1000 次读回同一个值；连续不变只能说明计数停了。
 */
#define UDELAY_STALL_LIMIT 1000u

void udelay(u32 us)
{
    u32 start = timer_ticks();
    u32 ticks = us * TIMER0_TICKS_PER_US;
    u32 last = start;
    u32 still = 0;

    /* Unsigned difference, so the 32-bit wrap after ~17 minutes at 4 MHz does
     * not matter. */
    for (;;) {
        u32 now = timer_ticks();

        if ((now - start) >= ticks)
            break;

        if (now == last) {
            if (++still >= UDELAY_STALL_LIMIT)
                break;
        } else {
            last = now;
            still = 0;
        }
    }
}

void mdelay(u32 ms)
{
    while (ms--)
        udelay(1000);
}
