#include "a3.h"
#include "dwc3.h"
#include "dwc3_regs.h"
#include "io.h"
#include "led.h"
#include "timer.h"
#include "usb.h"
#include "usb_phy.h"

/*
 * Reached from the vector table in start.S.  Never returns: the point is to
 * put the exception type on the lamp instead of letting the BootROM's handler
 * swallow the fault behind an anonymous 2-short pattern.
 *
 *   2 undefined   4 prefetch abort   5 data abort   6 IRQ   7 FIQ
 */
void __attribute__((used)) exc_report(u32 code)
{
    u32 round;

    __asm volatile ("cpsid if" ::: "memory");

    /*
     * NOP-timed on purpose: if the ROM counter has stopped, mdelay() would
     * freeze here and the exception would be invisible -- which is exactly the
     * "silent death" this table exists to rule out.
     */
    for (;;) {
        u32 n = code ? code : 1u;

        while (n--) {
            u32 i;

            led_on();
            for (i = 0; i < 3000000u; i++)
                __asm volatile ("nop");
            led_off();
            for (i = 0; i < 3000000u; i++)
                __asm volatile ("nop");
        }

        for (round = 0; round < 12000000u; round++)
            __asm volatile ("nop");
    }
}

/*
 * Boot order matters:
 *
 *   1. blink twice before touching anything else, so "no light at all"
 *      unambiguously means the ROM never jumped to the payload (the A3 error
 *      codes below are blinking, not silence);
 *   2. verify the container -- a truncated image must not reach the USB code,
 *      because a half-written payload would otherwise fail in a way that
 *      looks like a USB problem;
 *   3. wait 300 ms for the host to release D-.  The carrier stops 0.1 s after
 *      the last record, and the PHY must not claim D- before then.  One blink
 *      marks the line going idle; a fixed delay is used because polling the
 *      line is the one thing here that has never been proven from payload
 *      context;
 *   4. power the PHY and bring the gadget up immediately, while the camera is
 *      still on the injector.  This follows the verified A6700 flow: the
 *      device controller is left running with no VBUS, and the later host
 *      attach is what makes it enumerate.  Do not sample VBUS before the PHY
 *      is up -- that status window is not valid yet, and waiting for an edge
 *      there only adds a timeout that fires while the camera is still on the
 *      bench;
 *   5. initialise the core and arm ep0 before raising D+ (dwc3_connect), so
 *      the first SETUP is never missed;
 *   6. three slow blinks mark USB ready.  Only now unplug the whole injector
 *      and move the PC data cable to the camera (the PC end stays put);
 *   7. hand over to fastboot once the host selects the configuration.
 *
 * Every failure parks on a distinct light code, since there is no console.
 * The A3 codes use led_raw() so they are 3 long + n short: led_code() would
 * always produce "1 long + n short", and the BootROM itself blinks 0x2004
 * (1 long + 4 short) and 0x2005 (1 long + 5 short) for loader failures, so
 * the two cannot be told apart (see a3.c for the error numbering):
 *   3 long + 2 short   A3 magic
 *   3 long + 3 short   A3 header fields
 *   3 long + 4 short   A3 header CRC
 *   3 long + 5 short   A3 payload CRC
 *   3 long + 6 short   A3 image size
 *   0x10        DWC3 soft reset / core init failed
 *   0x11        ep0 endpoint command failed
 *   0x12        DWC3 never left the halted state after RUN_STOP
 */

#if defined(PROBE) || defined(PROBE2) || defined(PROBE3)
/*
 * Shared hardware for the probe builds.  Lamp language is short blinks only
 * (250 ms on / 250 ms off), one group per step, 1500 ms of dark between
 * groups so "1 2 3 4" reads as four groups instead of one long run.
 *
 * Polarity: led_on() lights the lamp (0xF2910000+0x48), led_off() kills it
 * (+0x44).  See led.c -- the first version had these swapped and every gap
 * looked like a steady ON.
 */
static void probe_blink(u32 count)
{
    u32 i;

    for (i = 0; i < count; i++) {
        led_on();
        mdelay(250);
        led_off();
        mdelay(250);
    }
}

/* Volatile so -O2 cannot drop a load that has no other consumer. */
static volatile u32 probe_val[4];
#endif

#ifdef PROBE2
/*
 * probe2: the clkrst3 half of §9.18.3, then re-read the two windows that the
 * read-only probe (probe) could not reach.
 *
 * probe already proved, with no writes at all:
 *   0xF2900034 (clkrst3 +0x34)  readable
 *   0xF29000C4 (clkrst3 +0xC4)  readable
 *   0xF0220000 (add)            read hangs  <-- stopped here
 *
 * This build performs the four opening writes that the kernel's fusb_otg
 * driver does before touching any DWC3 register, then retries add and global:
 *
 *   1 short   about to write otg +0x100 and the three clkrst3 registers
 *   2 short   writes done (and 110 ms settled), about to read 0xF0220000
 *   3 short   add answered, about to read 0xF020C100
 *   4 short   global answered too -- the window is open
 *
 * Stop point = the step that hung:
 *   1 short  a clkrst3/otg write itself aborts (would contradict probe)
 *   2 short  clkrst3 is open but add still dead -> add has its own parent
 *            clock/power gate, keep looking upward
 *   4 short  go straight to the full §9.18.3 sequence
 */
int main(void)
{
    int err;

    __asm volatile ("cpsid i" ::: "memory");

    led_prepare();
    led_off();
    mdelay(500);

    err = a3_verify();
    if (err)
        led_raw(0x4000u | (0x01u + (u32)err));

    probe_blink(1);
    mdelay(1500);

    /* §9.18.3, first four writes, verbatim order. */
    setbits32(0xF2920100u, 0x00010000u);    /* otg  +0x100 |= 0x10000 */
    write32(0xF29000C4u, 0x00000200u);      /* clkrst3 +0xC4 = assert  */
    write32(0xF2900034u, 0x00004000u);      /* clkrst3 +0x34 = clock   */
    write32(0xF29000C4u, 0x00000400u);      /* clkrst3 +0xC4 = release */
    dsb();
    udelay(110000);                         /* the kernel waits ~107.4 ms */

    probe_blink(2);
    mdelay(1500);
    probe_val[0] = read32(0xF0220000u);

    probe_blink(3);
    mdelay(1500);
    probe_val[1] = read32(0xF020C100u);

    mdelay(1000);
    probe_blink(4);

    for (;;)
        ;
}
#elif defined(PROBE3)
/*
 * probe3: is the DWC3 TX/RX FIFO configured at all?
 *
 * Every observation so far fits "SETUP (OUT) arrives, the data stage's IN
 * transfer never leaves": the host sees D+ and asks for the device
 * descriptor, then gives up with DEVICE_DESCRIPTOR_FAILURE.  The one piece of
 * DWC3 bring-up this payload does not perform is GTXFIFOSIZ/GRXFIFOSIZ -- and
 * the A7C kernel never writes them either (notes M5-3), which means the
 * bootloader is what normally gets them right.  On the UART-boot path no
 * bootloader has ever run.
 *
 * Read-only apart from the four window-opening writes probe2 established.
 *
 *   1 short   injection verified, about to open the window
 *   2 short   window open, the DWC3 global block answers
 *   3 short   GSNPSID looks like a Synopsys id (0x5533xxxx)
 *   4 short   GTXFIFOSIZ0 != 0   -- EP0 owns a TxFIFO
 *   5 short   GRXFIFOSIZ0 != 0   -- there is an RxFIFO
 *   6 short   all of the above; the FIFO is not the problem
 *
 * Read it backwards: the last group seen is the last check that passed, so
 * "3 short and nothing after it" means GTXFIFOSIZ0 is zero.
 */
int main(void)
{
    int err;

    __asm volatile ("cpsid i" ::: "memory");

    led_prepare();
    led_off();
    mdelay(500);

    err = a3_verify();
    if (err)
        led_raw(0x4000u | (0x01u + (u32)err));

    probe_blink(1);
    mdelay(1500);

    /* probe2's four opening writes, verbatim. */
    setbits32(0xF2920100u, 0x00010000u);    /* otg  +0x100 |= 0x10000 */
    write32(0xF29000C4u, 0x00000200u);      /* clkrst3 +0xC4 = assert  */
    write32(0xF2900034u, 0x00004000u);      /* clkrst3 +0x34 = clock   */
    write32(0xF29000C4u, 0x00000400u);      /* clkrst3 +0xC4 = release */
    dsb();
    udelay(110000);

    probe_val[0] = read32(0xF020C100u);     /* global window */
    probe_blink(2);
    mdelay(1500);

    probe_val[1] = read32(0xF020C120u);     /* GSNPSID */
    if ((probe_val[1] & 0xFFFF0000u) != 0x55330000u) {
        for (;;)
            ;
    }
    probe_blink(3);
    mdelay(1500);

    probe_val[2] = read32(0xF020C300u);     /* GTXFIFOSIZ(0) */
    if (!probe_val[2]) {
        for (;;)
            ;
    }
    probe_blink(4);
    mdelay(1500);

    probe_val[3] = read32(0xF020C380u);     /* GRXFIFOSIZ(0) */
    if (!probe_val[3]) {
        for (;;)
            ;
    }
    probe_blink(5);
    mdelay(1500);

    probe_blink(6);

    for (;;)
        ;
}
#elif defined(PROBE)
/*
 * Read-only probe build (-Probe, see §9.18.5 in
 * a7c_payload_phased_conclusions_20261020.md).
 *
 * Purpose: decide whether the clkrst3 domain (0xF2900000) is already powered
 * on the UART-boot path, and at which of the four addresses the bus first
 * stops responding.  Nothing here writes a USB register -- the only stores are
 * the lamp (0xF2910000 / 0xF2912000).
 *
 *   1 short, then read   0xF2900034   clkrst3 +0x34 (clock gate)
 *   2 short, then read   0xF29000C4   clkrst3 +0xC4 (reset assert/release)
 *   3 short, then read   0xF0220000   add window, first word
 *   4 short, then read   0xF020C100   global window, first word
 *   5 short              all four reads returned
 *
 * How to read it: the count that stops the sequence is the address that hung.
 * A3 verification failures still use the 3 long + n short codes so they cannot
 * be mistaken for a probe point.
 */
int main(void)
{
    int err;

    __asm volatile ("cpsid i" ::: "memory");

    led_prepare();              /* leaves the lamp dark */
    led_off();
    mdelay(500);

    err = a3_verify();
    if (err)
        led_raw(0x4000u | (0x01u + (u32)err));

    probe_blink(1);
    mdelay(1500);
    probe_val[0] = read32(0xF2900034u);

    probe_blink(2);
    mdelay(1500);
    probe_val[1] = read32(0xF29000C4u);

    probe_blink(3);
    mdelay(1500);
    probe_val[2] = read32(0xF0220000u);

    probe_blink(4);
    mdelay(1500);
    probe_val[3] = read32(0xF020C100u);

    mdelay(1000);
    probe_blink(5);

    for (;;)
        ;
}
#elif defined(SELFTEST)
/*
 * Diagnostic build (-DSELFTEST): verify the container and report, nothing
 * else. No USB register is touched, so this isolates the loader / A3 CRC path
 * from the USB bring-up when bringing the payload up on hardware.
 *
 * The failure code deliberately does NOT use led_code(): that always produces
 * "1 long + n short", and the BootROM itself blinks 0x2004 (1 long + 4 short)
 * for several loader failures, so "1 long 4 short" cannot be attributed to the
 * payload. Failures are reported as 3 long + (1 + error) short instead:
 *   3 long 2 short   A3 magic
 *   3 long 3 short   A3 header fields
 *   3 long 4 short   A3 header CRC     <- header word at +0x1c was damaged
 *   3 long 5 short   A3 payload CRC
 *   3 long 6 short   A3 image size
 *   1 long 48 short  verify passed
 */
int main(void)
{
    int err = a3_verify();

    if (err)
        led_raw(0x4000u | (0x01u + (u32)err));

    led_code(0x30);

    for (;;)
        ;
}
#else
/*
 * Three slow blinks.  Deliberately slower than the quick 2+1 entry blinks so
 * the report "USB initialised" cannot be mistaken for a run that died halfway
 * through the bring-up (which leaves the lamp off, or on if it died during an
 * on-phase).
 */
static void led_ready(void)
{
    u32 i;

    for (i = 0; i < 3; i++) {
        led_on();
        mdelay(400);
        led_off();
        mdelay(300);
    }
}

#ifdef BEAT
/*
 * -Beat 对照构建（一次性诊断，2026-10-22）。
 *
 * 目的：把"插上电脑后灯恒定长亮"这件事和 USB 事件处理彻底切开。
 * 这个构建 connect 之后**不再访问任何 USB 寄存器**，只做 NOP 计时的心跳闪灯。
 *
 *   插线后心跳继续  -> CPU 活着，灯被别的东西（VBUS/硬件）接管了
 *   插线后心跳停止  -> 第一次总线复位本身就把 CPU 弄停了，与事件代码无关
 *
 * NOP 计时是刻意的：万一 timer 也停了，心跳照样能跑，结论不会被计时器污染。
 */
#define BEAT_NOP 4000000u

static void beat_delay(void)
{
#ifdef BEAT_TIMER
    /* 对照：同样的心跳，但节拍改由 timer 驱动。
     * 若插线后心跳停住，就证明 timer 在插线瞬间死了。 */
    mdelay(150);
#else
    u32 i;

    for (i = 0; i < BEAT_NOP; i++)
        __asm volatile ("nop");
#endif
}

static void beat_forever(void)
{
    for (;;) {
        led_on();
        beat_delay();
        led_off();
        beat_delay();
        beat_delay();
        beat_delay();
    }
}
#endif

#ifdef ENUM
/*
 * Version marker as two digit groups: tens, a pause, then units.
 * 12 -> "1 short, 700 ms dark, 2 short" (~1.3 s).
 *
 * The old form was one 500 ms long flash per round, which cost 12 s of
 * bring-up by round 61 and would only get worse.  The two groups are short
 * blinks (100 ms on) so they cannot be confused with led_ready()'s three slow
 * 400 ms blinks, and the 700 ms inter-group gap is 7x the 100 ms intra-group
 * gap, so the grouping survives being watched by eye.
 *
 * No division: v is a constant here, but the image is -nostdlib and a runtime
 * /10 would pull in __aeabi_uidivmod.  Subtract instead.
 */
static void led_version(u32 v)
{
    u32 tens = 0;

    while (v >= 10u) {
        v -= 10u;
        tens++;
    }

    if (tens) {
        led_blink(tens);
        mdelay(700);
    }
    led_blink(v ? v : 1u);
}
#endif

int main(void)
{
    int err;

    /* The ROM may hand over with interrupts enabled and there is no vector
     * table of our own: leave them masked so a stray IRQ cannot vector into
     * stale ROM state and park the CPU mid-blink.  FIQ is masked as well --
     * it used to be the one vector still routed to the ROM's handler. */
    __asm volatile ("cpsid if" ::: "memory");

    led_prepare();
#ifdef ENUM
    /* Three, a full second, four: no previous build and no status code can
     * be mistaken for this, so the bench can tell at a glance which image is
     * running before it interprets anything else.
     *
     * 2026-10-23 第二十二轮：第二组由 3 改成 4（"3-4"）。上一版是 "3-3"，
     * 现场无法分辨新旧镜像，这一下就是版本标记。
     * 第二十三轮：4 → 5（"3-5"），站点编号整体重排，见 usb.c 的站点表。
     * 第二十四轮：5 → 6（"3-6"），仪器换成"阶段归属记账 + 静默后整段播报"。
     * 这一版插线后**不会**再出现"闪一下就停"的站点，读数形态与上一版完全不同。
     * 第二十五轮：6 → 7（"3-7"），13/14 的 HWO 采样扩到数据阶段入口与复位前，
     * 心跳改成"静默后才闪"。
     * 第二十六轮：7 → 8（"3-8"），账本改为**倒序播报**（最新一条先播），
     * 防止播报被相机重启打断时丢掉尾部的关键记录。
     * 第二十七轮：8 → 9（"3-9"），账本加**噪声屏蔽**：见过第一个 SETUP 完成
     * （3）之后不再记复位/连接/装 SETUP TRB，读数里只剩 3/5/7/10/12/13/14/15。
     * 第二十八轮：版本组不再用"数几下"，改成**一个长闪**（亮 500 / 灭 500）。
     * led_blink() 是亮 100 / 灭 100（5 Hz），9 下整组只有 1.8 s，现场两次都被
     * 读成 7 下；长闪与短闪的宽度差 5 倍，一眼可分，且只需确认"有 / 没有"。
     * 今后每变一次版本，长闪次数 1 → 2 → 3；超过 3 再换编码。
     * 第二十九轮：长闪 1 → 2，启动灯语 "3 短 → 2 长 → 1 短"。本轮把 ep0 的
     * SETUP / DATA / STATUS 三个控制阶段各分到一条独立的 TRB 槽，验证读数里
     * 的 13 究竟是"控制器没交回 TRB"还是"被下一阶段写进同一个 TRB"。
     * 第三十轮：长闪 2 → 3（"3 短 → 3 长 → 1 短"）。账本里 10 以上的码改成
     * 长闪前缀（13 → 1 长 3 短），本组必须换号才能分辨新旧镜像。
     * 第三十一轮：长闪 3 → 4。ep0 状态阶段改纯反应式（状态 TRB 只在
     * NRDY(控制阶段=2) 装，删掉 11 守卫），状态端点改由 SETUP 的
     * bmRequestType 定死（零长度请求用 EP0_IN）。
     * 第三十三轮：长闪 4 → 5（"3 短 → 5 长 → 1 短"）。上一轮忘了升版本，
     * 与第三十一轮同为 4 长，读数又相同，无法排除跑的是旧镜像；这一轮
     * 必须能靠灯语和上一轮区分开。本轮内容是 7 级台阶探针 + 去掉 SETUP
     * 分支的 dsb() + SET_ADDRESS 延迟到状态阶段之后落 DCFG。
     * 第三十四轮：长闪 5 → 6（"3 短 → 6 长 → 1 短"）。SETUP 分支的 ctrl
     * 解析改成逐字节 volatile 读 + 逐字段 volatile 写（消掉两条非对齐字
     * 访问），并在 dmb 与解析之间加台阶 8。
     * 第三十五轮：长闪 6 → 7（"3 短 → 7 长 → 1 短"）。SET_ADDRESS 恢复
     * 当场写 DCFG（控制器在状态阶段结束时自行切换）；撤掉 8 级一次性
     * 台阶，换成可重复的每请求播报「2 长 + n 短」。
     * 第三十六轮：长闪 7 → 8（"3 短 → 8 长 → 1 短"）。枚举事件路径**零灯语**：
     * 撤掉指纹和每请求播报，只留一个字节的账本记录；账本改为只播最后 8 条；
     * 泵不再抢占灯态；枚举成功后常亮。
     * 第三十七轮：长闪 8 → 9（"3 短 → 9 长 → 1 短"）。播报改成三字段短灯语
     * （SETUP 数 / 完成事件数 / 武装数），节拍改用 mdelay；心跳只在无待播内容时闪。
     * 第三十八轮：长闪 9 → 10（"3 短 → 10 长 → 1 短"）。只改事件消费长度
     * （端点事件按 8 字节消费），灯语结构不动，便于与 9 长那轮直接对照。
     * 第三十九轮：长闪 10 → 11（"3 短 → 11 长 → 1 短"）。撤销 8 字节消费（回到
     * 固定 4 字节），播报字段换成 武装数 / 已释放数 / 完成事件数。
     * 第四十轮：长闪 11 → 12（"3 短 → 12 长 → 1 短"）。控制传输数据/状态 TRB
     * 一律装物理端点 0；播报字段换成 武装失败数 / 完成事件数 / NRDY 端点号+1，
     * 并改为提早就播、只播一次。
     * 第四十一轮：长闪 12 → 13（"3 短 → 13 长 → 1 短"）。数据/状态 TRB 改为
     * 装**事件点名的物理端点**；播报第 3 段换成复位时 DATA TRB 的 HWO 归宿。
     * 第四十二轮：长闪 13 → 14（"3 短 → 14 长 → 1 短"）。STARTTRANSFER 的
     * PARAM 字段带上 SETTRANSFRESOURCE 返回的资源索引；播报第 3 段换成
     * ep1 的资源索引。
     * 第四十三轮：长闪 14 → 15（"3 短 → 15 长 → 1 短"）。TXFIFOSZ 口径由
     * 2^(code+8) 改成 2^(code+3)（512 B ⇒ code 6），并在每次总线复位后、
     * DEPSTARTCFG 之前重写 GTXFIFOSIZ(0)/GRXFIFOSIZ(0)；播报第 3 段换成
     * 读回的 TxFIFO size code（期望 6）。
     * 第四十四轮：长闪 15 → 16（"3 短 → 16 长 → 1 短"）。第 43 轮读数
     * 1长-1长-6 确认 FIFO 尺寸假设排除，播报第 3 段换回复位时 DATA 槽 TRB
     * 的 HWO 归宿（1 = 控制器没取过，2 = 取了没走完）。
     * 第四十五轮：长闪 16 → 17（"3 短 → 17 长 → 1 短"）。第 44 轮读数
     * 1长-1长-1 把分叉定死在"控制器从不取 EP0_IN 的 DATA TRB"；本轮在每次
     * 武装数据/状态阶段前重新 DEPCFG + SETTRANSFRESOURCE 并写回 DALEPENA，
     * STARTTRANSFER 改用新拿到的资源索引。
     * 第四十六轮：长闪 17 → 18（"3 短 → 18 长 → 1 短"）。第 45 轮读数与
     * 第 44 轮逐字段相同，rearm 方案证伪；本轮把 ep0 整条路径退回 ZV-E10
     * 已验证形态：三阶段共用一张 TRB、STARTTRANSFER 不带 PARAM、不再清
     * GUSB2PHYCFG 挂起位、不再重写 FIFO。
     * 第四十九轮：长闪 18 → 19（"3 短 → 19 长 → 1 短"）。播报由 4 段扩到
     * 6 段，新增 A（NRDY 点名的物理端点号 + 1）与 B（DALEPENA 低两位 + 1），
     * 只读取、不改枚举行为。
     * 第五十轮：长闪 19 → 1。用户决定版本号从 1 重新计数，只要能区分开即可
     * （19 长闪约 19 s，起播阶段太长）。与第 28 轮同为 1 长，但播报形态不同：
     * 那一版是账本回放，本版固定 6 段。本轮第 5、6 段换成 SETUP 完成数与
     * 数据 TRB 回读比对值。
     * 第五十一轮：长闪 1 → 2（"3 短 → 2 长 → 1 短"）。第 50 轮读数
     * 0-0-1-1-3-1 证实 SETUP 链成立、控制器不取 ep1 的 DATA TRB；段 6（TRB
     * 回读）在"没人碰"前提下无信息量，退役。本轮第 6 段换成 DATA 阶段
     * XFERNOTREADY 次数，用来分辨"控制器不轮询 ep1"与"轮询但拒绝 TRB"。
     * 第五十二轮：长闪 2 → 3（"3 短 → 3 长 → 1 短"）。第 51 轮读数
     * 0-0-1-1-6-9 以段 6 封顶定死机理 B（控制器反复轮询 ep1、每次判为
     * 不可执行）⇒ EP0 路上唯一未验证的资源是 TxFIFO 分区。本轮在
     * usb_phy_init() 之后、任何 DWC3 复位之前把 boot 链留下的
     * GTXFIFOSIZ(0)/GRXFIFOSIZ(0) 存下来，全局复位后原样写回，
     * 不再用硬编码的 128 words。
     * 第五十三轮：长闪 3 → 4（"3 短 → 4 长 → 1 短"）。第 52 轮读数
     * 0-0-1-1-5-9 与第 51 轮等价 ⇒ boot 分区没有带来任何改变，但无法区分
     * "捕获被判无效、实际退回 128 words"与"boot 值被采纳却同样无效"。
     * 本轮只加探针：段 4 从 CONNECTSPD 换成实际采用的分区 TXFIFOSZ 编码。
     * 第五十四轮：长闪 4 → 5（"3 短 → 5 长 → 1 短"）。第 53 轮读数
     * 0-0-1-0-3-9 证明 boot 链在开窗时刻没留下可用分区，第 52 轮实际写的是
     * 硬编码 128 words ⇒ "接续 boot 分区"作废，FIFO 重新成为头号嫌疑。
     * 本轮只改一处：TXFIFOSZ 口径由 2^(code+3)（512 B ⇒ code 6）换回
     * 2^(code+8)（512 B ⇒ code 1），让 DEPTH=128 / SZ=1 两字段自洽。
     * 第五十五轮：长闪 5 → 6（"3 短 → 6 长 → 1 短"）。第 54 轮读数
     * 0-0-1-0-2-9 与第 53 轮只差段 5 的窗口波动，SZ 6→1 行为毫无变化
     * ⇒ FIFO 分区（容量/编码）这条线出局。主矛盾回到"ep1 的 CONTROL_DATA
     * TRB 从未被控制器取走"。本轮改的是装 TRB 的时机：SETUP 完成回调里
     * 直接装数据 TRB（对齐 Linux dwc3），NRDY(DATA) 不再负责首次武装，
     * 只做补装与计数。
     * 第五十六轮：长闪 6 → 7（"3 短 → 7 长 → 1 短"）。第 55 轮读数
     * 0-0-1-0-5-9 说明两次 DEPSTRTTRANSFER 都被接受、数据 TRB 仍没人取
     * ⇒ 装 TRB 的时机不是根因。本轮改的是装到哪里：SETUP/DATA/STATUS 重新
     * 各占一格，且三格在内存里按消费顺序紧邻（DATA 恰在 SETUP 的下一格），
     * 用来判决"控制器取 TRB 的指针已推进到下一格"这个假设。
     * 第五十七轮：长闪 7 → 8（"3 短 → 8 长 → 1 短"）。第 56 轮读数与第 55
     * 轮逐字段相同 ⇒ 换槽也无效，且预装把 NRDY 这条唯一的参考武装点挡住了。
     * 本轮回到 ZV-E10 / 7M4 的武装点：SETUP 完成只解析、数据/状态 TRB 一律
     * 等 NRDY 再装（删掉预装与 ep0_data_armed 守卫）；段 4 换成 NRDY 里
     * 成功装出 DATA TRB 的次数，用来确认这条路径这次真的执行了。
     * 第五十八轮：长闪 8 → 9（"3 短 → 9 长 → 1 短"）。第 57 轮读数
     * 0-0-1-9-5-9 说明武装路径已跑满 9 次、命令全成功，TRB 仍无人取 ⇒
     * 武装时机/守卫/槽位三线全部关闭。本轮改的是核初始化：撤掉 A7C 独有的
     * GCTL.CORESOFTRESET 脉冲（两份能枚举的实现都没有它），让 ROM 留下的
     * FIFO 分区/GCTL 状态不被洗掉，其余一律不动。
     * 第五十九轮：长闪 9 → 10（"3 短 → 10 长 → 1 短"）。第 58 轮撤掉核复位后
     * 现象未变，说明病灶不在核复位本身。本轮做真正的单变量：用 dwc3.c 的
     * DWC3_FIFO_WRITE=0 停写 GTXFIFOSIZ/GRXFIFOSIZ（"必须写 FIFO"从未被
     * A/B 验证过），同时把下方那两个会挂死自检的 led_raw 换成不阻塞的
     * led_blink，让观测链能走到 dwc3_connect()。
     * 第六十轮：长闪 10 → 11（"3 短 → 11 长 → 1 短"）。第 59 轮完整枚举成功
     * （18d1:4ee0 / ILCE-7C / WinUSB），本轮把胜利配置固化：彻底删除 FIFO
     * 写入与 boot 捕获，去掉 17/18 自检（写已不在，自检失去意义），
     * 并把段间黑隔从 1500 ms 缩到 1000 ms。
     * 第六十一轮：长闪 11 → 12（"3 短 → 12 长 → 1 短"）。第 60 轮两次插拔
     * 均稳定枚举，本轮首次动 bulk 通路：总线复位后重新武装 bulk 端点
     * （此前 SET_CONFIGURATION 只置 pending，而阻塞在 usb_bulk_recv 里的人
     * 永远不会去执行它），并让 usb_bulk_recover() 能收拾任何错误码。
     * 枚举路径一行未改，失败时回滚到 r60 备份即可。
     * 第六十六轮（O-5）：版本灯语由「12 个 500 ms 长闪」改成**两位数分组**
     * （十位 1 短 → 黑 700 ms → 个位 2 短），起播由约 13.6 s 降到约 2.9 s；
     * 灯语形态整体变化，反而比只改次数更容易与旧镜像分辨。 */
    led_blink(3);
    mdelay(1000);
    led_version(12);
#else
    led_blink(2);               /* payload is executing */
#endif

    err = a3_verify();

    if (err)
        led_raw(0x4000u | (0x01u + (u32)err));

    /* Let the injector release D-.  The host drops the carrier 0.1 s after the
     * last record, so a fixed 300 ms covers that tail with margin; polling the
     * line through [0xF2915010] is not worth the risk (that read has never been
     * proven from payload context and an abort here strands the CPU). */
    mdelay(300);
    led_blink(1);               /* carrier released, D- is idle */

    usb_phy_init();

    err = dwc3_core_init();
    if (err == DWC3_ERR_NPSID)
        /* 1 long + 19 short：GSNPSID 不是 0x5533xxxx，说明 0xF020C100
         * 窗口没开或基址不对 —— 与 0x10（软复位/初始化失败）分开报。 */
        led_code(0x13);
    else if (err != DWC3_OK)
        led_code(0x10);

    if (usb_gadget_init() != 0)
        led_code(0x11);

    /* Separate the ready marker from the earlier blinks, otherwise the
     * 1-blink (D- idle) and the 3-blink (USB ready) read as one long run. */
    mdelay(700);
    led_ready();                /* ep0 armed, about to attach */

#ifdef HOLD
    /* Stay lit from here on.  Every report in this build is a dark gap against
     * this baseline, so a lamp that never settles into a steady on -- or that
     * keeps blinking after the reports would have run out -- is proof that
     * something other than this payload is driving it. */
    led_on();
#endif

    /*
     * Attach last.  The host resets the bus within ~100 ms of seeing D+ and
     * expects a device descriptor immediately after that reset, and a bus
     * reset discards the SETUP TRB armed by ep0_init().  With RUN_STOP raised
     * before the light show, the reset landed in a 2.8 s window with no event
     * pumping: the core's reset was never processed, ep0 was never re-armed
     * and the host reported "device descriptor request failed".  Attaching
     * here means usb_event_pump() below is already running when the reset
     * arrives.
     */
    if (dwc3_connect() != DWC3_OK)
        led_code(0x12);

#ifdef TRACE
    /* 4 quick blinks: dwc3_connect() came back, so the DCTL write and the
     * DSTS poll both answered and the CPU is still ours.  No 4-blink after the
     * 3 long means it wedged inside connect (or was reset from there). */
    led_blink(4);
#endif

#ifdef BEAT
    /* 从这里开始不再碰 USB：见 beat_forever() 注释。 */
    beat_forever();
#endif

    usb_gadget_run();

    for (;;)
        ;
}
#endif /* SELFTEST */
