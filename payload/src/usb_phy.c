#include "usb_phy.h"
#include "io.h"

/*
 * A7C USB PHY power-up.
 *
 * This is fusb_otg_resume_initial_set() from the stock kernel (vaddr
 * 0x602b1ce4) transcribed write for write. Unlike the ILCE-7M4 there is no SCU
 * clock block, no TCA handshake and no USB3 PIPE control here: the part is
 * USB 2.0 high speed only, and the Type-C/OTG block is what gates the PHY.
 *
 * The kernel reaches these registers through a MIPI-style wrapper that lives
 * at the DWC3 base; 0xF020C100 is the same `regs` pointer dwc3_probe() keeps.
 *
 * Window opening (added 2026-10-22, see §9.18.3 of
 * a7c_payload_phased_conclusions_20261020.md).  From UART boot nothing has
 * enabled the DWC3 clocks, so the first access to 0xF0220000 / 0xF020xxxx
 * aborts the bus.  The kernel does this in the per-window callback at
 * 0x602B1644 (idx5, clkrst3) before it touches any DWC3 register.  Verified on
 * hardware with the two probe builds:
 *
 *   probe  : reading 0xF0220000 hangs, 0xF2900034 / 0xF29000C4 are readable
 *   probe2 : after the four writes below, 0xF0220000 and 0xF020C100 both read
 *            back, so the sequence really is what opens the windows.
 */
void usb_phy_init(void)
{
    /* OTG block: hold it out of suspend. */
    setbits32(USB_OTG_BASE + 0x100u, BIT(16));

    /* Open clkrst3: assert, clock gate, release, exactly the kernel's order
     * for the three clkrst3 registers.  Everything after this point talks to
     * 0xF02xxxxx, which is unreachable until these writes have landed.
     *
     * The kernel also pokes add+0 (the low four bits) *before* the release;
     * that exact ordering was never exercised by probe2, so it is done after
     * the settle delay here instead -- same bits, only a point that hardware
     * has already proven readable. */
    write32(0xF29000C4u, 0x00000200u);
    write32(0xF2900034u, 0x00004000u);
    write32(0xF29000C4u, 0x00000400u);
    dsb();
    udelay(110000);             /* the kernel waits ~107.4 ms here */

    clrsetbits32(0xF0220000u, 0x0000000Fu, 0x00000003u);

    /* Power the PHY down, then back up, with the kernel's settle time. */
    write32(0xF020C7C4u, 0x400u);
    (void)read32(0xF020C7C4u);
    mdelay(25);

    /*
     * 2026-10-23 第二十轮：删掉三处 ZV-E10（同为 CXD90045、实机枚举成功）
     * 明确不做的写。
     *
     * 原来的顺序是：
     *     GUSB3PIPECTL = 0x11080002      (Sony 值)
     *     GUSB3PIPECTL = 0x00020000      ← 把上面那行整个覆盖掉
     *     GUSB2PHYCFG  = 0x00000040      ← 置 SUSPHY，主动把 PHY 挂起
     *     OTG+0x40/0x50 |= 0x300         ← 计划文件里标着"自拟、语义未知"
     *
     * 对照 ZV-E10 的 usb_phy.c：它只写一次 `GUSB3PIPECTL = 0x11080002`，
     * 不覆盖、不置 SUSPHY，也没有 `0x300` 那两个写；它的注释还专门写着
     * "The driver also parks the PHY in suspend here; a device that is about
     * to be enumerated must not do that."
     *
     * 覆盖 GUSB3PIPECTL 这一条尤其可疑：Sony 值刚写进去就被 0x20000 顶掉，
     * 等于那一步白做，还可能把 PHY 置成某种挂起/低功耗状态。
     */
    write32(0xF020C2C0u, 0x11080002u);

    /* ADPADDR / ADPDATA（RX_SCOPE_LFPS_EN）。ZV-E10 在同一位置成对写入，
     * 这里保留，地址沿用 A7C 内核反汇编得到的 0xF0231080/0x1084。 */
    write32(0xF0231080u, 0x1026u);
    write32(0xF0231084u, 0x1u);

    mdelay(5);
}

u32 usb_otg_status(void)
{
    return read32(USB_OTG_STATUS);
}