#include "dwc3.h"
#include "dwc3_regs.h"
#include "timer.h"

#ifdef ENUM
#include "led.h"

/*
 * 站点灯标（定义在 usb.c）。仅在第一次总线复位之后武装，用来把"卡在
 * 端点命令的哪一步"变成能数出来的闪次。
 */
extern volatile u32 g_site_on;
extern void enum_site(u32 n);
extern void enum_site_once(u32 n);
#endif

/*
 * DWC3 core bring-up. The sequence is the one the stock A7C kernel performs in
 * dwc3_probe(), minus everything that needs the Linux driver model: no TCA
 * handshake (the PHY has none), no USB3 blocks.
 */

/*
 * GEVNTADR holds the event buffer address with its low 12 bits forced to zero,
 * so the buffer has to be 4 KB aligned.  At 64-byte alignment the core wrote
 * events at the start of the containing 4 KB page (0xFE021000) while the
 * payload read them from 0xFE021780 -- i.e. it consumed events from 480
 * entries earlier, which is why events appeared to arrive at random.  Linux
 * never notices because dma_alloc_coherent() hands out page-aligned memory.
 */
static u8 event_buffer[DWC3_EVENT_BUFFERS_SIZE] __attribute__((aligned(4096)));
static u32 evt_lpos;

/* DCFG as last written, so SET_ADDRESS never reads the register back. */
static u32 dcfg_cache;

/*
 * DALEPENA 的影子副本（2026-10-23 加）。
 *
 * `dwc3_set_address()` 的注释里记着一条真机结论：USB 相关路径上对
 * **DCFG 的读**会 stall 总线，所以那里改成了写缓存值。`DALEPENA` 一直还是
 * 读-改-写（`clrbits32`/`setbits32`），而它正好是复位路径上的第一个访问 ——
 * 于是同一种"读会 stall"在复位路径上又出现了一次。
 *
 * 这里把 DALEPENA 也改成影子写：只在使能/禁用端点时改内存里的副本，
 * 寄存器只写不读。core 初始化把寄存器清 0 时把影子一起清 0。
 */
static u32 dalepena_shadow;

/* Called repeatedly while dwc3_depcmd() spins. There is no activity light on
 * this payload -- the BootROM reporter does not return -- so this is only a
 * scheduling point. */
#ifdef ENUM
/*
 * 端点命令的存活探针（2026-10-23 加）。
 *
 * 命令正常 retire 时，这个钩子在一次 depcmd 里只会被调用几十次；只有命令
 * 迟迟不 retire，才能累到几十万次以上。所以在这里翻转灯，等于给"卡在端点
 * 命令上"单独配了一路信号，且不依赖主循环：
 *
 *   灯按主循环心跳的节奏慢慢闪  ⇒ 命令接口没事，卡点在别处
 *   灯明显更快地闪（每秒数次）  ⇒ 有端点命令反复跑满轮询窗口，命令不 retire
 *   灯彻底不动                  ⇒ 要么没有命令超时，要么 CPU 已不在软件路径上
 *
 * 2026-10-23 对齐内核后，轮询上限从 2000 降到 500，并且每次轮询之间补
 * udelay(1)（与功能构建同一窗口，约 500 µs）。掩码同步降到 2^8：一次
 * 超时的 depcmd 大约翻两次灯，正常 retire 的命令仍翻不动。
 */
void dwc3_wait_tick(void)
{
    static u32 spins;
    static u32 lit;

    udelay(1);

    if ((++spins & 0xFFu) != 0)
        return;

    if (lit)
        led_off();
    else
        led_on();
    lit = !lit;
}
#else
/* 内核的轮询窗口是 500 µs 量级，所以这里也要有间隔：纯背靠背的 MMIO 读
 * 只有几十微秒，会把本来能 retire 的命令判成超时。 */
void dwc3_wait_tick(void)
{
    udelay(1);
}
#endif

/* The A7C driver polls DCTL.CSFTRST for 500 ms before giving up (notes
 * section 9.6.3, step 6). */
#define SOFTRESET_TIMEOUT_TICKS  (TIMER0_HZ / 2u)
/*
 * 第三十二轮：超时判据原本只靠 `timer_ticks()`。如果 TIMER0 在复位窗口里
 * 停摆，`timer_ticks() - start` 恒为 0，超时永不成立；而 `udelay(10)` 里
 * 的 `UDELAY_STALL_LIMIT` 又会立刻 break，于是循环退化成"每圈读一次 DCTL"
 * 的永久自旋。
 *
 * 需要说明的是：这条路径只在 `dwc3_core_init()` 里走一次（上电启动期），
 * **解释不了第三十一轮"1-2-6 之后长亮"** —— 那时它早就返回了。这里补上
 * 上限只是把已知的最后一处"依赖定时器的无界循环"消掉，属于清理，不是修复
 * 本轮现象。上限取得足够大：定时器正常时 500 ms 内必然退出，够不到它。
 */
#define SOFTRESET_SPIN_LIMIT  200000u

static int dwc3_core_soft_reset(void)
{
    u32 start = timer_ticks();
    u32 spins = 0;

    setbits32(DWC3_DCTL, DWC3_DCTL_CSFTRST);
    for (;;) {
        if (!(read32(DWC3_DCTL) & DWC3_DCTL_CSFTRST)) {
            /* The reset drops the DCFG speed and address with it. */
            return DWC3_OK;
        }
        if (timer_ticks() - start > SOFTRESET_TIMEOUT_TICKS)
            return DWC3_ERR_SOFTRESET;
        if (++spins >= SOFTRESET_SPIN_LIMIT)
            return DWC3_ERR_SOFTRESET;
        udelay(10);
    }
}

/* Sony's own driver for this SoC only ever clears the two suspend bits in
 * GUSB2PHYCFG: the A7C probe does `bic #0x140` (SUSPHY bit 6 + ENBLSLPM
 * bit 8) and nothing else (notes 9.6.3, 0x602b4404). The ILCE-7M4 payload's
 * UTMI 8-bit override (PHYIF/USBTRDTIM) is not carried over: the ZV-E10
 * toolkit dropped it for exactly the same reason and enumerates on the same
 * core. */
static void dwc3_phy_setup(void)
{
    u32 reg;

    reg = read32(DWC3_GUSB2PHYCFG(0));

    /*
     * 2026-10-23 第十九轮：这里**不要**照抄 7M4 payload 的 UTMI 8-bit override。
     *
     * 三条证据指向同一结论：
     *   1. A7C 内核在 0x602b4404 只清 bit8（ENBLSLPM），不写 PHYIF/USBTRDTIM；
     *   2. 旧注释记着 ZV-E10 toolkit 在同一颗 core 上**去掉**了这段 override 仍能枚举；
     *   3. 对同一台机器来说，内核行为优先于别的机型的 payload。
     * 反过来，若这颗 PHY 实际是 16 位而写成 8 位，会直接打断数据通路。
     */
    reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;
    reg &= ~DWC3_GUSB2PHYCFG_ENBLSLPM;

    write32(DWC3_GUSB2PHYCFG(0), reg);
}

/*
 * The A7C probe does exactly one thing to GCTL: `gctl &= ~0x38` (notes
 * section 9.6.3 step 8 / section 9.6.5 step 3). That clears SCALEDOWN
 * (bit 4..5) and DISSCRAMBLE (bit 3). PWRDNSCALE is never written and the
 * ip_rev-gated U2RSTECN bit 16 never sets, so neither belongs here -- writing
 * the 7M4 value would overwrite whatever the boot chain left in place.
 */
static void dwc3_setup_global_control(void)
{
    clrbits32(DWC3_GCTL, DWC3_GCTL_SCALEDOWN_MASK | DWC3_GCTL_DISSCRAMBLE);
}

/*
 * 2026-10-23 第十九轮：上一轮加的 `dwc3_disable_clock_gating()` 已撤销。
 *
 * 参考实现（7M4 / ZV-E10）在这一步是 `reg &= ~DWC3_GCTL_DSBLCLKGTNG` ——
 * 即**保持**时钟门控使能，与我的置位相反；A7C 内核的 `gctl &= ~0x38` 也
 * 根本不碰 bit0。两边都没有"关掉门控"这个动作，上一轮的假设（门控导致命令
 * 不 retire）在真机上也被否掉了（置位后读数没有任何变化）。
 */
static void dwc3_set_mode_device(void)
{
    /* Not part of the A7C probe: the device-mode half of the driver sets
     * PRTCAP when it connects. It is still the value this payload needs, so
     * write it explicitly rather than inherit it from the boot chain. */
    clrsetbits32(DWC3_GCTL, DWC3_GCTL_PRTCAP_MASK,
                 DWC3_GCTL_PRTCAP(DWC3_GCTL_PRTCAP_DEVICE));
}

/*
 * 第六十轮：FIFO 分区写入整段删除。
 *
 * 第五十九轮把两次 write32 用 DWC3_FIFO_WRITE=0 关掉做了单变量 A/B，
 * 结果设备完整枚举成功（18d1:4ee0 / ILCE-7C / WinUSB，Status=OK），
 * 证明从第十六轮起被当作前提的"必须写 GTXFIFOSIZ/GRXFIFOSIZ"并不成立：
 * boot 链留下的分区足够 EP0 与 fastboot 使用。因此不再重写，
 * 相关的 boot 捕获、探针与尺寸计算也一并移除。
 */



static void dwc3_event_buffer_setup(void)
{
    u32 addr = (u32)(unsigned long)event_buffer;
    u32 i;

    for (i = 0; i < DWC3_EVENT_BUFFERS_SIZE; i += 4)
        write32((u32)(unsigned long)&event_buffer[i], 0);
    evt_lpos = 0;

    write32(DWC3_GEVNTADRLO(0), addr);
    write32(DWC3_GEVNTADRHI(0), 0);
    write32(DWC3_GEVNTSIZ(0),
            DWC3_GEVNTSIZ_SIZE(DWC3_EVENT_BUFFERS_SIZE) | DWC3_GEVNTSIZ_INTMASK);
    write32(DWC3_GEVNTCOUNT(0), 0);
    dma_wmb();
}

int dwc3_core_init(void)
{
    u32 npsid;
    int ret;

    /*
     * 第一步先确认这确实是 Synopsys 的设备核，再动任何寄存器。
     * GSNPSID 是只读常量（0x5533xxxx），也是"0xF020C100 这扇窗口真的
     * 开了"的唯一廉价判据：窗口没开时读回来是无意义值，后面的软复位
     * 会静默超时、写坏的却是一个不存在的核。-Probe3 已经用它判读过一次，
     * 这里把结论固化成构建期的第一道闸（主计划 §12.9 C4）。
     */
    npsid = read32(DWC3_GSNPSID);
    if ((npsid & 0xFFFF0000u) != 0x55330000u)
        return DWC3_ERR_NPSID;

    ret = dwc3_core_soft_reset();
    if (ret)
        return ret;

    /*
     * 第五十八轮：这里原来紧跟软复位再打一次 GCTL.CORESOFTRESET 脉冲
     * （A7C 内核 0x602b2208 的做法，见笔记 9.6.3）。本轮把它整段撤掉，是
     * 单变量的 A/B 实验：
     *   - 两份**能正常枚举**的实现（ILCE-7M4-RE、zv-e10-toolkit）的
     *     dwc3.c 里 `CORESOFTRESET` 均零命中，它们只做 DCTL.CSFTRST；
     *   - 全局核复位会连同 ROM 留下的 FIFO 分区、GCTL/PHY 状态一起清掉。
     * 第五十九轮的 A/B 已经证明不写 FIFO 分区也能完整枚举，因此第六十轮
     * 把原来的 dwc3_fifo_setup() 整段删除，这里只剩 PHY / GCTL / 模式三步。
     */
    dwc3_phy_setup();
    dwc3_setup_global_control();
    dwc3_set_mode_device();

    clrsetbits32(DWC3_DCFG, DWC3_DCFG_SPEED_MASK | DWC3_DCFG_DEVADDR_MASK,
                 DWC3_DCFG_HIGHSPEED | DWC3_DCFG_DEVADDR(0));

    /* Snapshot once, while register reads are known to work, so
     * dwc3_set_address() never has to read DCFG back. */
    dcfg_cache = read32(DWC3_DCFG);

    dwc3_event_buffer_setup();

    /* 只使能本载荷实际处理的三个事件：DISCONN / USBRST / CONNECTDONE。
     * 内核实测写 0x1E1F，但 A7C 的 DEVT_TYPE 取位是 ((e)>>8)&0xF，若 CMDCMPLT
     * 的类型码恰为 2，会在控制传输中途被误判成 CONNECT_DONE 并重跑 ep0_init()，
     * 主机侧表现正是「设备描述符请求失败」。见主计划 §12.4。 */
    write32(DWC3_DEVTEN, 0x07u);
    write32(DWC3_DALEPENA, 0);
    dalepena_shadow = 0;
    return DWC3_OK;
}

int dwc3_connect(void)
{
    u32 timeout = 500;
    u32 reg;

    reg = read32(DWC3_DCTL);
    reg &= ~DWC3_DCTL_KEEP_CONNECT;
    reg |= DWC3_DCTL_RUN_STOP;
    write32(DWC3_DCTL, reg);

    while (timeout--) {
        if (!(read32(DWC3_DSTS) & DWC3_DSTS_DEVCTRLHLT))
            return DWC3_OK;
        udelay(100);
    }
    return DWC3_ERR_NOT_STARTED;
}

void dwc3_disconnect(void)
{
    clrbits32(DWC3_DCTL, DWC3_DCTL_RUN_STOP);
}

void dwc3_set_address(u32 addr)
{
    /*
     * Written from a cached copy, not read-modify-write: the only field that
     * changes here is DEVADDR, and a read of DCFG is the one access in the
     * SET_ADDRESS path that could stall the bus.
     */
    dcfg_cache = (dcfg_cache & ~DWC3_DCFG_DEVADDR_MASK)
               | DWC3_DCFG_DEVADDR(addr & 0x7Fu);
    write32(DWC3_DCFG, dcfg_cache);
}

/* ---- endpoint command layer --------------------------------------------- */

/*
 * 2026-10-23 第十九轮（对齐 A7C 内核 `__dwc3_gadget_ep_cmd`，VA 0x1ac3d0）：
 * 三处寄存器级偏差已按内核口径改掉。
 *
 *   1. 轮询上限 2000 → **500**，与内核一致。
 *   2. 超时**不再假装成功**：内核返回 `-ETIMEDOUT`，这里返回
 *      `DWC3_ERR_TIMEOUT`。旧写法"超时就返回 0"会让调用方以为端点已配好，
 *      失败被推到几百毫秒后的"数据装上了但永不完成"，反而更难查。
 *   3. 命令前后清/恢复 `GUSB2PHYCFG` 的 `SUSPHY|ENBLSLPM`（内核用
 *      `tst r,#0x140` 判这两位）。PHY 处于 suspend 时端点命令不 retire，
 *      CMDACT 会一直挂着 —— 这正是"轮询超时"的已知成因之一。
 *
 * 保留的历史结论：§9.23 记下的唯一一次完整枚举成功用的是"写命令后不再等"。
 * 但那次之后修掉了事件环对齐与 DGCMD 两个真 bug，当时的归因不再成立；
 * 丢掉每条命令的 status 代价太大，故回到内核语义。
 *
 * 第四十六轮：撤掉上一条里的第 3 项（清/恢复 `GUSB2PHYCFG` 的
 * `SUSPHY|ENBLSLPM`）。这是本载荷自创的动作，仓库内已验证能枚举的
 * ZV-E10 实现完全不碰这个寄存器（`zv-e10-toolkit/fastboot/src/dwc3.c`
 * 的 `dwc3_depcmd()`）；本轮按方案 A 一次对齐已验证实现。
 * 轮询口径（500 次 + 超时如实返回）保持不变。
 */
#define DEPCMD_POLL_LIMIT  500u

int dwc3_depcmd(u32 phys_ep, u32 cmd, u32 p0, u32 p1, u32 p2)
{
    u32 iters, reg;
    int ret;

#ifdef ENUM
    /* 站点 4：入口第一次 MMIO 读已经回来了。看不到它而前面的站点亮过 ⇒
     * 卡在读寄存器（或它之前），不是 PAR 写。 */
    enum_site_once(4);
#endif

    write32(DWC3_DEPCMDPAR0(phys_ep), p0);
    write32(DWC3_DEPCMDPAR1(phys_ep), p1);
    write32(DWC3_DEPCMDPAR2(phys_ep), p2);
    dma_wmb();
#ifdef ENUM
    /* 站点 5：三个参数寄存器写完。若卡在这里，说明 PAR 的写会 stall。 */
    enum_site_once(5);
#endif

    write32(DWC3_DEPCMD(phys_ep), cmd | DWC3_DEPCMD_CMDACT);
#ifdef ENUM
    /* 站点 6：命令寄存器写完、准备轮询。看到 5 没有 6，说明卡在写 DEPCMD
     * 上；看到 6 而探针不翻，说明卡在轮询里。 */
    enum_site_once(6);
#endif

    ret = DWC3_ERR_TIMEOUT;
    for (iters = 0; iters < DEPCMD_POLL_LIMIT; iters++) {
        reg = read32(DWC3_DEPCMD(phys_ep));
        if (!(reg & DWC3_DEPCMD_CMDACT)) {
            ret = (int)DWC3_DEPCMD_STATUS(reg);
            break;
        }
        dwc3_wait_tick();
    }

    return ret;
}

/*
 * DEPSTARTCFG goes through ep0's *endpoint* command interface, not DGCMD.
 *
 * An earlier version of this payload sent it to DGCMD (0xC714) on the theory
 * that re-partitioning the FIFO RAM must be a device-generic command.  That
 * was never proven: the only evidence was that the FIFO depth registers read
 * back with the new value, which says nothing about whether the core applied
 * them.  Both implementations that do work on this silicon send it the
 * endpoint way:
 *
 *   - the stock A7C kernel: __dwc3_gadget_ep_enable() @0x602b485c builds
 *     cmd = 9 with param 0 and hands it to the endpoint command sender
 *     @0x602b43cc, which writes PAR2/PAR1/PAR0 at 0xC800/04/08 + 0x10*ep and
 *     DEPCMD at 0xC80c + 0x10*ep;
 *   - the ILCE-7M4 payload this one is ported from: dwc3_depcmd(0, ...).
 *
 * Everything downstream (DEPCFG, SETTRANSFRESOURCE, STARTTRANSFER) is accepted
 * even when this command never ran, so the failure shows up only as a transfer
 * that is armed and never completes -- exactly what the bench sees.
 */
int dwc3_ep_start_config(void)
{
    return dwc3_depcmd(0, DWC3_DEPCMD_DEPSTARTCFG, 0, 0, 0);
}

/*
 * 第四十六轮：撤掉第四十二轮引入的「资源索引随 STARTTRANSFER 下发」。
 *
 * 第四十二轮曾把 SETTRANSFRESOURCE 回读到的索引存进 `dep_rsc[]`，并以
 * `DWC3_DEPCMD_PARAM(rsc)` 随命令下发；第四十四/四十五轮读数证明这既不是
 * 病因（索引确实为 2 且已下发，仍无任何完成事件）。仓库内已验证能枚举的
 * ZV-E10 实现同样只回读、不下发（`zv-e10-toolkit/fastboot/src/dwc3.c` 的
 * `dwc3_ep_start_xfer()`），本轮按方案 A 对齐。
 */
int dwc3_ep_config(u32 phys_ep, u32 type, u32 maxpacket)
{
    u32 p0 = DWC3_DEPCFG_EP_TYPE(type)
           | DWC3_DEPCFG_MAX_PACKET_SIZE(maxpacket)
           | DWC3_DEPCFG_ACTION_INIT;
    u32 p1 = DWC3_DEPCFG_EP_NUMBER(phys_ep) | DWC3_DEPCFG_INT_NUM(0);
    int ret;

    /* Every IN endpoint owns a TxFIFO, ep0 included; the ZV-E10 toolkit sets
     * the field unconditionally on the same core. OUT endpoints share the one
     * RxFIFO and the field is ignored there. */
    if (phys_ep & 1u)
        p0 |= DWC3_DEPCFG_FIFO_NUMBER(phys_ep >> 1);

    if (type == DWC3_DEPCMD_TYPE_CONTROL)
        p1 |= DWC3_DEPCFG_XFER_COMPLETE_EN | DWC3_DEPCFG_XFER_NOT_READY_EN;
    else
        p1 |= DWC3_DEPCFG_XFER_COMPLETE_EN | DWC3_DEPCFG_XFER_IN_PROGRESS_EN;

    ret = dwc3_depcmd(phys_ep, DWC3_DEPCMD_SETEPCONFIG, p0, p1, 0);
    if (ret)
        return ret;

    return dwc3_depcmd(phys_ep, DWC3_DEPCMD_SETTRANSFRESOURCE, 1, 0, 0);
}

void dwc3_ep_enable(u32 phys_ep)
{
    dalepena_shadow |= BIT(phys_ep);
    write32(DWC3_DALEPENA, dalepena_shadow);
}

void dwc3_ep_disable(u32 phys_ep)
{
    dalepena_shadow &= ~BIT(phys_ep);
    write32(DWC3_DALEPENA, dalepena_shadow);
}

/*
 * 第四十六轮：删除第四十五轮的 `dwc3_ep_rearm()`。
 *
 * 第四十五轮在每次武装数据/状态阶段前重跑 DEPCFG + SETTRANSFRESOURCE +
 * DALEPENA，读数与第四十四轮逐字段相同（`1长-1长-1`），方案证伪。
 * 仓库内已验证能枚举的 ZV-E10 实现没有这一步：ep0 只在 `RESET` 与
 * `CONNECT_DONE` 两处重跑 `ep0_init()`，本载荷第 14 轮起已与之一致。
 */
int dwc3_ep_start_xfer(u32 phys_ep, u64 trb_addr, u32 *rsc_idx)
{
    int ret;

    dma_wmb();
    ret = dwc3_depcmd(phys_ep, DWC3_DEPCMD_STARTTRANSFER,
                      (u32)(trb_addr >> 32), (u32)trb_addr, 0);
    if (ret)
        return ret;

    if (rsc_idx)
        *rsc_idx = DWC3_DEPCMD_GET_RSC_IDX(read32(DWC3_DEPCMD(phys_ep)));
    return 0;
}

int dwc3_ep_end_xfer(u32 phys_ep, u32 rsc_idx)
{
    return dwc3_depcmd(phys_ep,
                       DWC3_DEPCMD_ENDTRANSFER | DWC3_DEPCMD_HIPRI_FORCERM |
                       DWC3_DEPCMD_CMDIOC | DWC3_DEPCMD_PARAM(rsc_idx),
                       0, 0, 0);
}

int dwc3_ep_set_stall(u32 phys_ep)
{
    return dwc3_depcmd(phys_ep, DWC3_DEPCMD_SETSTALL, 0, 0, 0);
}

int dwc3_ep_clear_stall(u32 phys_ep)
{
    return dwc3_depcmd(phys_ep, DWC3_DEPCMD_CLEARSTALL, 0, 0, 0);
}

u32 dwc3_event_poll(void)
{
    u32 count = read32(DWC3_GEVNTCOUNT(0)) & DWC3_GEVNTCOUNT_MASK;
    u32 event;

    if (count < 4)
        return 0;

    dsb();
    event = *(volatile u32 *)&event_buffer[evt_lpos];

    /*
     * 第三十九轮：**撤销第三十八轮的"按事件长度消费"**。
     *
     * 上一轮假设端点事件在缓冲里占 2 个字，于是按类型写 4 / 8 字节。
     * 两轮读数推翻了它：
     *   第三十七轮（4 字节）：SETUP ≥9、武装 ≥9、完成 0
     *   第三十八轮（8 字节）：SETUP 4、武装 0、完成 0
     * 8 字节那一版把 SETUP 砍掉一半、把 NRDY 整条跳掉，是回归。
     *
     * GEVNTCOUNT 是字节计数，但每个事件都是一个 4 字节字（参数位编码在
     * 同一字里），ZV-E10 参考实现与 Linux dwc3 都是在处理完一个事件后
     * 无条件写 4。所以这里恢复固定 4 字节。
     */
    evt_lpos = (evt_lpos + 4) % DWC3_EVENT_BUFFERS_SIZE;

    /* Tell the controller those bytes are consumed. */
    write32(DWC3_GEVNTCOUNT(0), 4);
    return event;
}
