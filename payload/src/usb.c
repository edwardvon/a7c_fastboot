#include "usb.h"
#include "dwc3.h"
#include "dwc3_regs.h"
#include "fastboot.h"
#include "io.h"
#include "timer.h"
#if defined(ENUM) || defined(HOLD) || defined(TRACE)
#include "led.h"
#endif

/*
 * Minimal USB 2.0 device: one configuration, one vendor interface, two bulk
 * endpoints. Ported from the ILCE-7M4 payload; the only structural change is
 * that the DMA objects live in .bss instead of a linker-placed .usbdma
 * section, because no address above the 584 verified bytes of ESRAM has been
 * confirmed reachable from the DWC3 AXI master on the A7C.
 *
 * ep0 is driven by XferNotReady rather than by assuming the phase order: the
 * controller says which phase it wants and which physical endpoint to queue
 * it on.
 */

#define EP0_OUT     DWC3_PHYS_EP(0, 0)   /* 0 */
#define EP0_IN      DWC3_PHYS_EP(0, 1)   /* 1 */
#define EP_OUT      DWC3_PHYS_EP(1, 0)   /* 2 */
#define EP_IN       DWC3_PHYS_EP(1, 1)   /* 3 */

/* ---- DMA-visible objects ------------------------------------------------ */

/*
 * 第五十六轮：三个控制阶段重新各占一格，但这次**按消费顺序紧邻排列**。
 *
 * 第四十六轮曾把三张 TRB 合成一张（对齐 ZV-E10 参考实现），读数与第 44/45
 * 轮逐字段相同；第五十五轮又证明「装 TRB 的时机」也不是根因。剩下的最强
 * 机制假设是：控制器消费掉 SETUP 那条 TRB 之后，内部的取 TRB 指针已经
 * 推进到**下一格**（+16 B）。共用一张时那一格是空白的（HWO=0），控制器
 * 看了一眼就停下，于是命令被接受、DATA TRB 却永远没人取。
 *
 * 判决实验：让 SETUP/DATA/STATUS 三格在内存里按这个顺序连续摆放（数组元素
 * 天然连续，每格 16 B），使 DATA 恰好落在 SETUP 的下一格。若这个假设成立，
 * 段 ③ 应变成 2（被取走）且段 ② 非 0。
 *
 * 更正一处旧注释（原文写成「第 46 轮当时也是三张独立 TRB」，与笔记 §5.20
 * 矛盾）：第 46 轮的事实恰恰相反 —— 那一轮把 SETUP / DATA / STATUS 三张 TRB
 * **合并成一张** `ep0_trb`，三阶段复用同一物理地址（第 46~54 轮一直如此）。
 * 所以本轮才是第一次「三格按消费顺序分开摆放」，是全新实验，不是复测。
 */
static volatile struct dwc3_trb ep0_trb_ring[3] __attribute__((aligned(64)));

#define ep0_trb_setup   ep0_trb_ring[0]
#define ep0_trb_data    ep0_trb_ring[1]
#define ep0_trb_status  ep0_trb_ring[2]
static volatile struct dwc3_trb bulk_trb[2] __attribute__((aligned(64)));

/* Round 73: EP_IN event accounting and the deadline snapshot. The counters are
 * cheap (one increment per dep event) and are what makes the snapshot
 * interpretable: "no completion" and "no event at all" look identical from the
 * wait loop alone. */
static u32 bulk_in_evt_seq;
static u32 bulk_in_last_type;
static struct usb_bulk_diag bulk_diag;
static u8 setup_buf[64]                   __attribute__((aligned(64)));
static u8 ep0_buf[512]                    __attribute__((aligned(64)));

/* ---- descriptors -------------------------------------------------------- */

static const u8 device_desc[18] = {
    18, USB_DT_DEVICE,
    0x00, 0x02,             /* bcdUSB 2.00 */
    0xff, 0xff, 0xff,       /* vendor specific class/subclass/protocol */
    EP0_MAXPACKET,
    0xd1, 0x18,             /* idVendor  0x18d1 (Google) */
    0xe0, 0x4e,             /* idProduct 0x4ee0 (fastboot) */
    0x00, 0x01,             /* bcdDevice 1.00 */
    1, 2, 3,                /* iManufacturer, iProduct, iSerial */
    1,                      /* bNumConfigurations */
};

/* Fastboot's interface signature: class 0xff, subclass 0x42, protocol 0x03. */
static const u8 config_desc[32] = {
    /* configuration */
    9, USB_DT_CONFIG,
    32, 0,                  /* wTotalLength */
    1,                      /* bNumInterfaces */
    1,                      /* bConfigurationValue */
    0,                      /* iConfiguration */
    0x80,                   /* bus powered */
    0xfa,                   /* 500 mA */

    /* interface */
    9, USB_DT_INTERFACE,
    0,                      /* bInterfaceNumber */
    0,                      /* bAlternateSetting */
    2,                      /* bNumEndpoints */
    0xff, 0x42, 0x03,       /* fastboot */
    0,                      /* iInterface */

    /* bulk OUT */
    7, USB_DT_ENDPOINT,
    EP_BULK_OUT_ADDR,
    USB_ENDPOINT_XFER_BULK,
    (BULK_MAXPACKET & 0xff), (BULK_MAXPACKET >> 8),
    0,

    /* bulk IN */
    7, USB_DT_ENDPOINT,
    EP_BULK_IN_ADDR,
    USB_ENDPOINT_XFER_BULK,
    (BULK_MAXPACKET & 0xff), (BULK_MAXPACKET >> 8),
    0,
};

/* Required when bcdUSB >= 0x0200. */
static const u8 qualifier_desc[10] = {
    10, USB_DT_DEVICE_QUALIFIER,
    0x00, 0x02,
    0xff, 0xff, 0xff,
    EP0_MAXPACKET,
    1,                      /* bNumConfigurations */
    0,                      /* reserved */
};

static const u8 str_langid[4] = { 4, USB_DT_STRING, 0x09, 0x04 };

/* "Sony" */
static const u8 str_vendor[10] = {
    10, USB_DT_STRING,
    0x53, 0x00, 0x6f, 0x00, 0x6e, 0x00, 0x79, 0x00,
};

/* "ILCE-7C" */
static const u8 str_product[16] = {
    16, USB_DT_STRING,
    0x49, 0x00, 0x4c, 0x00, 0x43, 0x00, 0x45, 0x00,
    0x2d, 0x00, 0x37, 0x00, 0x43, 0x00,
};

/* "ILCE7C-fastboot" */
static const u8 str_serial[32] = {
    32, USB_DT_STRING,
    0x49, 0x00, 0x4c, 0x00, 0x43, 0x00, 0x45, 0x00,
    0x37, 0x00, 0x43, 0x00, 0x2d, 0x00, 0x66, 0x00,
    0x61, 0x00, 0x73, 0x00, 0x74, 0x00, 0x62, 0x00,
    0x6f, 0x00, 0x6f, 0x00, 0x74, 0x00,
};

/* ---- ep0 state ---------------------------------------------------------- */

/*
 * Which stage of a control transfer is armed. 第四十六轮起三个控制阶段共用
 * 同一张 TRB，这个序号只用于状态机与播报，不再索引 TRB。
 */
enum ep0_phase {
    EP0_PHASE_SETUP,        /* setup TRB armed, waiting for the packet */
    EP0_PHASE_DATA,         /* data TRB armed, transfer in flight        */
    EP0_PHASE_STATUS,       /* status TRB armed, transfer in flight      */
};

static enum ep0_phase ep0_phase;

/*
 * 第四十一轮：状态/数据阶段装哪个物理端点，**只由 NRDY 事件说**。
 *
 * 第四十轮的读数（武装失败 0 / 完成 0 / NRDY 落在物理端点 1）推翻了
 * "控制端点在本核上只有一个双向物理端点 0"的假设：控制器明确点名端点 1，
 * 而我们把 TRB 全装到端点 0，于是命令被接受、TRB 永远没人取。
 *
 * 所以这里不再预先算端点：`handle_depevt()` 里的 `ep` 就是
 * `DEPEVT_EP(event)`，直接拿它去武装。这条取位在本核上已被读数证实
 * （§9.23.2 的内核反汇编 `ubfx r3,r2,#1,#5`）。
 */
static struct usb_ctrlrequest ctrl;

static volatile int bulk_out_busy, bulk_in_busy;
static volatile u32 bulk_out_residue, bulk_in_residue;

/*
 * Incremented on every bus reset, which releases the transfer resources of
 * every endpoint and so kills any queued bulk transfer. The host resets on
 * every fastboot command, so that is the normal case; the bulk helpers watch
 * this across their wait loop and re-queue when it moves.
 */
static volatile u32 bulk_epoch;

/* Set by SET_CONFIGURATION, acted on by usb_bulk_enable_pending(). */
static volatile int bulk_enable_pending;

static const u8 *ep0_data;      /* what to send in the data stage */
static u32 ep0_data_len;
static volatile int configured;

/*
 * 第三十四轮：SETUP 缓冲逐字节、volatile 取数。
 *
 * 第三十三轮读数 `1-2-6-1-长亮` 把卡点钉在台阶 1 → 台阶 2 之间，那段窗口里
 * 只有 dmb 与这 5 次 ctrl 解析拷贝，其中两条是编译器合并出来的**非对齐字
 * 访问**：`ldr [r5,#66]`（读 setup_buf+1 起 4 字节）与 `str [r5,#298]`
 * （写 ctrl+2 起 4 字节，即 0x12a）。ctrl 是 packed 结构，字段偏移 1 和 2
 * 让字访问天然落成非对齐 —— 在这颗核上，非对齐字访问碰到 Device 内存是
 * UNPREDICTABLE，正是「没有异常灯语、灯却定在亮」的候选解释。
 *
 * 逐字节 volatile 读 + 逐字段 volatile 写，让每次访问都落在自然边界上。
 */
static u8 setup_byte(u32 i)
{
    return *(volatile const u8 *)&setup_buf[i];
}

/*
 * -Trace build: open the pump up and label every step.  No timer anywhere.
 *
 *   2 快闪   usb_gadget_run() entered -- the pump is starting
 *   1 快闪   an event word was read; dispatching it now
 *   3 快闪   the dispatch returned
 *   4 快闪   every 10,000 empty polls: polling works, nothing to dispatch
 *
 * One symbol per event, so the reading is a sequence, not a count:
 *
 *   2 → 1 → 3 → 1 → 3 …    dispatch always returns; the hang is elsewhere
 *   2 → 1 → (dark)         hangs *inside* handle_devt()/handle_depevt()
 *   2 → 4 → 4 → …          working, but no event ever arrives
 *   2 → (dark)             hangs inside the poll itself (register or ring read)
 *   (no 2 at all)          hangs before the pump
 *
 * The markers are slow (NOP-timed), which will break the host's control
 * transfer on purpose: this run is about where the CPU stops, not about
 * enumerating.
 */
#ifdef TRACE
static u32 st_reset, st_setup, st_data, st_done, st_stall, st_nrdy;

/* Roughly 0.1 s at 300 MHz, 0.3 s at 100 MHz -- the CPU clock is unknown here
 * and deliberately not measured, since the timer is what is in question. */
static void trace_nop_delay(void)
{
    u32 i;

    for (i = 0; i < 8000000u; i++)
        __asm volatile ("nop");
}

#define TRACE_FLASH(n)  ((void)0)
#define TRACE_RESET()   (st_reset++)
#define TRACE_SETUP()   (st_setup++)
#define TRACE_BLIP()    (st_data++)
#define TRACE_DONE()    (st_done++)
#define TRACE_STALL()   (st_stall++)
#define TRACE_NRDY()    (st_nrdy++)
#else
#define TRACE_FLASH(n)  ((void)0)
#define TRACE_RESET()   ((void)0)
#define TRACE_SETUP()   ((void)0)
#define TRACE_BLIP()    ((void)0)
#define TRACE_DONE()    ((void)0)
#define TRACE_STALL()   ((void)0)
#define TRACE_NRDY()    ((void)0)
#endif

#if defined(HOLD) || defined(ENUM)
/*
 * -Hold / -Enum 共用的码表（2026-10-23 第二十四轮：改记「阶段归属」）。
 *
 * 第二十三轮是"一次性站点 + 现场数灯"，真机读数 1-2-6-1-2-3-4-5-6-7-7-10：
 *
 *   a. 站点 7 出现了两次。站点走 enum_site_once() 的位掩码去重，同一编号
 *      在正常内存下不可能播两次，而现场两次复测读数一致。
 *   b. 站点 9（完成@DATA）缺失，站点 10（完成@STATUS）在场。
 *
 * 两个异常指向同一个结构性嫌疑：三个控制阶段**共用同一个 ep0_trb**。如果
 * 状态阶段的 NRDY 早于数据阶段完成，ep0_queue_status() 会把仍在 HWO 的
 * TRB 连同 ep0_phase 一起覆盖成 STATUS —— 之后到达的数据完成事件自然被
 * 算成 STATUS 阶段，即 9 缺 10 在。
 *
 * 所以这一轮不再记"播报点在哪"，而是记**谁在什么时候被装进 TRB**：
 * 装 DATA（5）之后、装 STATUS（6）之前，中间有没有出现"完成@DATA"（7）。
 * 一次读数即可判定覆盖是否发生，不依赖现场数灯。
 *
 *   1  总线复位
 *   2  CONNECT_DONE
 *   3  SETUP 完成（收到并解析完，进 handle_setup）
 *   4  装 SETUP TRB
 *   5  装 DATA TRB
 *   6  装 STATUS TRB
 *   7  完成：ep0_phase == DATA
 *   8  完成：ep0_phase == STATUS
 *   9  完成：ep0_phase 是别的值（既非 SETUP 也非 DATA/STATUS）
 *  10  NRDY(DATA)：没有数据可给 → stall
 *  11  NRDY(STATUS)：守卫跳过（第三十一轮起守卫已删除，本码不再出现）
 *  12  装 TRB 失败（STARTTRANSFER 超时或非零状态）
 *  13  装下一阶段 TRB 之前采样：上一阶段那条 TRB 的 HWO 仍为 1
 *  14  同样位置：上一阶段那条 TRB 已释放（控制器已经把它交回来了）
 *
 *      第二十五轮把 13/14 的采样点扩到三处，靠**位置**分辨，不新增码：
 *        `14-5`        → 装 DATA TRB 之前，SETUP 槽是否已释放
 *        `7-14-6`      → 装 STATUS TRB 之前，DATA 槽是否已释放（旧判据）
 *        `13-4-1`      → 总线复位前，DATA 槽是否还被控制器持有
 *      第二十九轮：三个控制阶段各占一个 TRB 槽，13 不再意味着"覆盖了未完成
 *      的传输"（已无覆盖），而是"控制器还没把上一阶段的 TRB 交回来"。
 *      第四十六轮回到单张 TRB，13 又同时意味着"上一阶段的 TRB 还没交回"。
 *      第五十六轮再次分成三格（按 SETUP→DATA→STATUS 顺序紧邻），语义同
 *      第二十九轮：13 表示"上一阶段那条 TRB 控制器还没交回来"。
 *  15  handle_setup 拒绝了请求 → stall
 */
#define ENUM_LOG_RESET       1u
#define ENUM_LOG_CONNECT     2u
#define ENUM_LOG_SETUP       3u
#define ENUM_LOG_ARM_SETUP   4u
#define ENUM_LOG_ARM_DATA    5u
#define ENUM_LOG_ARM_STATUS  6u
#define ENUM_LOG_DONE_DATA   7u
#define ENUM_LOG_DONE_STATUS 8u
#define ENUM_LOG_DONE_OTHER  9u
#define ENUM_LOG_NO_DATA    10u
#define ENUM_LOG_NRDY_SKIP  11u
#define ENUM_LOG_ARM_FAIL   12u
#define ENUM_LOG_OVERWRITE  13u
#define ENUM_LOG_RELEASED   14u
#define ENUM_LOG_REJECTED   15u
#endif

#if !defined(HOLD) && !defined(ENUM)
/* 非诊断构建不带站点灯标；on_reset() 里的 enum_site() 调用需要一个空实现。 */
static void enum_site(u32 n)
{
    (void)n;
}
#endif

#ifdef HOLD
/*
 * -Hold build: the same reports as -Enum, but inverted against a lamp that
 * stays ON for the payload's whole life.
 *
 * The point is not the reports, it is the baseline.  With the lamp held on,
 * "the lamp stopped being steady" is by itself proof that something other than
 * this payload is driving it -- the BootROM reporter or a ROM exception
 * handler.  On the bench the lamp showed a plain "2 short" loop with no long
 * flash after it, which no path in this file can produce: every report here is
 * followed by a long lead-in and a status group.
 *
 *   1 dark  data stage armed (STARTTRANSFER accepted)
 *   2 dark  data stage refused: STARTTRANSFER was rejected
 *   3 dark  data stage completed (the host took the descriptor)
 *   4 dark  after arming, the TRB's HWO is still set
 *   6 dark  after arming, HWO is already clear
 *   5 dark  setup not understood / nothing to send / status stage failed
 */
#define ENUM_MARK_ARMED      1u
#define ENUM_MARK_REJECTED   2u
#define ENUM_MARK_DONE       3u
#define ENUM_MARK_HWO_SET    4u
#define ENUM_MARK_TROUBLE    5u
#define ENUM_MARK_HWO_CLEAR  6u

static void enum_blink(u32 count)
{
    static int budget = 200;

    if (budget-- <= 0)
        return;

    while (count--) {
        led_off();
        mdelay(120);
        led_on();
        mdelay(120);
    }
    mdelay(2000);
}

/* -Hold 版不记账，直接播报（保持旧语义）。 */
static u8 enum_resets;   /* 与 -Enum 同名的计数，仅为共用 on_reset() 的代码 */
static u8 enum_log_n;    /* -Hold 永不增长：没有日志，静默播报分支不会触发 */
#define ENUM_QUIET_LIMIT 1u
#define ENUM_BEAT_EVERY  400000u  /* -Hold：灯本来就常亮，心跳翻转不适用但需能编译 */

static void enum_rec(u8 code)
{
    enum_blink(code);
}

static void enum_dump(void)
{
}

/* -Hold 构建不打站点：它的灯语义是"常亮 + 暗缺口"，接站点会打乱判读。 */
static void enum_site(u32 n)
{
    (void)n;
}
#else /* ---- -Enum ------------------------------------------------------- */
#ifdef ENUM
/*
 * -Enum 诊断构建专用：**先记账，后播报**（2026-10-23 第三版）。
 *
 * 上一版是"发生即播报"，而每个标记要阻塞约 0.1~0.5 s（NOP 计时，主频未知）。
 * 主机的控制传输超时是 100 ms 量级，于是诊断构建自己就把枚举打断 ——
 * 真机读数 7→9→8→1→7 正好是这个形状：8 和 1 的播报把主机的耐心耗光了。
 * 结论：**在枚举过程中播报，等于测什么坏什么**。
 *
 * 现在改成日志：事件发生时只往一个 24 字节数组里存一个码（纳秒级），
 * 只在"已经确定这次尝试失败"的时刻才整段播出来：
 *   - 第二次总线复位（主机放弃了第一次尝试）
 *   - 出现 8 / 9（请求被拒 / 复位后装不上 setup TRB，必然聋）
 * 播报仍然阻塞，但那时枚举已经结束了，阻塞无害。
 *
 * 码表见上面「-Hold / -Enum 共用的码表」。
 *
 * 第三十七轮：**播报改用 mdelay**，NOP 只留在空转泵里。
 *
 * 早先全用 NOP 的理由是"插线瞬间定时器计数率会变 2~3 倍"。计数率确实会变，
 * 但判读只看**比例**：短闪亮 120 ms / 灭 180 ms / 段间隔 1500 ms 之间的倍数
 * 关系不随计数率改变，绝对时长被压 2~3 倍也照样分得开。之前"间隔不明显"
 * 的根因正是 NOP 计时宽度随主频漂移，且段间隔只有闪间隙的 2 倍。
 */
/*
 * 第二十四轮：重新打开自动播报。第二十三轮的站点是"现场数灯"，读数里出现
 * 了去重机制不可能产生的重复组；这一轮改成"先记账、静默后整段播出"，
 * 读数不再依赖人眼在现场分辨顺序。
 *
 * 阈值的单位是"空泵次数"：每轮还有 ENUM_DARK_NOP(200000) 个 NOP，约 3~8 ms，
 * 3000 次 ≈ 10~25 s —— 远超主机任何一次重试间隔，台架上也等得起。
 *
 * 第四十轮：整段等待里我们自己只占 10~25 s，其余两分多钟全在 PC 侧
 * （每次 GET_DESCRIPTOR 等满 5 s 超时 + 端口复位，约 9 次）。所以改成
 * **提早就播、只播一次**：
 *   武装数 ≥ 2 且静默 ≥ 400 次泵（≈1.5~3 s）就先报一份；
 *   兜底仍保留 1200 次（≈4~10 s），防止一次都没武装成功时永远不播。
 * 代价是数字可能比最终值小，但三个字段的判据方向不变。
 */
#define ENUM_DUMP_ON     1
#define ENUM_QUIET_EARLY 400u
#define ENUM_QUIET_LIMIT 1200u
/*
 * 第三十七轮：三字段短灯语的节拍常量。
 *
 * 前几轮播的是"码序列"，一组 2~6 s、整段 8 组，再加上和长闪前缀等长的
 * 结束符，人眼根本记不住。这一版把信息压到三个计数，每段最多 9 下，
 * 段间隔拉到闪间隙的 8 倍，整段约 7 s，播完就停。
 *
 *   第 1 段 = 本次上电解析到的 SETUP 数      （码 3）
 *   第 2 段 = ep0 完成事件数（数据 + 状态）  （码 7 / 8）
 *   第 3 段 = 数据阶段武装成功数             （码 5）
 *
 * 数字 0 用"亮 600 ms 的一下长闪"表示：不会和"没闪/漏看"混淆，
 * 也和结束符（1500 ms 常亮）差 2.5 倍。
 */
#define ENUM_BEAT_ON_MS   120u    /* 短闪亮着的时长 */
#define ENUM_BEAT_OFF_MS  180u    /* 短闪之间的黑 */
#define ENUM_ZERO_ON_MS   600u    /* 数字 0：一下长亮 */
#define ENUM_FIELD_GAP_MS 1000u   /* 段间全黑（第六十轮 1500→1000，仍是短闪间隙的 5 倍） */
#define ENUM_END_ON_MS    1500u   /* 结束符常亮时长 */
#define ENUM_COUNT_MAX    9u      /* 单个字段的封顶，避免长串拖长整段 */
/*
 * 第二十五轮：心跳改成"只在安静下来之后才打"。
 *
 * 上一版的真机反馈是"没看到心跳短闪"：心跳每 2000 次泵一次，而静默判据是
 * 3000 次泵 —— 整段静默里平均只有一次，一次约 40~110 ms，夹在 10~25 s 全黑
 * 里几乎必漏。现在改成每 500 次泵一个机会（≈2~4 s 一次），并且**只在已经
 * 安静 200 次泵之后**才真的闪：枚举进行中一下都不闪。
 *
 * 这条前置条件是必要的，不是顺手：心跳本身要阻塞几十毫秒，而主机的控制
 * 传输超时只有 100 ms 量级 —— 在传输进行中打心跳，就等于自己把传输掐断。
 * 前几轮"测什么坏什么"的老毛病就是这个。
 */
#define ENUM_BEAT_EVERY  500u      /* 每这么多次泵给一次心跳机会 */
#define ENUM_BEAT_AFTER  200u      /* 连续这么多次泵没有新记录才允许闪 */
#define ENUM_BLIP_NOP    1500000u  /* 心跳那一下亮着的时长（单下，不连闪） */
#define ENUM_DARK_NOP    200000u   /* 每轮循环的暗期：保证"正常"看起来是闪不是常亮 */

static void enum_nop(u32 n)
{
    while (n--)
        __asm volatile ("nop");
}

#ifdef ENUM_DUMP_ON
/*
 * 第三十七轮：播报原语。
 *
 * 全部用 mdelay，不再用 NOP 计数（见上面 392 行附近的说明）。
 * 亮/灭/段间隔/结束符四者的时长比是 120 : 180 : 1500 : 1500，
 * 人眼只要分清"短闪"和"隔很久"，不需要记住任何码序列。
 */
static void enum_digit(u32 n)
{
    if (n == 0u) {
        /* 0 不播成"什么都不闪"，否则和"漏看"分不开。 */
        led_on();
        mdelay(ENUM_ZERO_ON_MS);
        led_off();
        return;
    }

    while (n--) {
        led_on();
        mdelay(ENUM_BEAT_ON_MS);
        led_off();
        mdelay(ENUM_BEAT_OFF_MS);
    }
}

/* 一段 = 1.5 s 全黑 + 一个数字：段前的黑期让"上一段结束了"无可争议。 */
static void enum_field(u32 n)
{
    led_off();
    mdelay(ENUM_FIELD_GAP_MS);
    enum_digit(n);
}

/* 结束符：常亮 1.5 s 后灭。与数字 0 的 600 ms 差 2.5 倍，不会读成一段。 */
static void enum_end_mark(void)
{
    led_off();
    mdelay(ENUM_FIELD_GAP_MS);
    led_on();
    mdelay(ENUM_END_ON_MS);
    led_off();
}
#endif

#define ENUM_LOG_MAX 24
static u8 enum_log[ENUM_LOG_MAX];
static u8 enum_log_n;
static u8 enum_resets;
/* 第二十七轮：见过第一个 SETUP 完成（3）之后置位，用来屏蔽复位/连接噪声。 */
static u8 enum_setup_seen;
/*
 * 第四十一轮：播报用的三个计数。
 *
 *   数据/状态武装失败数 / 完成事件数 / 复位时 DATA TRB 的归宿
 *
 * 与 enum_log 解耦，不受 24 条上限和"只看最后 8 条"的截断影响。
 * 第 3 段的含义：
 *   0 = 复位时没有处于数据阶段的 TRB（无从判定）
 *   1 = 码 13：HWO 还是 1，控制器从头到尾没取过这条数据 TRB
 *   2 = 码 14：HWO 已清但没报完成，控制器取了却走不完
 * enum_cnt_arm 不播出，只作为"提早就播"的触发条件。
 */
static u8 enum_cnt_arm;
static u8 enum_cnt_fail;
static u8 enum_cnt_done;
static u8 enum_cnt_hwo;
/*
 * 第四十八轮（A+B 的 B）：给 DSTS.CONNECTSPD 一个**粘性**槽位。
 *
 * 前面所有读数都只描述 EP0 走了多远，没有一条能回答「控制器自己到底
 * 有没有看到链路」。主机侧 USBPcap 对枚举失败的设备是瞎的（第 47 轮
 * 已实测：4 分钟零帧，而同段用户做了两次插拔并看到两次「设备描述符
 * 请求失败」），所以这条信息只能由设备自己在播报里给出。
 *
 * 取值：
 *   0     = 从 connect 到现在，CONNECTSPD 一直读到 0 —— 控制器没看到链路
 *   1..7  = 最后一次读到的非 0 原始编码
 *
 * 编码表有两种流传很广的说法：databook 2.90a 是 1=HS / 2=FS / 3=LS / 0=SS，
 * 而本仓库早前记的是 3=HS。两个表里 HS 都不为 0，所以"0 = 没连上"这个
 * 判据不受影响；读到非 0 时按两条表分别对照即可定表。
 *
 * 用粘性锁存而不是播报时现场读：主机放弃后会自己撤销连接，现场读很可能
 * 已经把速度清回 0，那就把"曾经连上过"抹掉了。
 */
static u8 enum_speed_seen;
/*
 * 第五十轮（探针 ②）：SETUP 阶段的 XFERCOMPLETE 次数。
 *
 * 段 1（武装失败数）与段 2（完成数）只覆盖 DATA / STATUS；SETUP 本身到底
 * 有没有完成，此前从未单独读过。若这里读到 0，说明第一张 TRB 就没走完，
 * 问题要往前挪一层；读到 ≥1 则说明「SETUP 完成 → 解析 → NRDY(DATA) →
 * 武装」这条链确实成立，卡点仍在「控制器不取 DATA TRB」。
 */
static u8 enum_cnt_setup;
/*
 * 第五十一轮（探针 ③）：DATA 阶段的 XFERNOTREADY 次数。
 *
 * 第 50 轮读数 `0-0-1-1-3-1` 把两件事分开了：SETUP 链实测成立（段 5 = 3），
 * 而数据 TRB 的 HWO 仍是 1（段 3 = 1）⇒ 控制器没取。段 6 = 1（TRB 三字段
 * 未被改写）在「控制器没碰过」的前提下几乎是必然，不能当证据用。
 *
 * 于是剩下的问题只有一个：控制器对 ep1 **是不轮询，还是轮询了但拒绝**？
 * 这两种情形对应完全不同的修法，而 XFERNOTREADY 的重复次数正好能把它们
 * 分开 —— NRDY 本身就是控制器「我现在取不到可用的 TRB」的喊话。
 *
 *   0    = 长闪：一次 DATA 阶段 NRDY 都没有（与段 3 = 1 矛盾，说明阶段判定有问题）
 *   1    = 短闪 1 下：只喊过一次。我们武装之后它就不再轮询 ep1
 *   2..8 = 短闪 N 下：武装之后它仍在反复喊 ⇒ 它在轮询但拒绝这张 TRB
 *   9    = 封顶（≥9）：一直在刷，等同于持续拒绝
 */
static u8 enum_cnt_nrdy;
/*
 * 第五十七轮（探针 ④）：NRDY(DATA) 里成功装出数据 TRB 的次数。
 *
 * 第 55/56 轮的读数无法证明「响应 NRDY 武装」这条路成不成立：预装之后
 * ep0_data_armed 守卫把 NRDY(DATA) 分支整个挡掉了，那段代码从未执行。
 * 第 57 轮删掉预装与守卫，把唯一武装点放回 NRDY，本段就是这条路径的
 * 执行次数：
 *
 *   0    = 长闪：NRDY(DATA) 里一次都没装成 —— 卡点在武装命令本身
 *   1    = 短闪 1 下：第一笔数据阶段只等到一次 NRDY
 *   2..9 = 短闪 N 下：每笔控制传输的 NRDY 都装成了（与段 6 对照）
 */
static u8 enum_cnt_arm_data;

#ifdef ENUM
/* 一次 MMIO 读，不阻塞。非 0 才写，避免断开后把锁存值冲掉。 */
static void enum_sample_speed(void)
{
    u32 spd = read32(DWC3_DSTS) & DWC3_DSTS_CONNECTSPD_MASK;

    if (spd)
        enum_speed_seen = (u8)spd;
}
#endif

/*
 * 站点灯标（2026-10-23 第五版仪器）。
 *
 * 前面的版本只能证明"卡在 handle_devt 里"，再往下就分不开了 —— 单灯只有亮/灭。
 * 这一版在每个关键语句前闪 n 次、然后保持点亮：
 *
 *   看到 6 闪后长亮 ⇒ 卡在站点 6 之后
 *   看到 3 闪后长亮 ⇒ 卡在站点 3 之后
 *
 * 站点编号只在第一次总线复位时武装（`g_site_on`），所以插线前 ep0_init 里
 * 那一串 depcmd 不会闪。
 */
volatile u32 g_site_on;

/*
 * 2026-10-23 第二十三轮：站点重新编号，覆盖"指纹 6（SETUP 完成）之后到
 * depcmd 之间"的整条链，并加上复位/事件路径，用来分辨"卡在处理 SETUP"
 * 与"卡在处理主机复位风暴"。
 *
 *   1 SETUP 分支：dsb() + ctrl 解析完（handle_setup() 之前）
 *   2 SETUP 分支：handle_setup() 返回（数据阶段装好之前）
 *   3 ep0_nrdy（XFERNOTREADY）入口
 *   4 depcmd 入口：GUSB2PHYCFG 读完（PAR 写之前）
 *   5 depcmd：三个 PAR 写完
 *   6 depcmd：DEPCMD 写完
 *   7 任何 devt（总线）事件入口
 *   8 on_reset() 体入口
 *   9 XFERCOMPLETE 的 DATA 分支      10 XFERCOMPLETE 的 STATUS 分支
 *
 * 站点只在指纹第三次播完之后才武装（g_site_on），因此插线前 ep0_init 里的
 * depcmd 不会闪。每个站点**全局只播一次**（enum_site_once）：主机复位重试
 * 会让同一条语句再次执行，若每次都播，读数会被重复的组淹没；只播一次时，
 * 读数就是"代码走到过的最远点"，顺序即代码顺序。
 */
#define ENUM_SITE_NOP   2000000u
#define ENUM_SITE_GAP   4000000u
/*
 * 收尾长暗期（第二十二轮起）：原来最后一闪的暗期与闪间一样长，紧接着
 * led_on() 保持常亮，肉眼会把常亮数成第 n+1 下。收尾先留几个脉冲宽度的
 * 暗期再常亮：读数变成"闪 n 下 → 明显停一下 → 长亮"，长亮不计入 n。
 */
#define ENUM_SITE_TAIL  8000000u

static u32 g_site_used;

void enum_site(u32 n)
{
    u32 i;

    if (!g_site_on)
        return;

    led_off();
    enum_nop(ENUM_SITE_GAP);
    for (i = 0; i < n; i++) {
        led_on();
        enum_nop(ENUM_SITE_NOP);
        led_off();
        enum_nop(ENUM_SITE_NOP);
    }
    enum_nop(ENUM_SITE_TAIL);
    led_on();
}

/* 每个站点全局只播一次；顺序即代码执行顺序。 */
void enum_site_once(u32 n)
{
    if (!g_site_on || n == 0u || n > 31u)
        return;
    if (g_site_used & (1u << (n - 1u)))
        return;
    g_site_used |= (1u << (n - 1u));
    enum_site(n);
}

/*
 * 处理器指纹（2026-10-23 第二十轮）。
 *
 * 真机读数：插线瞬间灯就长亮、主机报错后依然长亮，中间没有任何心跳。
 * 泵的心跳每 2000 次泵一次、与事件速率无关，所以"心跳全无"只能说明
 * 第一个事件之后 CPU 就没回到主循环 —— 但单灯分不出是哪个处理器。
 *
 * 这里在每个处理器入口闪 n 下（亮/灭各约 37~112 ms，与心跳短脉冲同档），
 * 然后交回泵继续：
 *
 *   闪 N 下后长亮     = 进了 N 号处理器，且在其中卡死（泵的 led_on 还亮着）
 *   闪 N 下后恢复心跳 = 该处理器正常返回
 *   一直不闪、灯常灭   = 卡在 dwc3_event_poll() 里
 *
 * 与十二轮关掉的站点灯标的区别：只标"入口"一处，半周期短一个数量级，
 * 且最多 4 组，不会像每站点约 1 s 那样把每次尝试都踩爆。
 *
 *   1 总线复位        2 CONNECT_DONE      3 DISCONNECT     4 其他 devt
 *   5 ep0 阶段请求(XFERNOTREADY)          6 ep0 SETUP 完成
 *   7 ep0 数据/状态完成                   8 bulk 或未知端点事件
 */
#define ENUM_FP_NOP     3000000u
#define ENUM_FP_GAP     6000000u
/* 第二十二轮：指纹收尾长暗期（与站点同档），消除"第 n 下还是 n+1 下"。 */
#define ENUM_FP_TAIL    12000000u
/*
 * 2026-10-23 第二十一轮：4 → 3。
 *
 * 真机读数稳定停在第三组（1-2-7）之后，第四组永远不会出现；预算正好卡在 3
 * 就不会有第四组指纹和站点编号撞号。
 */
#define ENUM_FP_BUDGET  3u

static void __attribute__((unused)) enum_fp(u32 n)
{
    static u32 used;
    u32 i;

    if (used >= ENUM_FP_BUDGET)
        return;
    used++;

    led_off();
    enum_nop(ENUM_FP_GAP);
    for (i = 0; i < n; i++) {
        led_on();
        enum_nop(ENUM_FP_NOP);
        led_off();
        enum_nop(ENUM_FP_NOP);
    }
    /* 第二十二轮：收尾长暗期，避免常亮被数成第 n+1 下。见 enum_site()。 */
    enum_nop(ENUM_FP_TAIL);
    led_on();

    /*
     * 第二十四轮：**不再武装站点**（原来这里是 g_site_on = 1）。
     *
     * 站点是"发生即播报"，会和静默后的时间线播报混在一起，而两者都用
     * "闪 n 下"表示，现场分不清一组读数属于哪套。这一轮只保留指纹
     * （1-2-6：确认镜像在跑、走到了第一次 SETUP），全部站点静默。
     */
}

/*
 * 第三十五轮：每请求播报（2026-10-24）。
 *
 * 第三十四轮的八级台阶全部到齐，卡点已推进到「第一笔控制传输完成、主机
 * 随后沉默」。台阶是一次性的，`1长3短` 之后就是观测盲区；这一轮换成
 * **可重复**的播报：每解析完一笔 SETUP 就播一次，主机重试、复位重来都
 * 会重新播。
 *
 * 位置：解析完成之后、handle_setup() 之前。此时主机正在等数据/状态阶段，
 * 我们只是 NAK 得久一点（协议允许）；**绝不放在状态阶段之后** ——
 * SET_ADDRESS 之后的 2 ms 是地址切换窗口，那里一个闪都不能加。
 *
 * 编码（区别于指纹的「n 短」与心跳的单闪）：2 长 + n 短。
 *   1 GET_DESCRIPTOR(device)      2 GET_DESCRIPTOR(config)
 *   3 GET_DESCRIPTOR(string)      4 GET_DESCRIPTOR(qualifier)
 *   5 SET_ADDRESS                 6 SET_CONFIGURATION
 *   7 其它（含不认识的请求）
 *
 * 时长：ENUM_NOP_UNIT 6M 约 75~225 ms，所以长闪 6M、短闪 3M（与指纹的
 * 计数单位一致，台架数得清）。单笔播报约 0.4~2 s；控制传输超时远大于
 * 此值，但这也意味着这一轮枚举会明显变慢。
 */
#define ENUM_REQ_LONG   6000000u
#define ENUM_REQ_SHORT  3000000u
#define ENUM_REQ_GAP    3000000u
#define ENUM_REQ_TAIL   3000000u

static void __attribute__((unused)) enum_req(u32 n)
{
    u32 i;

    if (n == 0u || n > 7u)
        n = 7u;

    led_off();
    enum_nop(ENUM_REQ_GAP);

    /* 前缀：2 长，和「n 短」的指纹、账本编码分开。 */
    for (i = 0; i < 2u; i++) {
        led_on();
        enum_nop(ENUM_REQ_LONG);
        led_off();
        enum_nop(ENUM_REQ_SHORT);
    }

    for (i = 0; i < n; i++) {
        led_on();
        enum_nop(ENUM_REQ_SHORT);
        led_off();
        enum_nop(ENUM_REQ_SHORT);
    }

    enum_nop(ENUM_REQ_TAIL);
    led_on();
}

/* 请求 → 播报编号。只用于 -Enum 的灯语，不参与任何协议判断。
 * 第三十六轮起不再有调用者（每请求播报已撤），保留以备后续轮次复用。 */
static u32 __attribute__((unused)) enum_req_code(void)
{
    if ((ctrl.bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
        return 7u;

    if (ctrl.bRequest == USB_REQ_GET_DESCRIPTOR) {
        switch ((ctrl.wValue >> 8) & 0xff) {
        case USB_DT_DEVICE:    return 1u;
        case USB_DT_CONFIG:    return 2u;
        case USB_DT_STRING:    return 3u;
        case USB_DT_DEVICE_QUALIFIER: return 4u;
        default:               return 7u;
        }
    }
    if (ctrl.bRequest == USB_REQ_SET_ADDRESS)       return 5u;
    if (ctrl.bRequest == USB_REQ_SET_CONFIGURATION) return 6u;
    return 7u;
}

/* 纳秒级：边界检查 + 一个字节写入。绝对不能在这里闪灯。 */
static void enum_rec(u8 code)
{
    /*
     * 第二十七轮：**噪声屏蔽**。
     *
     * 第二十六轮读数（倒序播报）是 4-2-1-4-4-2-1-4 ⇒ 按时间顺序即
     * 4-1-2-4-4-1-2-4，两个"总线复位 → CONNECT_DONE"循环，全是主机报
     * 未知设备之后的重试噪声，一条关键记录都没有。而指纹里已经有 6
     * （SETUP 完成）—— 同一事件的同一分支里 3 是无条件记录的（见
     * case EP0_PHASE_SETUP），所以账本里必然出现过 3；它要么被更早的一次
     * 播报清掉、要么被这串噪声挤到了播报之外。
     *
     * 不管哪种，处理方式一样：**一旦见过 3，就不再记 1/2/4**。
     * 这样账本里只剩 3/5/7/10/12/13/14/15，任何一版读数都是有效信息，
     * 再也不会被复位风暴淹没。标志是粘性的，不随清空回滚。
     */
    if (enum_setup_seen &&
        (code == ENUM_LOG_RESET || code == ENUM_LOG_CONNECT ||
         code == ENUM_LOG_ARM_SETUP))
        return;

    if (code == ENUM_LOG_SETUP)
        enum_setup_seen = 1;

    if (enum_log_n < ENUM_LOG_MAX)
        enum_log[enum_log_n++] = code;

    /*
     * 第四十轮：顺带累加播报字段。
     *
     *   12（ARM_FAIL）→ 武装失败数：STARTTRANSFER 被控制器拒绝，控制阶段
     *                   根本推不下去，这是最关键的一段。
     *   7 / 8        → 完成事件数：数据/状态 TRB 真的回来了。
     *   5 / 6        → 武装尝试数，不播出，只用来触发"提早就播"。
     */
    if (code == ENUM_LOG_ARM_FAIL) {
        if (enum_cnt_fail < ENUM_COUNT_MAX)
            enum_cnt_fail++;
    } else if (code == ENUM_LOG_DONE_DATA || code == ENUM_LOG_DONE_STATUS) {
        if (enum_cnt_done < ENUM_COUNT_MAX)
            enum_cnt_done++;
    } else if (code == ENUM_LOG_SETUP) {
        /* 第五十轮（探针 ②）：SETUP 阶段的完成次数。 */
        if (enum_cnt_setup < ENUM_COUNT_MAX)
            enum_cnt_setup++;
    } else if (code == ENUM_LOG_ARM_DATA || code == ENUM_LOG_ARM_STATUS) {
        if (enum_cnt_arm < ENUM_COUNT_MAX)
            enum_cnt_arm++;
    }
}

/* 只在"这次尝试已经失败、并且安静下来"之后调用。播报完清空，下一段重新计数。 */
static void enum_dump(void)
{
#ifdef ENUM_DUMP_ON
    /*
     * 第四十一轮起固定为「失败数 → 完成数 → 第 3 段 → 结束符」，
     * 第四十八轮插入第 4 段，第四十九轮追加第 5、6 段；
     * 第五十轮保留 6 段、把第 5、6 段整体换掉（A/B 退役）；
     * 第五十一轮只换第 6 段（TRB 回读退役，改为 DATA 阶段 NRDY 次数）：
     *
     *   失败数 / 完成数 / DATA 槽 HWO 归宿 / NRDY 装 DATA 次数 / SETUP 完成数 / DATA 阶段 NRDY 数
     *
     * 判读：
     *   第 1 段 ≥1            → 控制器拒绝 STARTTRANSFER（端点/TRB 侧）
     *   第 1 段 0、第 2 段 ≥1 → 按事件端点武装这条路走通了
     *
     * 第四十二轮：第 3 段曾是 **ep1（物理端点 1，即控制 IN）的
     * SETTRANSFRESOURCE 索引**，读数恒为 2，证明索引已随 STARTTRANSFER 下发，
     * 但完成事件仍是 0 ⇒ 索引假设证伪。
     *
     * 第四十三轮：第 3 段换成 **GTXFIFOSIZ(0) 读回的 size code**，读数 6，
     * 证明口径正确、写入生效且撑过了 DEPSTARTCFG —— FIFO 尺寸假设排除。
     *
     * 第四十四轮：第 3 段换回 **复位时 DATA 槽 TRB 的归宿**（来源是
     * on_reset() 里 ep0_phase == EP0_PHASE_DATA 时的 HWO 采样）：
     *   0 = 复位时没有处于数据阶段的 TRB（无从判定）
     *   1 = HWO 还是 1，控制器从头到尾没取过这条数据 TRB
     *   2 = HWO 已清但没报完成，控制器取了却走不完
     * FIFO 已确认正常，这一位用来把"不取 TRB"和"取了走不完"重新分开。
     */
    enum_field(enum_cnt_fail);
    enum_field(enum_cnt_done);
    enum_field(enum_cnt_hwo > ENUM_COUNT_MAX ? ENUM_COUNT_MAX : enum_cnt_hwo);
    /*
     * 第四十八轮：第 4 段 = DSTS.CONNECTSPD 的粘性读数（第五十三轮退役）。
     *   0    → 长闪一下：控制器自始至终没看到链路
     *   1..7 → 短闪 N 下：最后一次看到的非 0 原始编码
     *          （HS 在 databook 2.90a 里是 1，在本仓库早前记法是 3）
     */
    /*
     * 第五十七轮：第 4 段换成 **NRDY(DATA) 里成功装出数据 TRB 的次数**。
     *
     * 第 53/54 轮这里播的是实际采用的分区 size code，读数先 `0` 后 `0`，
     * 证明 boot 分区没接上、一直写硬编码 128 words；FIFO 这条线在第 54 轮
     * 已经出局，该探针不再需要。
     *
     * 现在要判的是「第 55/56 轮为什么读数一模一样」。那两轮在 SETUP 完成
     * 回调里预装数据 TRB，并用 ep0_data_armed 把 NRDY(DATA) 分支挡掉 ——
     * 参考实现唯一的武装点从未执行。本轮删掉预装与守卫，段 4 就是这条
     * 路径的执行次数：
     *
     *   0    = 长闪：NRDY(DATA) 到了但一次都没装成（武装命令被拒）
     *   1..9 = 短闪 N 下：NRDY 里装成了 N 次
     *
     * 判读：段 4 ≥1 且段 2 ≥1 → 参考路径成立、枚举打通；
     *       段 4 ≥1 且段 2 = 0 → 命令被接受但控制器仍不取 TRB（与 51~56 轮同）；
     *       段 4 = 0          → 连武装都没成，看段 6（NRDY 次数）定位。
     */
    enum_field(enum_cnt_arm_data > ENUM_COUNT_MAX ? ENUM_COUNT_MAX
                                                  : enum_cnt_arm_data);
    /*
     * 第五十轮：第 5、6 段。
     *
     * 第四十九轮的 A/B 已把两项嫌疑排除：段 5 = 2 ⇒ DEPEVT_EP = 1 = EP0_IN，
     * 武装端点没错；段 6 = 4 ⇒ DALEPENA 低两位已置起（但那只是我们自己的
     * 影子写，不能反证 DEPCFG 生效）。剩下嫌疑 3，于是把这两段换成：
     *
     *   第 5 段（探针 ②）= SETUP 阶段的 XFERCOMPLETE 次数
     *       0    → 长闪：第一张 TRB 就没走完，问题要往前挪一层
     *       1..9 → 短闪 N 下：SETUP 完成链成立，卡点仍在「不取 DATA TRB」
     *       （第 50 轮实测 = 3，该段保留）
     */
    enum_field(enum_cnt_setup);
    /*
     * 第五十一轮：第 6 段换成 DATA 阶段的 NRDY 次数（探针 ③）。
     *
     * 第 50 轮段 6（数据 TRB 回读比对）读数 1，但段 3 = 1 已经说明控制器
     * 从没取过那张 TRB —— 在「没人碰」的前提下「没被改写」是必然的，该段
     * 无法再提供信息，退役。换成一个能把「不轮询」和「轮询但拒绝」分开的
     * 计数：
     *
     *   0    = 长闪：没有 DATA 阶段 NRDY（与段 3 = 1 矛盾）
     *   1    = 短闪 1 下：只喊一次，武装后控制器不再轮询 ep1
     *   2..8 = 短闪 N 下：武装后仍在反复喊 ⇒ 轮询了但拒绝这张 TRB
     *   9    = 封顶：一直在刷
     */
    enum_field(enum_cnt_nrdy);
    enum_end_mark();

    enum_cnt_arm    = 0;
    enum_cnt_fail   = 0;
    enum_cnt_done   = 0;
    enum_cnt_hwo    = 0;
    enum_speed_seen = 0;
    enum_cnt_setup  = 0;
    enum_cnt_nrdy   = 0;
    enum_cnt_arm_data = 0;
    enum_log_n = 0;
#else
    /*
     * 2026-10-23 第十二轮：播报整体关闭（定义 `ENUM_DUMP_ON` 可恢复）。
     *
     * 站点读数证明主机在反复复位重试，也就是在枚举**还没结束**的时候就会
     * 走到这些"失败播报"分支；每播一次阻塞约 2 s，等于诊断自己把下一次
     * 尝试继续踩死。这一轮的判据改看主机侧，所以先把播报关掉。
     */
    enum_cnt_arm  = 0;
    enum_cnt_fail = 0;
    enum_cnt_done = 0;
    enum_cnt_hwo  = 0;
    enum_speed_seen = 0;
    enum_cnt_setup  = 0;
    enum_cnt_nrdy   = 0;
    enum_cnt_arm_data = 0;
    enum_log_n = 0;
#endif
}
#endif
#endif /* HOLD / ENUM */

static int ep0_queue_setup(void)
{
    ep0_trb_setup.bpl  = (u32)(unsigned long)setup_buf;
    ep0_trb_setup.bph  = 0;
    ep0_trb_setup.size = 8;
    ep0_trb_setup.ctrl =
                  DWC3_TRB_CTRL_TRBCTL(DWC3_TRBCTL_CONTROL_SETUP)
                | DWC3_TRB_CTRL_HWO
                | DWC3_TRB_CTRL_LST
                | DWC3_TRB_CTRL_IOC
                | DWC3_TRB_CTRL_ISP_IMI;
    ep0_phase = EP0_PHASE_SETUP;
#ifdef ENUM
    enum_rec(ENUM_LOG_ARM_SETUP);
#endif
    dma_wmb();

    /* Nothing re-arms ep0 if this fails, so the device is deaf from here on.
     * The caller must be told: a failure here and a failure in the data stage
     * look identical from the host's side, but only this one has no second
     * chance. */
    return dwc3_ep_start_xfer(EP0_OUT, (u64)(unsigned long)&ep0_trb_setup, 0);
}

static int ep0_queue_data(u32 phys_ep, const void *buf, u32 len)
{
    u32 xfer = len;

#ifdef ENUM
    /*
     * 第二十五轮：数据阶段入口采样上一张 TRB 的 HWO。
     *
     * 第五十六轮起三阶段各占一格，这里采的是 SETUP 槽：
     * 进数据阶段时，SETUP 那条 TRB 交回了吗。
     */
    enum_rec((ep0_trb_setup.ctrl & DWC3_TRB_CTRL_HWO)
                 ? ENUM_LOG_OVERWRITE : ENUM_LOG_RELEASED);
#endif

    /*
     * An OUT length must be a whole number of maximum-size packets; the
     * surplus is discarded.
     *
     * 第四十轮：判据从"端点号奇偶"改成**请求方向**。控制传输的数据/状态
     * TRB 现在一律装在物理端点 0，奇偶不再携带方向；只有 OUT 方向
     * （bmRequestType bit7 = 0）才需要整包对齐，IN 方向必须按真实长度装，
     * 否则 18 字节的设备描述符会被撑成 64 字节。
     */
    if (!(ctrl.bRequestType & USB_DIR_IN))
        xfer = (len + EP0_MAXPACKET - 1) & ~(u32)(EP0_MAXPACKET - 1);

    ep0_trb_data.bpl  = (u32)(unsigned long)buf;
    ep0_trb_data.bph  = 0;
    ep0_trb_data.size = xfer;
    ep0_trb_data.ctrl =
                  DWC3_TRB_CTRL_TRBCTL(DWC3_TRBCTL_CONTROL_DATA)
                | DWC3_TRB_CTRL_HWO
                | DWC3_TRB_CTRL_LST
                | DWC3_TRB_CTRL_IOC
                | DWC3_TRB_CTRL_ISP_IMI;
    ep0_phase = EP0_PHASE_DATA;
#ifdef ENUM
    enum_rec(ENUM_LOG_ARM_DATA);
#endif
    dma_wmb();
    return dwc3_ep_start_xfer(phys_ep, (u64)(unsigned long)&ep0_trb_data, 0);
}

static int ep0_queue_status(u32 phys_ep, int three_stage)
{
#ifdef ENUM
    /*
     * 第二十四轮的判决点：装状态 TRB 之前，上一阶段 TRB 的 HWO 还在不在。
     *
     * 第五十六轮起三阶段各占一格，这里采样的是 DATA 槽（两阶段控制传输时
     * 它还是空的，HWO=0，等价于"已交回"，不产生误报）：
     * HWO=1 ⇒ 控制器还持有上一阶段那条 TRB 就开始下一阶段；
     * HWO=0 ⇒ 已交回。
     */
    enum_rec((ep0_trb_data.ctrl & DWC3_TRB_CTRL_HWO)
                 ? ENUM_LOG_OVERWRITE : ENUM_LOG_RELEASED);
#endif
    ep0_trb_status.bpl  = 0;
    ep0_trb_status.bph  = 0;
    ep0_trb_status.size = 0;
    ep0_trb_status.ctrl =
                  DWC3_TRB_CTRL_TRBCTL(three_stage ? DWC3_TRBCTL_CONTROL_STATUS3
                                                   : DWC3_TRBCTL_CONTROL_STATUS2)
                | DWC3_TRB_CTRL_HWO
                | DWC3_TRB_CTRL_LST
                | DWC3_TRB_CTRL_IOC
                | DWC3_TRB_CTRL_ISP_IMI;
    ep0_phase = EP0_PHASE_STATUS;
#ifdef ENUM
    enum_rec(ENUM_LOG_ARM_STATUS);
#endif
    dma_wmb();
    return dwc3_ep_start_xfer(phys_ep, (u64)(unsigned long)&ep0_trb_status, 0);
}

static void ep0_stall(void)
{
    TRACE_STALL();

    /*
     * 只重新装 setup TRB，不发 SETSTALL（§9.23 修复 #6）。
     *
     * 反证要说清楚：Linux 的 dwc3_ep0_stall_and_restart() 确实会对 ep0 发
     * SETSTALL，再 ep0_out_start()，所以"内核不发"这个说法是错的。但仍然
     * 不发，理由两条，都来自真机：
     *   1. 唯一一次完整枚举成功的那版构建（§9.23）不带这一步；
     *   2. 带 SETSTALL 的版本曾在命令接口上挂死过 —— 当时归因于
     *      fire-and-forget 的 DEPCMD，现在 dwc3_depcmd() 会轮询、不会再
     *      无声丢弃状态，但"没被证明过"这一点没变。
     * 代价：主机看到的是超时而不是 STALL，枚举更慢；可控。
     */
    (void)ep0_queue_setup();
}

/* ---- bulk endpoints ----------------------------------------------------- */

static void bulk_endpoints_enable(void)
{
    int ret;

    /* No DEPSTARTCFG: it would reset the resources ep0 is already holding. */
    ret = dwc3_ep_config(EP_OUT, DWC3_DEPCMD_TYPE_BULK, BULK_MAXPACKET);
    if (ret) {
        usb_bulk_error = USB_BULK_ERR_CFG_OUT;
        return;
    }

    ret = dwc3_ep_config(EP_IN, DWC3_DEPCMD_TYPE_BULK, BULK_MAXPACKET);
    if (ret) {
        usb_bulk_error = USB_BULK_ERR_CFG_IN;
        return;
    }

    dwc3_ep_enable(EP_OUT);
    dwc3_ep_enable(EP_IN);
}

/* ---- standard request handling ------------------------------------------ */

static int handle_get_descriptor(void)
{
    u8 type  = (u8)(ctrl.wValue >> 8);
    u8 index = (u8)(ctrl.wValue & 0xff);

    switch (type) {
    case USB_DT_DEVICE:
        ep0_data = device_desc;
        ep0_data_len = sizeof(device_desc);
        return 0;
    case USB_DT_CONFIG:
        ep0_data = config_desc;
        ep0_data_len = sizeof(config_desc);
        return 0;
    case USB_DT_DEVICE_QUALIFIER:
        ep0_data = qualifier_desc;
        ep0_data_len = sizeof(qualifier_desc);
        return 0;
    case USB_DT_STRING:
        switch (index) {
        case 0: ep0_data = str_langid;  ep0_data_len = sizeof(str_langid);  return 0;
        case 1: ep0_data = str_vendor;  ep0_data_len = sizeof(str_vendor);  return 0;
        case 2: ep0_data = str_product; ep0_data_len = sizeof(str_product); return 0;
        case 3: ep0_data = str_serial;  ep0_data_len = sizeof(str_serial);  return 0;
        }
        return -1;
    }
    return -1;
}

static int handle_setup(void)
{
    ep0_data = 0;
    ep0_data_len = 0;

    if ((ctrl.bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
        return -1;

    switch (ctrl.bRequest) {
    case USB_REQ_GET_DESCRIPTOR:
        if (handle_get_descriptor() < 0)
            return -1;
        if (ep0_data_len > ctrl.wLength)
            ep0_data_len = ctrl.wLength;
        return 0;

    case USB_REQ_SET_ADDRESS:
        /*
         * 第三十五轮：恢复「当场写 DCFG」。
         *
         * ZV-E10 参考实现的注释写得很明确：
         *   "DWC3 wants the address programmed now; it applies it after the
         *    status stage itself."
         * 也就是控制器自己在状态阶段结束时切换地址；我们第三十三轮改成
         * 「状态阶段完成之后再写」，结果是主机第一笔到新地址的请求无人
         * 应答（USB 2.0 §9.4.6 要求状态阶段后 2 ms 内就能应答），Windows
         * 因此判「未知设备」并彻底沉默。
         */
        dwc3_set_address(ctrl.wValue & 0x7f);
        return 0;

    case USB_REQ_SET_CONFIGURATION:
        if ((ctrl.wValue & 0xff) == 1) {
            /* Deferred to the main loop: an endpoint command issued inside
             * the SETUP stage can stall the command interface and stop ep0
             * re-arming.
             *
             * Only the first one arms anything.  Windows configures the
             * device during enumeration and the fastboot host sends a second
             * SET_CONFIGURATION; re-running the endpoint commands with a bulk
             * transfer already queued wedges ep0 (ZV-E10 README). */
            if (!configured) {
                configured = 1;
                /* A fresh configuration invalidates errors left by the
                 * previous one. on_reset() clears them too, but a plain
                 * SET_CONFIGURATION(0) -> (1) cycle does not reset the bus. */
                usb_bulk_error = 0;
                bulk_enable_pending = 1;
            }
            return 0;
        }
        if ((ctrl.wValue & 0xff) == 0) {
            configured = 0;
            return 0;
        }
        return -1;

    case USB_REQ_GET_CONFIGURATION:
        ep0_buf[0] = (u8)(configured ? 1 : 0);
        ep0_data = ep0_buf;
        ep0_data_len = 1;
        return 0;

    case USB_REQ_GET_STATUS:
        ep0_buf[0] = 0x00;   /* bus powered, matching bmAttributes 0x80 */
        ep0_buf[1] = 0x00;
        ep0_data = ep0_buf;
        ep0_data_len = 2;
        if (ep0_data_len > ctrl.wLength)
            ep0_data_len = ctrl.wLength;
        return 0;

    case USB_REQ_SET_INTERFACE:
        return 0;

    case USB_REQ_GET_INTERFACE:
        ep0_buf[0] = 0;
        ep0_data = ep0_buf;
        ep0_data_len = 1;
        return 0;

    case USB_REQ_CLEAR_FEATURE:
    case USB_REQ_SET_FEATURE:
        return 0;
    }
    return -1;
}

/* ---- event handling ----------------------------------------------------- */

/*
 * Which step of ep0_init() failed, for the -Enum/-Hold report only.
 * 1 = DEPSTARTCFG, 2 = DEPCFG(out), 3 = DEPCFG(in), 4 = enable + arm.
 */
#ifdef ENUM
static int ep0_fail_stage;
#define EP0_STAGE(s)    (ep0_fail_stage = (s))
#else
#define EP0_STAGE(s)    ((void)0)
#endif

static int ep0_init(void)
{
    int ret;

    /* Once per bus reset; each endpoint then claims its resource at DEPCFG. */
    EP0_STAGE(1);
    ret = dwc3_ep_start_config();
    if (ret)
        return ret;

    EP0_STAGE(2);
    ret = dwc3_ep_config(EP0_OUT, DWC3_DEPCMD_TYPE_CONTROL, EP0_MAXPACKET);
    if (ret)
        return ret;

    EP0_STAGE(3);
    ret = dwc3_ep_config(EP0_IN, DWC3_DEPCMD_TYPE_CONTROL, EP0_MAXPACKET);
    if (ret)
        return ret;

    EP0_STAGE(4);
    dwc3_ep_enable(EP0_OUT);
    dwc3_ep_enable(EP0_IN);
    return ep0_queue_setup();
}

static void on_reset(void)
{
    int rc;

#ifdef ENUM
    /* 站点 8：on_reset() 体入口。配合站点 7 使用：7 亮而 8 不亮 ⇒ 卡在
     * handle_devt() 的入口到 on_reset() 之间的分支判断；8 亮而后续站点
     * 不亮 ⇒ 卡在下面的 DALEPENA / DCFG 写或 ep0_init() 的 depcmd 上。 */
    enum_site_once(8);
#endif

    configured = 0;

    /*
     * Drop the bulk endpoints before ep0_init() pulls their resources away,
     * and release anyone waiting on a transfer that is now void.
     */
    dwc3_ep_disable(EP_OUT);
    dwc3_ep_disable(EP_IN);

    bulk_out_busy = 0;
    bulk_in_busy = 0;
    bulk_epoch++;

    /* The reset voids any error the previous session left behind; otherwise
     * usb_bulk_send() would short-circuit to 0 for the whole next session. */
    usb_bulk_error = 0;

    /*
     * 2026-10-23 第十四轮：与 7M4 / ZV-E10 参考实现对齐 —— 复位后**完整
     * 重跑 ep0_init()**（DEPSTARTCFG + DEPCFG×2 + DALEPENA + 装 setup TRB）。
     *
     * 此前这里只清地址 + 重装 setup TRB，理由是"运行态发 DEPSTARTCFG 会把
     * 端点洗掉、此后收不到 SETUP"。那条读数出自事件缓冲还没 4 KB 对齐的
     * 时期（§9.23.2 #4 记过：那时事件时有时无、行为随机）。
     *
     * 现在底层 bug 都已修完，唯独"数据阶段装上却不完成"解决不了；
     * 而参考实现在两处事件里都重配 ep0，是这一族芯片上唯一验证过的做法。
     */
#ifdef ENUM
    /*
     * 第二十五轮：**复位前**的最后一个判决点。
     *
     * 下面 ep0_init() 会把 SETUP 槽重写，所以必须在这里看：如果上一个
     * 装的是 DATA TRB、而且它的 HWO 还是 1，说明控制器从头到尾没碰过它 ——
     * 传输根本没开始；HWO 已经是 0 而 XFERCOMPLETE 没来，说明控制器收了
     * TRB 却不报完成，那是另一类问题（事件/中断侧）。
     *
     * 读数形态：`13-4-1`（没被取用）或 `14-4-1`（被取用但没完成）。
     */
    if (ep0_phase == EP0_PHASE_DATA) {
        int held = (ep0_trb_data.ctrl & DWC3_TRB_CTRL_HWO) != 0;

        /*
         * 第四十一轮：这就是播报第 3 段的来源。
         * 1 = 控制器没取过（HWO 还在），2 = 取了但没走完（HWO 已清）。
         */
        enum_cnt_hwo = held ? 1u : 2u;
        enum_rec(held ? ENUM_LOG_OVERWRITE : ENUM_LOG_RELEASED);
    }
#endif

    dwc3_set_address(0);
    rc = ep0_init();

    /* One report per bus reset: counted, reported later. */
    TRACE_RESET();

#ifdef ENUM
    /*
     * 2026-10-23 第十二轮：这里不再播报。
     *
     * 上一轮的读数（站点 1–6 全过、且在循环）说明主机在**反复复位重试**，
     * 那么"第二次复位就播报"等于每两轮就往控制传输里塞 2 s 阻塞 ——
     * 诊断动作本身在阻止枚举成功。这一轮只留记账，判据改看主机侧。
     */
    enum_resets++;
    enum_rec(ENUM_LOG_RESET);
    if (rc)
        enum_rec(ENUM_LOG_ARM_FAIL);
#else
    (void)rc;
#endif
}

static void handle_depevt(u32 event)
{
    u32 ep     = DEPEVT_EP(event);
    u32 type   = DEPEVT_TYPE(event);

    /*
     * 第三十六轮：这里原来的入口指纹（enum_fp 5/6/7/8）已撤。
     *
     * 指纹每组要阻塞约 0.4~1.1 s，而它就插在 SETUP 解析与数据阶段武装之间
     * —— 等于诊断动作自己推迟每一笔控制传输。第三十五轮"两次请求就放弃、
     * 越插越少"的读数，最可能就是它造成的。这一轮事件路径零灯语，只记账。
     */

    /*
     * 2026-10-23 第十六轮：XFERNOTREADY 先按**类型**处理，不参与下面的端点号过滤。
     *
     * 依据是配置本身，不是猜测：bulk 端点的 DEPCFG 里我们只使能了
     * `XFER_COMPLETE_EN | XFER_IN_PROGRESS_EN`（见 dwc3_ep_config），
     * **没有** `XFER_NOT_READY_EN` —— 所以这个核上唯一会报 XFERNOTREADY 的
     * 端点就是 ep0。它属于谁，从类型就能确定，不需要读端点号。
     *
     * 第四十一轮：`DEPEVT_EP()` 的可信度已由读数确认 —— 第四十轮第 3 段
     * 播报 `2`，即 NRDY 落在物理端点 1，与内核反汇编的取位
     * （`ubfx r3,r2,#1,#5`）一致。所以这里仍然按类型优先处理，但拿到
     * `ep` 之后**用它武装**，不再把它丢掉。
     */
    if (type == DWC3_DEPEVT_XFERNOTREADY)
        goto ep0_nrdy;

    if (ep == EP_OUT || ep == EP_IN) {
        /* With XFER_IN_PROGRESS_EN set this core may report a finished TRB as
         * XferInProgress rather than XferComplete. */
        if (type == DWC3_DEPEVT_XFERCOMPLETE ||
            type == DWC3_DEPEVT_XFERINPROGRESS) {
            /* TRB.size is the residue, not the count. */
            if (ep == EP_OUT) {
                bulk_out_residue = bulk_trb[0].size & DWC3_TRB_SIZE_MASK;
                bulk_out_busy = 0;
            } else {
                bulk_in_residue = bulk_trb[1].size & DWC3_TRB_SIZE_MASK;
                bulk_in_busy = 0;
                bulk_in_evt_seq++;
                bulk_in_last_type = type;
            }
        }
        return;
    }

    if (ep > EP0_IN)
        return;

    switch (type) {
    case DWC3_DEPEVT_XFERCOMPLETE:
        switch (ep0_phase) {
        case EP0_PHASE_SETUP:
            /*
             * A new control request has landed in setup_buf.
             *
             * 第三十三轮：原来的 dsb() 已删除，换成只排序、不等完成的
             * dmb()。dsb 会等所有未完成的事务落地，是这段窗口里唯一会
             * 无限等待的动作；dmb 只保证读顺序。
             */
            __asm volatile ("dmb" ::: "memory");
            {
                /*
                 * 第三十四轮：逐字段 volatile 写。字段偏移 0/1/2/4/6 全部
                 * 自然对齐，编译器不能再把 bRequest+wValue 合并成一条
                 * 落向 ctrl+1 的非对齐字访问。
                 */
                volatile struct usb_ctrlrequest *c = &ctrl;

                c->bRequestType = setup_byte(0);
                c->bRequest     = setup_byte(1);
                c->wValue       = (u16)(setup_byte(2) | ((u16)setup_byte(3) << 8));
                c->wIndex       = (u16)(setup_byte(4) | ((u16)setup_byte(5) << 8));
                c->wLength      = (u16)(setup_byte(6) | ((u16)setup_byte(7) << 8));
            }
            /*
             * 第四十一轮：这里不再预算状态阶段端点。
             *
             * 第四十轮在此把它定死为物理端点 0，读数证明控制器要的是
             * 端点 1。现在状态 TRB 一律装 NRDY 事件点名的那个端点。
             */
#ifdef ENUM
            /*
             * 第三十六轮：每请求播报（2 长 + n 短）已撤。
             *
             * 它每笔要阻塞 0.4~1.1 s，而位置正在 handle_setup() 之前 ——
             * 主机在等数据阶段，我们却先黑着灯数脉冲。第三十五轮读数里
             * 主机只读两次设备描述符就放弃，这是首要嫌疑。
             * 现在这里只留一个字节的记账，请求类型由账本后续可推断。
             */

            /* 任何 SETUP 都在这里留痕（含 wLength==0 的请求）。 */
            enum_rec(ENUM_LOG_SETUP);
#endif

            /* Three short blinks: a setup packet was parsed. */
            TRACE_SETUP();

            /* handle_setup() has to run before any mark: it fills in the
             * request descriptor the marks themselves are about. */
            {
                int rc = handle_setup();
                if (rc < 0) {
#ifdef ENUM
                    /*
                     * 不认识的请求 ⇒ 枚举必然失败。第三十六轮起这里只记账，
                     * **不再当场播报**：Dump 一旦在事件路径里跑，就会把后面
                     * 主机的重试也一起拖死；失败由静默后的整段播出可见。
                     */
                    enum_rec(ENUM_LOG_REJECTED);
#endif
                    ep0_stall();
                    break;
                }
            }

            /*
             * 第五十七轮：回到 ZV-E10 / 7M4 的武装点 —— SETUP 完成回调
             * **只解析、不装数据 TRB**，数据与状态阶段一律等 NRDY 再装
             * （见下面 DWC3_DEPEVT_XFERNOTREADY 分支）。
             *
             * 第 55 轮在这里预装、第 56 轮又试过换槽，两轮读数
             * （0-0-1-0-5-9 / 0-0-1-0-5-9）逐字段相同 ⇒ 预装这条偏离关闭。
             *
             * wLength == 0 的请求（SET_ADDRESS 等）状态阶段本来就走 NRDY。
             */
            break;

        case EP0_PHASE_DATA:
            /*
             * 只记账，不武装。状态 TRB 一律等 NRDY(控制阶段=2) 再装 ——
             * ZV-E10 / 7M4 两份能枚举的实现都是这样，第 30 轮的
             * `7 → 14 → 6 → 11 → 3` 正是「提前装好、控制器随后才要状态」
             * 的结果：TRB 已 HWO，命令被接受（无 12），但既不完成也不报错。
             */
#ifdef ENUM
            enum_site_once(9);       /* 进了 DATA 分支 */
#endif
            TRACE_BLIP();
#ifdef ENUM
            enum_rec(ENUM_LOG_DONE_DATA);
#endif
            break;

        case EP0_PHASE_STATUS:
#ifdef ENUM
            enum_rec(ENUM_LOG_DONE_STATUS);
#endif
            /*
             * 第三十五轮：这里不再有延迟落地址 —— SET_ADDRESS 已在
             * handle_setup() 里当场写 DCFG，由控制器在状态阶段结束时
             * 自行切换（与 ZV-E10 参考一致）。这一段绝不能加任何灯语：
             * 状态阶段之后 2 ms 内必须能在新地址上应答。
             */
            /* Re-arm, so a SETUP is always outstanding. */
            (void)ep0_queue_setup();
            TRACE_DONE();
            break;

        default:
#ifdef ENUM
            /* ep0_phase 被踩坏：既不是 SETUP/DATA 也不是 STATUS。 */
            enum_rec(ENUM_LOG_DONE_OTHER);
#endif
            break;
        }
        break;

    case DWC3_DEPEVT_XFERNOTREADY:
    /* 上面按类型提前跳进来：只有 ep0 会产生这个事件。 */
    ep0_nrdy:
#ifdef ENUM
        /* 站点 3：XFERNOTREADY 入口。SETUP 处理完之后主机要数据/要状态
         * 就走这里；看不到它而前面站点 2 已经亮过，说明主机没再要数据，
         * 卡点在"回到泵之后"。 */
        enum_site_once(3);
#endif
        /*
         * This is the stage machine's only input.  The word carries the
         * control phase in the low two bits of its status field:
         * 1 = data stage, 2 = status stage (same encoding the ZV-E10 toolkit
         * uses on this core).
         */
        TRACE_NRDY();
        switch (DEPEVT_STATUS_CONTROL_PHASE(DEPEVT_STATUS(event))) {
        case DEPEVT_STATUS_CONTROL_DATA:
#ifdef ENUM
            /* 第五十一轮（探针 ③）：数 DATA 阶段 NRDY，口径见 enum_cnt_nrdy。 */
            if (enum_cnt_nrdy < ENUM_COUNT_MAX)
                enum_cnt_nrdy++;
#endif
            if (!ep0_data || !ep0_data_len) {
                /* The host wants data we do not have. */
#ifdef ENUM
                /* 第三十六轮：只记账，不当场播报（理由同 REJECTED）。 */
                enum_rec(ENUM_LOG_NO_DATA);
#endif
                ep0_stall();
                break;
            }
            /*
             * 第五十七轮：这里是**唯一**的 DATA 武装点，而且每次 NRDY 都重装
             * （ZV-E10 / 7M4 两份能枚举的实现都是这样，没有任何守卫）。
             *
             * 第 55/56 轮把首条 TRB 挪到 SETUP 完成回调预装、再用
             * ep0_data_armed 把这个分支挡掉，等于跳过了参考实现唯一的武装点；
             * 两轮读数 0-0-1-0-5-9 逐字段相同，预装与换槽都已证伪。
             *
             * 第四十一轮：数据阶段装**事件点名的物理端点**（`ep`）。
             *
             * 第四十轮的第三段播报是 `2`，即 NRDY 落在物理端点 1；而那一轮
             * 把 TRB 全装到端点 0，读数就是 0 / 0，命令被接受但没有一条完成
             * —— 主机一直在端点 1 上等，端点 0 的 TRB 永远没人取。
             *
             * 本核的 `DEPEVT_EP()` 取位已由内核反汇编证实（§9.23.2：
             * `ubfx r3,r2,#1,#5`），所以直接用它，不再按请求方向推算。
             */
#ifdef ENUM
            {
                int rc = ep0_queue_data(ep, ep0_data, ep0_data_len);

                if (rc) {
                    /* ep0_queue_data() 内部已经记过 5（装了数据 TRB），
                     * 这里补 12：命令没被接受。第三十六轮起不当场播报。 */
                    enum_rec(ENUM_LOG_ARM_FAIL);
                    ep0_stall();
                } else if (enum_cnt_arm_data < ENUM_COUNT_MAX) {
                    /* 第五十七轮（探针 ④）：本处成功装出 DATA TRB 的次数。 */
                    enum_cnt_arm_data++;
                }
            }
#else
            if (ep0_queue_data(ep, ep0_data, ep0_data_len)) {
                TRACE_STALL();
                ep0_stall();
            }
#endif
            break;

        case DEPEVT_STATUS_CONTROL_STATUS:
            /*
             * 状态 TRB 的唯一武装点。
             *
             * 上一版在数据阶段完成时就提前装好了状态 TRB，又在这里用一个
             * 「已装过就跳过」的守卫把本事件挡掉（码 11）。结果就是第 30 轮
             * 的 `7 → 14 → 6 → 11 → 3`：提前装的那条既不完成也不报错，
             * 控制器真正要状态时又被挡掉，主机等到超时复位。守卫已删除。
             *
             * 第四十一轮：端点取 NRDY 事件点名的那个（`ep`），不再预先
             * 在解析 SETUP 时定死。状态阶段的方向与数据阶段相反，只有
             * 控制器自己知道这一笔该落在端点 0 还是端点 1。
             */
            if (ep0_queue_status(ep, ctrl.wLength ? 1 : 0)) {
                TRACE_STALL();
#ifdef ENUM
                enum_rec(ENUM_LOG_ARM_FAIL);
#endif
            }
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

static void handle_devt(u32 event)
{
#ifdef ENUM
    /* 站点 7：任何总线事件入口。它的作用是分辨"卡在处理 SETUP"与
     * "卡在处理主机复位风暴"：SETUP 之后若还来了 devt，这一组就会亮。 */
    enum_site_once(7);
#endif
    switch (DEVT_TYPE(event)) {
    case DWC3_DEVICE_EVENT_RESET:
        /* 第三十六轮：指纹 1 已撤（阻塞会盖住主机复位后的第一笔 SETUP）。 */
        on_reset();
        break;
    case DWC3_DEVICE_EVENT_CONNECT_DONE:
        /*
         * 2026-10-23 第十四轮：恢复 7M4 / ZV-E10 参考实现的做法 ——
         * CONNECT_DONE 时**完整重跑 ep0_init()**（DEPSTARTCFG + DEPCFG×2 +
         * DALEPENA + 装 setup TRB），而不是只重装 TRB。
         *
         * 此前这里只装 TRB，理由是"运行态发 DEPSTARTCFG 会把端点洗掉"。
         * 那条结论是在事件缓冲还没 4 KB 对齐的时候读出来的（§9.23.2 #4：
         * 那时事件时有时无、行为随机），"复位后再也收不到 SETUP"正是那种
         * 随机性的表现，不能作为删掉这一步的依据。
         *
         * 参考实现是这一族芯片上唯一被公开验证过的版本，它在 RESET 和
         * CONNECT_DONE 两处都重跑 ep0_init()，能正常枚举。
         */
#ifdef ENUM
        enum_rec(ENUM_LOG_CONNECT);
#endif
        (void)ep0_init();
        break;
    case DWC3_DEVICE_EVENT_DISCONNECT:
        configured = 0;
        break;
    default:
        break;
    }
}

int usb_gadget_init(void)
{
    return ep0_init();
}

void usb_event_pump(void)
{
    u32 event;
    int budget = 64;

    /* Drain: nothing consumes the 4 KB buffer while an endpoint command
     * waits. The budget stops a flood starving the caller. */
    while (budget-- > 0) {
        /*
         * 第三十六轮：泵里原来的 led_off()/led_on()（第四版仪器，用来分辨
         * "卡在读事件"与"卡在处理事件"）已撤。
         *
         * 它本身不阻塞，但会**抢占灯态**：枚举成功后主循环想用常亮作成功
         * 标志，泵每调一次就把灯灭掉，常亮根本立不住。这一轮灯的语义统一
         * 交给主循环与账本播报。
         */
        event = dwc3_event_poll();
        if (!event)
            return;

        if (DWC3_EVENT_IS_DEVT(event))
            handle_devt(event);
        else
            handle_depevt(event);
    }
}

int usb_is_configured(void)
{
    return configured;
}

/* Called from the main loop, outside any control transfer, so a failure here
 * cannot strand ep0 mid-transaction. */
void usb_bulk_enable_pending(void)
{
    if (!bulk_enable_pending)
        return;
    bulk_enable_pending = 0;
    bulk_endpoints_enable();
    if (usb_bulk_error)
        TRACE_STALL();
}

/* ---- bulk transfers ----------------------------------------------------- */

/* Set when an endpoint command during SET_CONFIGURATION was rejected, or when
 * a bulk transfer could not be started or never completed. */
int usb_bulk_error;

/* Resource index per direction, so a queued transfer can be ended later. */
static u32 bulk_rsc[2];

static int bulk_queue(u32 phys_ep, const void *buf, u32 len, int is_in)
{
    volatile struct dwc3_trb *trb = &bulk_trb[is_in ? 1 : 0];

    trb->bpl  = (u32)(unsigned long)buf;
    trb->bph  = 0;
    trb->size = len;
    trb->ctrl = DWC3_TRB_CTRL_TRBCTL(DWC3_TRBCTL_NORMAL)
              | DWC3_TRB_CTRL_HWO
              | DWC3_TRB_CTRL_LST
              | DWC3_TRB_CTRL_IOC
              | DWC3_TRB_CTRL_ISP_IMI;
    dma_wmb();
    return dwc3_ep_start_xfer(phys_ep, (u64)(unsigned long)trb,
                              &bulk_rsc[is_in ? 1 : 0]);
}

/*
 * Applied only to transfers the host has committed to. usb_bulk_recv() is not
 * one: waiting for a command is open-ended.
 *
 * Half a second is far past any real transfer -- a 64-byte reply lands in
 * microseconds, a full upload chunk in milliseconds -- and is slack for a host
 * stalling between reads of a long upload.
 */
#define BULK_TIMEOUT_TICKS  (TIMER0_HZ / 2)

/* Round 73: after the deadline, keep pumping for another 1.5 s before ending
 * the transfer. 0.5 + 1.5 s stays under the host's 5 s read timeout, so a late
 * completion can still be handed over normally instead of leaving a queued
 * transfer that would arrive one frame behind. */
#define BULK_LATE_TICKS     (TIMER0_HZ + TIMER0_HZ / 2)

u32 usb_bulk_recv(void *buf, u32 len)
{
    /* A short packet from the host ends the transfer early, and the residue
     * gives the count. */
    u32 xfer = (len + BULK_MAXPACKET - 1) & ~(u32)(BULK_MAXPACKET - 1);

    for (;;) {
        u32 epoch;

        while (!configured)
            usb_event_pump();

        /*
         * A bus reset during the previous wait cleared configured and
         * disabled the bulk endpoints. SET_CONFIGURATION only raises
         * bulk_enable_pending; nobody else runs it while we are blocked here,
         * so the endpoints would stay disabled and every queue would fail.
         */
        usb_bulk_enable_pending();

        epoch = bulk_epoch;
        bulk_out_residue = 0;
        bulk_out_busy = 1;

        if (bulk_queue(EP_OUT, buf, xfer, 0) != 0) {
            usb_bulk_error = USB_BULK_ERR_OUT_CMD;
            return 0;
        }

        /* No deadline: waiting for the host is the normal state. */
        while (bulk_out_busy && bulk_epoch == epoch)
            usb_event_pump();

        /* Reset mid-flight: the transfer is void. */
        if (bulk_epoch != epoch)
            continue;

        dsb();
        return xfer - bulk_out_residue;
    }
}

/*
 * A transfer left queued completes on the host's next read, and from then on
 * every answer is one behind.
 *
 * Round 82 (route B): ENDTRANSFER used to be fired and forgotten. Two things
 * are recorded now, because round 80 showed the session never recovers after a
 * bulk IN deadline (the retry failed on its very first chunk while `oem armed`
 * still read back the latch):
 *
 *   1. the command status, which dwc3_depcmd() already waits for;
 *   2. whether the controller actually gave the TRB back (HWO -> 0).
 *
 * Without (2) a wedged endpoint looks exactly like a clean abort, and the next
 * STARTTRANSFER is issued on an endpoint that cannot accept it.
 */
#define BULK_END_TICKS  (TIMER0_HZ / 50)     /* 20 ms: a release is immediate */
#define BULK_END_SETTLE (TIMER0_HZ / 500)    /* 2 ms: watch a release come back */

static int bulk_end_stuck;

/* Round 86: one read is not evidence. Round 85 ended the wait at 3882 ticks --
 * the ENDTRANSFER command's own duration -- with HWO=1 in the next instruction,
 * so the sample that ended the wait disagreed with its successor. DSB first and
 * require two consecutive zeros before calling the TRB released. */
static u32 bulk_in_hwo(void)
{
    dsb();
    return (bulk_trb[1].ctrl & DWC3_TRB_CTRL_HWO) ? 1u : 0u;
}

void usb_bulk_abort(void)
{
    if (bulk_in_busy) {
        u32 start = timer_ticks();
        u32 zeros = 0;
        u32 t_clear = 0;
        u32 t_rearm = 0;
        u32 s2;

        bulk_diag.end_st = (u32)dwc3_ep_end_xfer(EP_IN, bulk_rsc[1]);

        while (zeros < 2 && timer_ticks() - start < BULK_END_TICKS) {
            if (bulk_in_hwo()) {
                zeros = 0;
            } else if (++zeros == 2) {
                t_clear = timer_ticks() - start;
            }
            usb_event_pump();
        }

        /* Only a confirmed release can be re-armed; without one the endpoint is
         * simply stuck and the rebuild below is the only way forward. */
        if (t_clear) {
            s2 = timer_ticks();
            while (timer_ticks() - s2 < BULK_END_SETTLE) {
                if (bulk_in_hwo()) {
                    t_rearm = timer_ticks() - start;
                    break;
                }
                usb_event_pump();
            }
        }

        bulk_diag.end_clr  = t_clear;
        bulk_diag.end_re   = t_rearm;
        bulk_diag.end_wt   = timer_ticks() - start;
        bulk_diag.end_hwo  = bulk_in_hwo();
        bulk_end_stuck     = (bulk_diag.end_st != 0 || bulk_diag.end_hwo != 0);
        bulk_in_busy = 0;
    }
    if (bulk_out_busy) {
        dwc3_ep_end_xfer(EP_OUT, bulk_rsc[0]);
        bulk_out_busy = 0;
    }
}

/*
 * A host that stopped reading mid-reply costs one aborted transfer, not the
 * session. An endpoint command that was rejected is different: the endpoints
 * hold no usable resources, so the only way back is to rebuild them.
 *
 * Everything is recoverable. Leaving an error set used to pin the session
 * forever, because usb_bulk_send() short-circuits on it and the only accepted
 * value was IN_TIMEOUT.
 */
int usb_bulk_recover(void)
{
    int timeout;
    int stuck;

    if (!usb_bulk_error)
        return 0;

    timeout = (usb_bulk_error == USB_BULK_ERR_IN_TIMEOUT);

    usb_bulk_abort();
    stuck = bulk_end_stuck;      /* read before the next command overwrites it */
    usb_bulk_error = 0;

    /*
     * Round 82 (route B): a deadline whose TRB the controller never gave back
     * is not a queue-position problem. Round 80 proved the difference matters --
     * after such a deadline every later chunk of the same session failed on its
     * first block, while `oem armed` still answered. Only a rebuild of the
     * endpoints clears that, so do not take the cheap path here.
     */
    if (timeout && !stuck)
        return 0;

    if (!configured)
        return -1;

    bulk_enable_pending = 1;
    usb_bulk_enable_pending();
    return usb_bulk_error ? -1 : 0;
}

u32 usb_bulk_send(const void *buf, u32 len)
{
    /* Once the host has stopped listening, do not spend the deadline per
     * line of a multi-line reply. */
    if (usb_bulk_error)
        return 0;

    for (;;) {
        u32 epoch, start, evt0;

        while (!configured)
            usb_event_pump();

        /* Same reason as in usb_bulk_recv(): a reset leaves the bulk
         * endpoints disabled until someone acts on bulk_enable_pending. */
        usb_bulk_enable_pending();

        epoch = bulk_epoch;
        bulk_in_residue = 0;
        bulk_in_busy = 1;

        if (bulk_queue(EP_IN, buf, len, 1) != 0) {
            usb_bulk_error = USB_BULK_ERR_IN_CMD;
            return 0;
        }

        start = timer_ticks();
        evt0  = bulk_in_evt_seq;
        while (bulk_in_busy && bulk_epoch == epoch) {
            usb_event_pump();
            if (timer_ticks() - start > BULK_TIMEOUT_TICKS) {
                /* Round 73: snapshot before doing anything that destroys the
                 * evidence, then wait out the late window. */
                bulk_diag.hwo  = (bulk_trb[1].ctrl & DWC3_TRB_CTRL_HWO) ? 1u : 0u;
                bulk_diag.tsz  = bulk_trb[1].size & DWC3_TRB_SIZE_MASK;
                bulk_diag.cmd  = read32(DWC3_DEPCMD(EP_IN));
                bulk_diag.gev  = read32(DWC3_GEVNTCOUNT(0)) & DWC3_GEVNTCOUNT_MASK;
                bulk_diag.evt  = bulk_in_evt_seq - evt0;
                bulk_diag.typ  = bulk_in_last_type;
                bulk_diag.late = 0;

                while (bulk_in_busy && bulk_epoch == epoch &&
                       timer_ticks() - start <= BULK_TIMEOUT_TICKS + BULK_LATE_TICKS)
                    usb_event_pump();

                bulk_diag.evt = bulk_in_evt_seq - evt0;
                bulk_diag.typ = bulk_in_last_type;

                if (!bulk_in_busy && bulk_epoch == epoch) {
                    bulk_diag.late = timer_ticks() - start - BULK_TIMEOUT_TICKS;
                    return len - bulk_in_residue;
                }

                /* Nothing arrived: re-read the same fields so the report shows
                 * whether the late window changed the controller's state. */
                bulk_diag.hwo = (bulk_trb[1].ctrl & DWC3_TRB_CTRL_HWO) ? 1u : 0u;
                bulk_diag.tsz = bulk_trb[1].size & DWC3_TRB_SIZE_MASK;
                bulk_diag.cmd = read32(DWC3_DEPCMD(EP_IN));
                bulk_diag.gev = read32(DWC3_GEVNTCOUNT(0)) & DWC3_GEVNTCOUNT_MASK;

                usb_bulk_error = USB_BULK_ERR_IN_TIMEOUT;
                usb_bulk_abort();
                return 0;
            }
        }

        /* A reset between command and answer means the host is no longer
         * listening for it. Re-sending would desynchronise the
         * command/response lockstep. */
        if (bulk_epoch != epoch)
            return 0;

        return len - bulk_in_residue;
    }
}

u32 usb_bulk_epoch(void)
{
    return bulk_epoch;
}

const struct usb_bulk_diag *usb_bulk_diag(void)
{
    return &bulk_diag;
}

void usb_gadget_run(void)
{
#ifdef TRACE
    /*
     * 只回答一个问题：泵事件的时候 CPU 还活着吗。
     *
     *   心跳（亮/灭）一直在   泵在跑，内核逻辑没把我们困住
     *   心跳停在某个相位     卡在 usb_event_pump() 里面 —— 而且卡的位置
     *                        就发生在那一刻，不再有 "1e6 次之后进死循环"
     *                        这种会被误读成 "泵完了" 的第三种状态
     *
     * 旧版在 1e6 次空泵之后进入 NOP 死循环并继续闪灯，于是 "泵停了" 和
     * "泵空了" 都表现为灯还在闪，两种结论分不开。现在没有分叉：灯只要还
     * 在闪，泵就一直在跑。
     *
     * 全部 NOP 计时，路径上没有任何定时器（插线瞬间计数率会变 2~3 倍）。
     */
    for (;;) {
        usb_event_pump();

        led_on();
        trace_nop_delay();
        led_off();
        trace_nop_delay();
        trace_nop_delay();
        trace_nop_delay();
    }
#endif

#ifdef ENUM
    /*
     * 枚举期间零阻塞；判定"这段交互已经结束"才播报。
     *
     * 播报内容自第三十七轮起是三个计数（见 enum_dump()）。账本 enum_log
     * 仍然保留，但只用来判断"有没有新动静"，不再被播出。
     *
     * 静默判据用"空泵次数"而不是片内定时器：插线瞬间计数率会变 2~3 倍，
     * 次数不会。次数换算成秒依赖主频，但这里只需要"远超任何正常间隔"。
     *
     * 第四十轮：**只播一次，并且提前播**。
     *
     * 上一版要安静 3000 次泵才播，而 PC 侧每次 GET_DESCRIPTOR 等满 5 s
     * 超时 + 复位要重试约 9 次，整段两三分钟里我们一直在等"主机先试完"。
     * 现在两条触发条件取先到者：
     *   1) 武装尝试 ≥2 且静默 ≥400 次泵 —— 控制阶段已经推过，先报一份；
     *   2) 兜底 1200 次泵 —— 一次都没武装成功时也能报（数字会是 0）。
     * 播完置 dumped，绝不再播：后面主机的重试会刷新计数，但灯语只保留
     * 第一份，避免多段读数叠在一起。
     */
    {
        u32 quiet = 0;
        u32 beat = 0;
        u32 dumped = 0;

        /*
         * 丢掉 ep0_init() 阶段记下的那一条"装 SETUP TRB"：那是插线之前
         * 我们自己装的，和主机侧的时间线无关，留着会让读数凭空多出个头。
         */
        enum_log_n = 0;
        /* 第四十八轮：connect 已完成，先锁一次速度，再进泵循环持续观察。 */
        enum_sample_speed();

        while (!configured) {
            u8 before = enum_log_n;

            usb_event_pump();
            enum_sample_speed();
            enum_nop(ENUM_DARK_NOP);

            if (enum_log_n != before) {
                quiet = 0;
            } else {
                quiet++;
            }

            /*
             * 第三十七轮：心跳只在"没有待播内容"时才打。
             *
             * 播报前要先安静若干次泵，心跳原本会在这段时间插进来好几次，
             * 和真正的读数混成一片 —— 这是"灯语太长、间隔不明显"的一部分。
             * 现在只要账本里有东西（待播），一律不闪；播完账本清零，心跳恢复，
             * 这时它只表示"还活着、没有新动静"。
             */
            if (++beat >= ENUM_BEAT_EVERY) {
                beat = 0;
                if (quiet >= ENUM_BEAT_AFTER && enum_log_n == 0u) {
                    led_on();
                    enum_nop(ENUM_BLIP_NOP);
                    led_off();
                }
            }

            if (!dumped && enum_log_n &&
                ((enum_cnt_arm >= 2u && quiet >= ENUM_QUIET_EARLY) ||
                 quiet >= ENUM_QUIET_LIMIT)) {
                quiet = 0;
                dumped = 1;
                enum_dump();
            }
        }

        /*
         * 第三十六轮：**常亮 = 枚举成功**。
         *
         * 走到这里说明主机发过 SET_CONFIGURATION(1)，`configured` 被置位，
         * 已经进入 fastboot 通道。此后不再有任何灯语：常亮就是判据，不用
         * 再靠数闪。泵内部也不再动灯（见 usb_event_pump）。
         */
        led_on();
    }
#else
    while (!configured)
        usb_event_pump();
#endif

    fastboot_loop();
}
