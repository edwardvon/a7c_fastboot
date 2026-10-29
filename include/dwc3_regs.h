#ifndef A7C_DWC3_REGS_H
#define A7C_DWC3_REGS_H

#include "io.h"

/*
 * Synopsys DWC3 register map, A7C base address.
 *
 * Cross-checked against the stock kernel: it keeps `regs = 0xF020C100`, which
 * is DWC3_BASE + 0xC100, and reaches DCFG/DCTL/DEVTEN/DSTS at regs+0x600/
 * +0x604/+0x608/+0x60C -- identical to the ILCE-7M4 layout, only the base
 * differs (7M4 uses 0xF10C0000).
 */
#define DWC3_BASE               0xF0200000u

#define DWC3_GLOBALS_START      0xC100u
#define DWC3_DEVICE_START       0xC700u
#define DWC3_DEP_START          0xC800u

#define DWC3_GSBUSCFG0          (DWC3_BASE + 0xC100u)
#define DWC3_GCTL               (DWC3_BASE + 0xC110u)
#define DWC3_GSNPSID            (DWC3_BASE + 0xC120u)
#define DWC3_GUSB2PHYCFG(n)     (DWC3_BASE + 0xC200u + ((n) * 0x04u))

/*
 * TxFIFO/RxFIFO sizing.  Each TxFIFO is {TXFIFODEPTH[31:16], TXFIFOSZ[7:0]}:
 * the depth field is the depth in 32-bit words and is what a transfer has to
 * fit into.  Ep0's entry starts at the bottom of the Tx RAM, so nothing else
 * claims space before it.
 */
#define DWC3_GTXFIFOSIZ(n)      (DWC3_BASE + 0xC300u + ((n) * 0x04u))
#define DWC3_GRXFIFOSIZ(n)      (DWC3_BASE + 0xC380u + ((n) * 0x04u))
#define DWC3_FIFOSIZ_DEPTH(x)   (((x) >> 16) & 0xFFFFu)
#define DWC3_FIFOSIZ_CODE(x)    ((x) & 0xFFu)

#define DWC3_GEVNTADRLO(n)      (DWC3_BASE + 0xC400u + ((n) * 0x10u))
#define DWC3_GEVNTADRHI(n)      (DWC3_BASE + 0xC404u + ((n) * 0x10u))
#define DWC3_GEVNTSIZ(n)        (DWC3_BASE + 0xC408u + ((n) * 0x10u))
#define DWC3_GEVNTCOUNT(n)      (DWC3_BASE + 0xC40Cu + ((n) * 0x10u))

#define DWC3_DCFG               (DWC3_BASE + 0xC700u)
#define DWC3_DCTL               (DWC3_BASE + 0xC704u)
#define DWC3_DEVTEN             (DWC3_BASE + 0xC708u)
#define DWC3_DSTS               (DWC3_BASE + 0xC70Cu)
#define DWC3_DGCMDPAR           (DWC3_BASE + 0xC710u)
#define DWC3_DGCMD              (DWC3_BASE + 0xC714u)
#define DWC3_DALEPENA           (DWC3_BASE + 0xC720u)

/*
 * The parameter registers count *down* from the command register:
 * PAR2 at +0x00, PAR1 at +0x04, PAR0 at +0x08, DEPCMD at +0x0C.
 * This is what the A7C driver uses (notes section 9.6.1: "DEPCMDPAR2/1/0 =
 * 0x700/0x704/0x708 + 0x10n", i.e. 0xF020C800/804/808) and it is also the
 * generic DWC3 layout. Getting this backwards silently corrupts every
 * SETEPCONFIG -- the command still returns status 0.
 */
#define DWC3_DEP_BASE(n)        (DWC3_BASE + 0xC800u + ((n) * 0x10u))
#define DWC3_DEPCMDPAR0(n)      (DWC3_DEP_BASE(n) + 0x08u)
#define DWC3_DEPCMDPAR1(n)      (DWC3_DEP_BASE(n) + 0x04u)
#define DWC3_DEPCMDPAR2(n)      (DWC3_DEP_BASE(n) + 0x00u)
#define DWC3_DEPCMD(n)          (DWC3_DEP_BASE(n) + 0x0Cu)

/* ---- GCTL --------------------------------------------------------------- */

#define DWC3_GCTL_PRTCAP(n)         ((n) << 12)
#define DWC3_GCTL_PRTCAP_MASK       (3u << 12)
#define DWC3_GCTL_PRTCAP_HOST       1u
#define DWC3_GCTL_PRTCAP_DEVICE     2u
#define DWC3_GCTL_CORESOFTRESET     BIT(11)
/* A7C only ever touches bit 3..5 here; there is deliberately no PWRDNSCALE
 * macro, because the A7C driver never writes that field. */
#define DWC3_GCTL_SCALEDOWN_MASK    (3u << 4)
#define DWC3_GCTL_DISSCRAMBLE       BIT(3)
/* 1 = 关掉 core 的时钟门控。默认 0（门控使能），空闲时 core 会关内部时钟。
 * 参考实现（7M4 / ZV-E10）是**清**这一位，即保持门控使能；裸机不要自作主张置 1。 */
#define DWC3_GCTL_DSBLCLKGTNG       BIT(0)
/* Sony 专用：suspend 时钟的 power-down scale。参考实现写 2。 */
#define DWC3_GCTL_PWRDNSCALE(n)     ((n) << 19)
#define DWC3_GCTL_PWRDNSCALE_MASK   DWC3_GCTL_PWRDNSCALE(0x1fffu)

/* ---- GUSB2PHYCFG -------------------------------------------------------- */

#define DWC3_GUSB2PHYCFG_SUSPHY         BIT(6)
#define DWC3_GUSB2PHYCFG_ULPI_UTMI      BIT(4)
#define DWC3_GUSB2PHYCFG_PHYIF(n)       ((n) << 3)
#define DWC3_GUSB2PHYCFG_PHYIF_MASK     DWC3_GUSB2PHYCFG_PHYIF(1)
#define DWC3_GUSB2PHYCFG_ENBLSLPM       BIT(8)
#define DWC3_GUSB2PHYCFG_USBTRDTIM(n)   ((n) << 10)
#define DWC3_GUSB2PHYCFG_USBTRDTIM_MASK DWC3_GUSB2PHYCFG_USBTRDTIM(0xFu)

#define UTMI_PHYIF_8_BIT        0u
#define USBTRDTIM_UTMI_8_BIT    9u

/* ---- DCFG / DCTL / DSTS / DEVTEN ---------------------------------------- */

#define DWC3_DCFG_DEVADDR(a)        ((a) << 3)
#define DWC3_DCFG_DEVADDR_MASK      DWC3_DCFG_DEVADDR(0x7Fu)
#define DWC3_DCFG_SPEED_MASK        7u
#define DWC3_DCFG_HIGHSPEED         0u
#define DWC3_DCFG_FULLSPEED         1u

#define DWC3_DCTL_RUN_STOP          BIT(31)
#define DWC3_DCTL_CSFTRST           BIT(30)
#define DWC3_DCTL_KEEP_CONNECT      BIT(19)

#define DWC3_DSTS_DEVCTRLHLT        BIT(22)
#define DWC3_DSTS_CONNECTSPD_MASK   (7u << 0)

#define DWC3_DEVTEN_DISCONNECT      BIT(0)
#define DWC3_DEVTEN_RESET           BIT(1)
#define DWC3_DEVTEN_CONNECT_DONE    BIT(2)

#define DWC3_GEVNTCOUNT_MASK        0xFFFCu
#define DWC3_GEVNTSIZ_INTMASK       BIT(31)
#define DWC3_GEVNTSIZ_SIZE(n)       ((n) & 0xFFFFu)

#define DWC3_EVENT_BUFFERS_SIZE     4096u

/* ---- DEPCMD ------------------------------------------------------------- */

#define DWC3_DEPCMD_CMDACT          BIT(10)
#define DWC3_DEPCMD_CMDIOC          BIT(8)
#define DWC3_DEPCMD_HIPRI_FORCERM   BIT(11)
#define DWC3_DEPCMD_PARAM(x)        ((x) << 16)
/*
 * DEPCMD layout (DWC_usb3 2.90a databook):
 *   [3:0]   CMD_TYPE      -- also the readback of the command just issued
 *   [8]     CMDIOC
 *   [10]    CMDACT
 *   [11]    HIPRI_FORCERM
 *   [15]    CMDSTS        -- 0 = command completed, 1 = command failed
 *   [31:16] command parameter
 *
 * The 4-bit [15:12] CMD_STATUS field is the DWC3 3.x layout and does not
 * apply to this core; the ZV-E10 toolkit reads bit 15 alone on the same
 * silicon. Reading the status from [3:0] returns the command code itself, so
 * every command looks like a failure.
 */
#define DWC3_DEPCMD_STATUS(x)       (((x) >> 15) & 0x1u)
#define DWC3_DEPCMD_GET_RSC_IDX(x)  (((x) >> 16) & 0x7Fu)

#define DWC3_DEPCMD_DEPSTARTCFG         0x09u
#define DWC3_DEPCMD_ENDTRANSFER         0x08u
#define DWC3_DEPCMD_UPDATETRANSFER      0x07u
#define DWC3_DEPCMD_STARTTRANSFER       0x06u
#define DWC3_DEPCMD_CLEARSTALL          0x05u
#define DWC3_DEPCMD_SETSTALL            0x04u
#define DWC3_DEPCMD_SETTRANSFRESOURCE   0x02u
#define DWC3_DEPCMD_SETEPCONFIG         0x01u

#define DWC3_DEPCMD_TYPE_CONTROL    0u
#define DWC3_DEPCMD_TYPE_ISOC       1u
#define DWC3_DEPCMD_TYPE_BULK       2u
#define DWC3_DEPCMD_TYPE_INTR       3u

/* DEPCFG parameter 0 */
#define DWC3_DEPCFG_EP_TYPE(n)          (((n) & 3u) << 1)
#define DWC3_DEPCFG_MAX_PACKET_SIZE(n)  (((n) & 0x7FFu) << 3)
#define DWC3_DEPCFG_FIFO_NUMBER(n)      (((n) & 0x1Fu) << 17)
#define DWC3_DEPCFG_BURST_SIZE(n)       (((n) & 0xFu) << 22)
#define DWC3_DEPCFG_ACTION_INIT         (0u << 30)
#define DWC3_DEPCFG_ACTION_RESTORE      (1u << 30)

/* DEPCFG parameter 1 */
#define DWC3_DEPCFG_INT_NUM(n)              ((n) & 0x1Fu)
#define DWC3_DEPCFG_XFER_COMPLETE_EN        BIT(8)
#define DWC3_DEPCFG_XFER_IN_PROGRESS_EN     BIT(9)
#define DWC3_DEPCFG_XFER_NOT_READY_EN       BIT(10)
#define DWC3_DEPCFG_EP_NUMBER(n)            (((n) & 0x1Fu) << 25)

/* ---- TRB ---------------------------------------------------------------- */

struct dwc3_trb {
    u32 bpl;
    u32 bph;
    u32 size;
    u32 ctrl;
} __attribute__((packed));

#define DWC3_TRB_SIZE_MASK          0x00FFFFFFu

#define DWC3_TRB_CTRL_HWO           BIT(0)
#define DWC3_TRB_CTRL_LST           BIT(1)
#define DWC3_TRB_CTRL_CHN           BIT(2)
#define DWC3_TRB_CTRL_CSP           BIT(3)
#define DWC3_TRB_CTRL_TRBCTL(n)     (((n) & 0x3Fu) << 4)
#define DWC3_TRB_CTRL_ISP_IMI       BIT(10)
#define DWC3_TRB_CTRL_IOC           BIT(11)

#define DWC3_TRBCTL_NORMAL          1u
#define DWC3_TRBCTL_CONTROL_SETUP   2u
#define DWC3_TRBCTL_CONTROL_STATUS2 3u
#define DWC3_TRBCTL_CONTROL_STATUS3 4u
#define DWC3_TRBCTL_CONTROL_DATA    5u
#define DWC3_TRBCTL_ISOCHRONOUS     6u
#define DWC3_TRBCTL_LINK_TRB        8u

/* ---- events ------------------------------------------------------------- */

/* Physical endpoint number: USB endpoint number << 1, direction in bit 0. */
#define DWC3_PHYS_EP(num, is_in)    (((num) << 1) | ((is_in) ? 1u : 0u))

#define DWC3_EVENT_IS_DEVT(e)       ((e) & 1u)
/* In the 32-bit event format an event word has a type field in bits[7:1] and a
 * subtype in bits[11:8]. For a device event the type field is always zero and
 * the event code lives in the subtype field; the stock kernel proves this at
 * 0x602b5830 (`tst r2,#0xfe` -> bail, then `ubfx r2,r2,#8,#4` -> switch). */
#define DEVT_TYPE(e)                (((e) >> 8) & 0xFu)
#define DEPEVT_EP(e)                (((e) >> 1) & 0x1Fu)
#define DEPEVT_TYPE(e)              (((e) >> 6) & 0xFu)
#define DEPEVT_STATUS(e)            (((e) >> 12) & 0xFu)

/*
 * Control phase inside a XferNotReady status word on ep0. The low two bits of
 * the status field carry it: 1 = data stage wants a buffer, 2 = status stage.
 * Taken from the ZV-E10 toolkit, which runs the same core (DWC_usb3 2.90a).
 */
#define DEPEVT_STATUS_CONTROL_DATA      1u
#define DEPEVT_STATUS_CONTROL_STATUS    2u
#define DEPEVT_STATUS_CONTROL_PHASE(n)  ((n) & 3u)

#define DWC3_DEVICE_EVENT_DISCONNECT     0u
#define DWC3_DEVICE_EVENT_RESET          1u
#define DWC3_DEVICE_EVENT_CONNECT_DONE   2u
#define DWC3_DEVICE_EVENT_LINK_STATUS    3u

#define DWC3_DEPEVT_XFERCOMPLETE    1u
#define DWC3_DEPEVT_XFERINPROGRESS  2u
#define DWC3_DEPEVT_XFERNOTREADY    3u
#define DWC3_DEPEVT_EPCMDCOMPLETE   7u

#endif /* A7C_DWC3_REGS_H */
