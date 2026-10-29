#include "storage.h"
#include "io.h"
#include "timer.h"

/*
 * Ported from zv-e10-toolkit/fastboot/stage2/src/sdhci.c (same SoC family,
 * same Arasan SDHCI). What changed:
 *
 *   - the write half is compiled out: CMD24 is defined only inside
 *     `#if STORAGE_ALLOW_WRITE`, which only build.ps1 -Write sets, so the
 *     default image has no code path that can program the card, by
 *     construction rather than by a runtime flag;
 *   - a multi-sector read is one CMD18 with auto-CMD12 (P13), so the card's
 *     per-block access latency is paid once per chunk instead of once per
 *     sector; a CMD18 failure is stopped and the range is re-read with the
 *     CMD17 loop, which stays the known-good path;
 *   - identification runs at 400 kHz as the spec requires; the data phase
 *     then runs at 24 MHz (r97). Only the host divider moves -- the card is
 *     idle in TRAN, 24 MHz is inside its default-speed range, and no card
 *     register is written, so no mode switch is needed;
 *   - the result goes into a struct the fastboot layer can print, instead of
 *     a fixed SRAM address;
 *   - the clock is explicitly stopped before the divider is changed. The A7C
 *     hands over with CLOCK_CONTROL = 0x0007 (SD_CLK_EN = 1, divider 0, i.e.
 *     192 MHz), so "the clock is off at this point" is NOT true here even
 *     though it may have been on the ZV-E10.
 *
 * Base 0xF0304000 and every offset below were confirmed on the A7C in
 * round 62 (2026-10-25): CAPABILITIES 0x64EEC000, PRESENT_STATE 0x01FF0000,
 * HOST_VERSION 0x0102 -- bit-for-bit identical to the ZV-E10 readings.
 */

#define SDHCI_BASE              0xF0304000u

#define SDHCI_BLOCK_SIZE        (SDHCI_BASE + 0x04)   /* 16-bit */
#define SDHCI_BLOCK_COUNT       (SDHCI_BASE + 0x06)   /* 16-bit */
#define SDHCI_ARGUMENT          (SDHCI_BASE + 0x08)
#define SDHCI_TRANSFER_MODE     (SDHCI_BASE + 0x0C)   /* 16-bit */
#define SDHCI_COMMAND           (SDHCI_BASE + 0x0E)
#define SDHCI_RESPONSE          (SDHCI_BASE + 0x10)
#define SDHCI_BUFFER            (SDHCI_BASE + 0x20)
#define SDHCI_PRESENT_STATE     (SDHCI_BASE + 0x24)
#define SDHCI_HOST_CONTROL      (SDHCI_BASE + 0x28)   /* 8-bit  */
#define SDHCI_CLOCK_CONTROL     (SDHCI_BASE + 0x2C)   /* 16-bit */
#define SDHCI_SOFTWARE_RESET    (SDHCI_BASE + 0x2F)   /* 8-bit  */
#define SDHCI_INT_STATUS        (SDHCI_BASE + 0x30)   /* 16-bit */
#define SDHCI_ERR_INT_STATUS    (SDHCI_BASE + 0x32)   /* 16-bit */
#define SDHCI_INT_ENABLE        (SDHCI_BASE + 0x34)   /* 16-bit */
#define SDHCI_ERR_INT_ENABLE    (SDHCI_BASE + 0x36)   /* 16-bit */
#define SDHCI_CAPABILITIES      (SDHCI_BASE + 0x40)
#define SDHCI_HOST_VERSION      (SDHCI_BASE + 0xFE)   /* 16-bit */

#define SDHCI_CLK_INT_EN        BIT(0)
#define SDHCI_CLK_INT_STABLE    BIT(1)
#define SDHCI_CLK_SD_EN         BIT(2)
#define SDHCI_CLK_DIV_SHIFT     8
#define SDHCI_CLK_DIV_HI_SHIFT  6

#define SDHCI_RESET_CMD         BIT(1)
#define SDHCI_RESET_DATA        BIT(2)

#define SDHCI_CMD_INHIBIT       BIT(0)
#define SDHCI_DATA_INHIBIT      BIT(1)

#define SDHCI_INT_CMD_COMPLETE  BIT(0)
#define SDHCI_INT_XFER_COMPLETE BIT(1)
#define SDHCI_INT_BUF_WRITE_RDY BIT(4)   /* write builds only, see below */
#define SDHCI_INT_BUF_READ_RDY  BIT(5)
#define SDHCI_INT_ERROR         BIT(15)

#define SDHCI_TRNS_BLK_CNT_EN   BIT(1)
#define SDHCI_TRNS_AUTO_CMD12   BIT(2)
#define SDHCI_TRNS_READ         BIT(4)
#define SDHCI_TRNS_MULTI        BIT(5)

#define SDHCI_CMD_RESP_NONE     0x00
#define SDHCI_CMD_RESP_LONG     0x01
#define SDHCI_CMD_RESP_SHORT    0x02
#define SDHCI_CMD_RESP_BUSY     0x03
#define SDHCI_CMD_CRC_CHECK     BIT(3)
#define SDHCI_CMD_INDEX_CHECK   BIT(4)
#define SDHCI_CMD_DATA_PRESENT  BIT(5)
#define SDHCI_MAKE_CMD(idx, f)  (u16)(((idx) << 8) | (f))

#define MMC_CMD_GO_IDLE_STATE       0
#define MMC_CMD_SEND_OP_COND        1
#define MMC_CMD_ALL_SEND_CID        2
#define MMC_CMD_SET_RELATIVE_ADDR   3
#define MMC_CMD_SELECT_CARD         7
#define MMC_CMD_SEND_CSD            9
#define MMC_CMD_SEND_STATUS         13
/* Read only. 24 (WRITE_BLOCK) is defined further down, inside the
 * STORAGE_ALLOW_WRITE guard: adding it to a default build is the change that
 * must not be possible by accident. */
#define MMC_CMD_READ_SINGLE_BLOCK   17
#define MMC_CMD_STOP_TRANSMISSION   12
#define MMC_CMD_READ_MULTIPLE_BLOCK 18

#if STORAGE_ALLOW_WRITE
#define MMC_CMD_WRITE_SINGLE_BLOCK  24
#endif

#define MMC_R1_READY_FOR_DATA   BIT(8)
#define MMC_R1_STATE(r)         (((r) >> 9) & 0xf)
#define MMC_STATE_TRAN          4

/* OCR argument: bit 30 = sector addressing, 0xFF8000 = 2.7-3.6 V.
 * Same value the ZV-E10 stage-2 used successfully. */
#define MMC_OCR_ARG             0x40FF8080u

#define SECTOR_SIZE             512

/* Data-phase dividers tried once per session, fastest first (see
 * sdhci_pick_div()). The eMMC default-speed range ends at 26 MHz, so every
 * candidate is reachable with the host divider alone -- no CMD6 mode switch. */

#define CMD_TIMEOUT_TICKS       (TIMER0_HZ / 2)     /* 500 ms */
#define BUSY_TIMEOUT_TICKS      (TIMER0_HZ)         /* 1 s    */
#define IDENT_TIMEOUT_TICKS     (TIMER0_HZ)         /* 1 s for CMD1 polling */

static void struct_clear(void *p, u32 size)
{
    u32 i;

    /* Word-wise on purpose: every member is 16- or 32-bit and the sizes are
     * multiples of 4, so padding is cleared too. */
    for (i = 0; i < size / sizeof(u32); i++)
        ((volatile u32 *)p)[i] = 0;
}

static void sdhci_reset(u8 mask)
{
    u32 start = timer_ticks();

    write8(SDHCI_SOFTWARE_RESET, mask);
    while (read8(SDHCI_SOFTWARE_RESET) & mask) {
        if (timer_ticks() - start > CMD_TIMEOUT_TICKS)
            return;
    }
}

static void sdhci_recover(void)
{
    sdhci_reset(SDHCI_RESET_CMD | SDHCI_RESET_DATA);
    write16(SDHCI_INT_STATUS, read16(SDHCI_INT_STATUS));
    write16(SDHCI_ERR_INT_STATUS, read16(SDHCI_ERR_INT_STATUS));
}

static int wait_inhibit(u32 mask)
{
    u32 start = timer_ticks();

    while (read32(SDHCI_PRESENT_STATE) & mask) {
        if (timer_ticks() - start > CMD_TIMEOUT_TICKS)
            return STORAGE_ERR_INHIBIT;
    }
    return STORAGE_OK;
}

static int wait_int(u16 mask, u32 timeout, int err)
{
    u32 start = timer_ticks();
    u16 sts;

    for (;;) {
        sts = read16(SDHCI_INT_STATUS);

        if (sts & SDHCI_INT_ERROR) {
            write16(SDHCI_ERR_INT_STATUS, read16(SDHCI_ERR_INT_STATUS));
            write16(SDHCI_INT_STATUS, sts);
            sdhci_recover();
            return STORAGE_ERR_CMD;
        }
        if (sts & mask) {
            write16(SDHCI_INT_STATUS, mask);
            return STORAGE_OK;
        }
        if (timer_ticks() - start > timeout) {
            sdhci_recover();
            return err;
        }
    }
}

static int send_cmd(u8 index, u32 arg, u16 flags)
{
    int ret;

    ret = wait_inhibit(SDHCI_CMD_INHIBIT);
    if (ret) {
        sdhci_recover();
        ret = wait_inhibit(SDHCI_CMD_INHIBIT);
        if (ret)
            return ret;
    }

    write16(SDHCI_INT_STATUS, read16(SDHCI_INT_STATUS));
    write16(SDHCI_ERR_INT_STATUS, read16(SDHCI_ERR_INT_STATUS));

    write32(SDHCI_ARGUMENT, arg);
    dsb();
    write16(SDHCI_COMMAND, SDHCI_MAKE_CMD(index, flags));
    dsb();

    return wait_int(SDHCI_INT_CMD_COMPLETE, CMD_TIMEOUT_TICKS,
                    STORAGE_ERR_TIMEOUT);
}

/* CMD7 is R1b: DAT0 stays low until the card has switched state. */
static int mmc_cmd(u8 index, u32 arg, u16 flags, u32 *resp)
{
    int ret = send_cmd(index, arg, flags);

    if (resp)
        *resp = read32(SDHCI_RESPONSE);

    if (ret == STORAGE_OK && (flags & 0x3) == SDHCI_CMD_RESP_BUSY)
        ret = wait_int(SDHCI_INT_XFER_COMPLETE, BUSY_TIMEOUT_TICKS,
                       STORAGE_ERR_BUSY);
    return ret;
}

/* div is the SDHCI divider N: card clock = base / (2*N). */
static void sdhci_set_clock(u32 div)
{
    u32 start;
    u16 clk;

    /* Stop the card clock first: at hand-over it is running at 192 MHz
     * (divider 0), and changing the divider underneath it is not something
     * this controller documents. */
    clk = read16(SDHCI_CLOCK_CONTROL);
    write16(SDHCI_CLOCK_CONTROL, clk & (u16)~SDHCI_CLK_SD_EN);

    clk = (u16)(((div & 0xff) << SDHCI_CLK_DIV_SHIFT) |
                (((div >> 8) & 0x3) << SDHCI_CLK_DIV_HI_SHIFT) |
                SDHCI_CLK_INT_EN);
    write16(SDHCI_CLOCK_CONTROL, clk);

    start = timer_ticks();
    while (!(read16(SDHCI_CLOCK_CONTROL) & SDHCI_CLK_INT_STABLE)) {
        if (timer_ticks() - start > CMD_TIMEOUT_TICKS)
            break;
    }

    write16(SDHCI_CLOCK_CONTROL, (u16)(clk | SDHCI_CLK_SD_EN));
    udelay(100);
}

/* Snapshot the boot chain's state, then put the host into a state where the
 * identification sequence can run at 400 kHz. */
static void sdhci_begin(struct storage_id *out)
{
    u32 base_khz, div;

    out->clock_before  = read16(SDHCI_CLOCK_CONTROL);
    out->host_before   = read8(SDHCI_HOST_CONTROL);
    out->caps          = read32(SDHCI_CAPABILITIES);
    out->host_version  = read16(SDHCI_HOST_VERSION);
    out->pstate_before = read32(SDHCI_PRESENT_STATE);

    /* Identification must run at 400 kHz or less. CAP bits 15:8 hold a base
     * clock in MHz (0xC0 = 192 MHz on this part), but the divided clock really
     * runs off 48 MHz (P12), so this div lands at ~100 kHz -- still legal. */
    base_khz = ((out->caps >> 8) & 0xff) * 1000;
    if (base_khz == 0)
        base_khz = 192000;
    div = base_khz / (2 * 400);
    if (div == 0)
        div = 1;

    /* The controller gates the status bits themselves, so polling needs the
     * enables set. Interrupts stay masked at the CPU (cpsid i). */
    write16(SDHCI_INT_ENABLE, SDHCI_INT_CMD_COMPLETE |
                              SDHCI_INT_XFER_COMPLETE |
                              SDHCI_INT_BUF_READ_RDY
#if STORAGE_ALLOW_WRITE
                              | SDHCI_INT_BUF_WRITE_RDY
#endif
                              );
    write16(SDHCI_ERR_INT_ENABLE, 0xffff);

    sdhci_reset(SDHCI_RESET_CMD | SDHCI_RESET_DATA);
    write8(SDHCI_HOST_CONTROL, (u8)(out->host_before & ~0x22));  /* 1-bit */
    sdhci_set_clock(div);
}

/* Restore whatever the boot chain left in HOST_CONTROL / CLOCK_CONTROL. The
 * controller is re-initialised from scratch by the stock firmware, so this is
 * about not leaving a half-configured clock behind for the next reader. */
static void sdhci_end(struct storage_id *out)
{
    out->pstate_after = read32(SDHCI_PRESENT_STATE);

    write8(SDHCI_HOST_CONTROL, out->host_before);
    write16(SDHCI_CLOCK_CONTROL, out->clock_before);
    udelay(100);
}

/* CMD0/1/2/3/9/7/13: idle -> ready -> ident -> stby -> tran. */
static int mmc_ident_seq(struct storage_id *out)
{
    u16 r1flags = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC_CHECK |
                  SDHCI_CMD_INDEX_CHECK;
    u32 start, resp = 0;
    int ret;

    out->step = 1;
    ret = send_cmd(MMC_CMD_GO_IDLE_STATE, 0, SDHCI_CMD_RESP_NONE);
    if (ret)
        return ret;
    mdelay(2);

    out->step = 2;
    start = timer_ticks();
    for (;;) {
        ret = mmc_cmd(MMC_CMD_SEND_OP_COND, MMC_OCR_ARG,
                      SDHCI_CMD_RESP_SHORT, &resp);
        if (ret)
            return ret;
        out->ocr = resp;
        if (resp & 0x80000000u)
            break;
        if (timer_ticks() - start > IDENT_TIMEOUT_TICKS)
            return STORAGE_ERR_TIMEOUT;
        mdelay(5);
    }

    out->step = 3;
    ret = mmc_cmd(MMC_CMD_ALL_SEND_CID, 0,
                  SDHCI_CMD_RESP_LONG | SDHCI_CMD_CRC_CHECK, 0);
    if (ret)
        return ret;
    out->cid[0] = read32(SDHCI_RESPONSE + 0x0);
    out->cid[1] = read32(SDHCI_RESPONSE + 0x4);
    out->cid[2] = read32(SDHCI_RESPONSE + 0x8);
    out->cid[3] = read32(SDHCI_RESPONSE + 0xC);

    out->step = 4;
    out->rca = 1;
    ret = mmc_cmd(MMC_CMD_SET_RELATIVE_ADDR, out->rca << 16, r1flags, 0);
    if (ret)
        return ret;

    out->step = 5;
    ret = mmc_cmd(MMC_CMD_SEND_CSD, out->rca << 16,
                  SDHCI_CMD_RESP_LONG | SDHCI_CMD_CRC_CHECK, 0);
    if (ret)
        return ret;
    out->csd[0] = read32(SDHCI_RESPONSE + 0x0);
    out->csd[1] = read32(SDHCI_RESPONSE + 0x4);
    out->csd[2] = read32(SDHCI_RESPONSE + 0x8);
    out->csd[3] = read32(SDHCI_RESPONSE + 0xC);

    out->step = 6;
    ret = mmc_cmd(MMC_CMD_SELECT_CARD, out->rca << 16,
                  SDHCI_CMD_RESP_BUSY | SDHCI_CMD_CRC_CHECK |
                  SDHCI_CMD_INDEX_CHECK, 0);
    if (ret)
        return ret;

    out->step = 7;
    ret = mmc_cmd(MMC_CMD_SEND_STATUS, out->rca << 16, r1flags, &resp);
    out->r1 = resp;
    if (ret)
        return ret;

    if (MMC_R1_STATE(resp) != MMC_STATE_TRAN ||
        !(resp & MMC_R1_READY_FOR_DATA))
        return STORAGE_ERR_STATE;
    return STORAGE_OK;
}

/*
 * One 512-byte sector, PIO, no DMA. `lba` is a block address: OCR bit 30 was
 * set in CMD1, so the card counts in 512-byte units and no byte offset ever
 * reaches the argument register.
 */
static int read_block(u32 lba, u32 *dst)
{
    u32 w;
    int ret;

    ret = wait_inhibit(SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT);
    if (ret)
        return ret;

    write16(SDHCI_BLOCK_SIZE, SECTOR_SIZE);
    write16(SDHCI_BLOCK_COUNT, 1);
    write16(SDHCI_TRANSFER_MODE, SDHCI_TRNS_READ | SDHCI_TRNS_BLK_CNT_EN);
    dsb();

    ret = send_cmd(MMC_CMD_READ_SINGLE_BLOCK, lba,
                   SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC_CHECK |
                   SDHCI_CMD_INDEX_CHECK | SDHCI_CMD_DATA_PRESENT);
    if (ret)
        return ret;

    ret = wait_int(SDHCI_INT_BUF_READ_RDY, CMD_TIMEOUT_TICKS, STORAGE_ERR_DATA);
    if (ret)
        return ret;

    for (w = 0; w < SECTOR_SIZE / 4; w++)
        dst[w] = read32(SDHCI_BUFFER);

    return wait_int(SDHCI_INT_XFER_COMPLETE, CMD_TIMEOUT_TICKS,
                    STORAGE_ERR_DATA);
}

#if STORAGE_ALLOW_WRITE
/*
 * M-7: one 512-byte sector, PIO, no DMA -- the mirror image of read_block().
 *
 * SDHCI_TRNS_READ stays clear, because that bit is the direction select.
 * BUF_WRITE_RDY is the controller saying the card has opened its data phase,
 * so the FIFO is filled only after it; filling first would overrun.
 *
 * XFER_COMPLETE means the host has handed every word over, not that the card
 * has finished programming it.  mmc_wait_ready() below holds the caller until
 * CMD13 reports READY_FOR_DATA in TRAN; without that a read-back issued right
 * afterwards would race the write and compare stale bytes.
 */
static int write_block(u32 lba, const u32 *src)
{
    u32 w;
    int ret;

    ret = wait_inhibit(SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT);
    if (ret)
        return ret;

    write16(SDHCI_BLOCK_SIZE, SECTOR_SIZE);
    write16(SDHCI_BLOCK_COUNT, 1);
    write16(SDHCI_TRANSFER_MODE, SDHCI_TRNS_BLK_CNT_EN);
    dsb();

    ret = send_cmd(MMC_CMD_WRITE_SINGLE_BLOCK, lba,
                   SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC_CHECK |
                   SDHCI_CMD_INDEX_CHECK | SDHCI_CMD_DATA_PRESENT);
    if (ret)
        return ret;

    ret = wait_int(SDHCI_INT_BUF_WRITE_RDY, CMD_TIMEOUT_TICKS,
                   STORAGE_ERR_DATA);
    if (ret)
        return ret;

    for (w = 0; w < SECTOR_SIZE / 4; w++)
        write32(SDHCI_BUFFER, src[w]);

    return wait_int(SDHCI_INT_XFER_COMPLETE, CMD_TIMEOUT_TICKS,
                    STORAGE_ERR_DATA);
}

/* CMD13 until the card is back in TRAN with READY_FOR_DATA. */
static int mmc_wait_ready(struct storage_id *out)
{
    u32 start = timer_ticks();
    u32 resp;
    int ret;

    for (;;) {
        ret = mmc_cmd(MMC_CMD_SEND_STATUS, out->rca << 16,
                      SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC_CHECK |
                      SDHCI_CMD_INDEX_CHECK, &resp);
        if (ret)
            return ret;
        out->r1 = resp;
        if (MMC_R1_STATE(resp) == MMC_STATE_TRAN &&
            (resp & MMC_R1_READY_FOR_DATA))
            return STORAGE_OK;
        if (timer_ticks() - start > BUSY_TIMEOUT_TICKS) {
            sdhci_recover();
            return STORAGE_ERR_BUSY;
        }
        mdelay(1);
    }
}
#endif

/*
 * P13: `nblocks` consecutive sectors in one CMD18, PIO, still no DMA.
 *
 * The command is set up exactly like read_block() except for BLOCK_COUNT,
 * MULTI, BLK_CNT_EN and auto-CMD12. The card streams the blocks back to back
 * and the host handler raises BUF_READ_RDY once per block, so the FIFO is
 * drained 128 words at a time inside the loop rather than after the fact --
 * with auto-CMD12 the transfer needs no separate CMD12 from us. Read-only.
 */
static int read_blocks_multi(u32 lba, u32 *dst, u32 nblocks)
{
    u32 b, w;
    int ret;

    ret = wait_inhibit(SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT);
    if (ret)
        return ret;

    write16(SDHCI_BLOCK_SIZE, SECTOR_SIZE);
    write16(SDHCI_BLOCK_COUNT, (u16)nblocks);
    write16(SDHCI_TRANSFER_MODE, SDHCI_TRNS_READ | SDHCI_TRNS_MULTI |
                                SDHCI_TRNS_BLK_CNT_EN |
                                SDHCI_TRNS_AUTO_CMD12);
    dsb();

    ret = send_cmd(MMC_CMD_READ_MULTIPLE_BLOCK, lba,
                   SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC_CHECK |
                   SDHCI_CMD_INDEX_CHECK | SDHCI_CMD_DATA_PRESENT);
    if (ret)
        return ret;

    for (b = 0; b < nblocks; b++) {
        ret = wait_int(SDHCI_INT_BUF_READ_RDY, CMD_TIMEOUT_TICKS,
                       STORAGE_ERR_DATA);
        if (ret)
            return ret;
        for (w = 0; w < SECTOR_SIZE / 4; w++)
            *dst++ = read32(SDHCI_BUFFER);
    }

    return wait_int(SDHCI_INT_XFER_COMPLETE, CMD_TIMEOUT_TICKS,
                    STORAGE_ERR_DATA);
}

/*
 * Recovery for a CMD18 that failed part way through. Auto-CMD12 only fires
 * when the transfer completes, so an aborted one leaves the card expecting to
 * keep streaming: reset the host, send the explicit CMD12, reset again. The
 * caller re-reads the same range with the CMD17 loop, so nothing here has to
 * preserve the partial data.
 */
static void stop_multi(void)
{
    sdhci_reset(SDHCI_RESET_CMD | SDHCI_RESET_DATA);
    write16(SDHCI_INT_STATUS, read16(SDHCI_INT_STATUS));
    write16(SDHCI_ERR_INT_STATUS, read16(SDHCI_ERR_INT_STATUS));

    (void)send_cmd(MMC_CMD_STOP_TRANSMISSION, 0,
                   SDHCI_CMD_RESP_BUSY | SDHCI_CMD_CRC_CHECK |
                   SDHCI_CMD_INDEX_CHECK);

    sdhci_reset(SDHCI_RESET_CMD | SDHCI_RESET_DATA);
}

static int check_args(void *dst, u32 nblocks)
{
    if (nblocks == 0 || nblocks > STORAGE_MAX_BLOCKS || dst == 0 ||
        ((u32)dst & 3) != 0)
        return STORAGE_ERR_ARG;
    return STORAGE_OK;
}

/*
 * P12: pick the fastest data-phase divider that still reads a block cleanly.
 *
 * CAPABILITIES[15:8] reports 0xC0 here, i.e. 192 MHz, and that is the value
 * both clock setups divide from. Two measurements say the divided-clock base
 * is really 48 MHz, so every divider so far ran 4x slower than intended:
 *
 *   div 4   intended 24 MHz -> 0.171 ms/block, measured 0.797 ms (P11)
 *   div 240 intended 400 kHz -> 10.24 ms/block, measured 42.4 ms (r97 era)
 *
 * Both ratios are 4, which is exactly 192/48. Deduced rather than trusted:
 * block 0 is read once at div 4, 2 and 1, the fastest error-free result wins,
 * and the session keeps that divider. Div 1 is the 24 MHz target, div 4 the
 * divider earlier builds shipped with, so a failure always has a known-good
 * fallback. Read-only; nothing here touches the card's state.
 */
static u32 cal_buf[SECTOR_SIZE / sizeof(u32)];

static u32 cal_measure(u32 div, int *err)
{
    u32 start = timer_ticks();

    sdhci_set_clock(div);
    *err = read_block(0, cal_buf);
    return timer_ticks() - start;
}

static int sdhci_pick_div(struct storage_id *id)
{
    static const u32 cand[3] = { 4, 2, 1 };
    u32 best_div = 4, best_ticks = 0;
    u32 i;
    int err;

    for (i = 0; i < 3; i++) {
        u32 dt = cal_measure(cand[i], &err);

        id->cal_div[i] = cand[i];
        id->cal_ticks[i] = (u32)err ? 0 : dt;
        if (err == STORAGE_OK) {
            if (best_ticks == 0 || dt < best_ticks) {
                best_ticks = dt;
                best_div = cand[i];
            }
            continue;
        }

        /* A failing divider may leave a transaction in flight. Go back to the
         * best one and prove the card still answers before carrying on. */
        sdhci_set_clock(best_div);
        err = read_block(0, cal_buf);
        if (err)
            return err;
    }

    sdhci_set_clock(best_div);
    id->cal_sel = best_div;
    return STORAGE_OK;
}

int storage_open(struct storage_id *out)
{
    struct storage_id scratch;
    struct storage_id *id = out ? out : &scratch;
    int ret;

    struct_clear(id, sizeof(*id));

    sdhci_begin(id);
    ret = mmc_ident_seq(id);

    /* Identification is over: from here on the clock is the data-phase one. */
    if (ret == STORAGE_OK)
        ret = sdhci_pick_div(id);

    /* No sdhci_end() here: the caller owns the session and closes it. The
     * registers are restored just as promptly on the error path because every
     * caller does storage_close() unconditionally. */
    id->err = (u32)ret;
    return ret;
}

int storage_read_sectors_live(u32 lba, void *dst, u32 nblocks,
                              struct storage_id *out)
{
    struct storage_id scratch;
    struct storage_id *id = out ? out : &scratch;
    u32 i;
    int ret;

    /* No struct_clear() here: the session fields (caps/cid/csd/rca) belong to
     * storage_open() and are part of the report. */
    ret = check_args(dst, nblocks);
    if (ret) {
        id->err = (u32)ret;
        return ret;
    }

    /* P13: one CMD18 covers the whole range. On failure the first error is
     * recorded, the card is taken out of the streaming state and the same
     * range is re-read below with the known-good CMD17 loop. */
    if (nblocks > 1) {
        ret = read_blocks_multi(lba, (u32 *)dst, nblocks);
        if (ret == STORAGE_OK) {
            id->multi_sectors += nblocks;
            id->step = 8 + nblocks;
            id->err = STORAGE_OK;
            return STORAGE_OK;
        }
        if (id->multi_err == 0)
            id->multi_err = (u32)ret;
        stop_multi();
    }

    for (i = 0; i < nblocks; i++) {
        id->step = 8 + i;         /* step 8..11 = sector being read */
        ret = read_block(lba + i,
                         (u32 *)dst + (i * (SECTOR_SIZE / sizeof(u32))));
        if (ret) {
            id->err = (u32)ret;
            return ret;
        }
    }
    id->step = 8 + nblocks;       /* 12 = all sectors read */
    id->err = STORAGE_OK;
    return STORAGE_OK;
}

#if STORAGE_ALLOW_WRITE
int storage_write_sectors_live(u32 lba, const void *src, u32 nblocks,
                               struct storage_id *out)
{
    struct storage_id scratch;
    struct storage_id *id = out ? out : &scratch;
    u32 i;
    int ret;

    /* No struct_clear(): the session fields belong to storage_open(). */
    if (nblocks == 0 || nblocks > STORAGE_MAX_BLOCKS || src == 0 ||
        ((u32)src & 3) != 0) {
        id->err = STORAGE_ERR_ARG;
        return STORAGE_ERR_ARG;
    }

    for (i = 0; i < nblocks; i++) {
        ret = write_block(lba + i,
                          (const u32 *)src + (i * (SECTOR_SIZE / sizeof(u32))));
        if (ret == STORAGE_OK)
            ret = mmc_wait_ready(id);
        if (ret) {
            /* Keep the first failure: a retry must not overwrite the reason
             * the write session was abandoned. */
            if (id->write_err == 0)
                id->write_err = (u32)ret;
            id->err = (u32)ret;
            return ret;
        }
        id->write_sectors++;
    }
    id->err = STORAGE_OK;
    return STORAGE_OK;
}
#endif

void storage_close(struct storage_id *out)
{
    struct storage_id scratch;
    struct storage_id *id = out ? out : &scratch;

    sdhci_end(id);
}

int storage_ident(struct storage_id *out)
{
    int ret;

    struct_clear(out, sizeof(*out));

    sdhci_begin(out);
    ret = mmc_ident_seq(out);
    sdhci_end(out);

    out->err = (u32)ret;
    return ret;
}

int storage_read_sectors(u32 lba, void *dst, u32 nblocks,
                         struct storage_id *out)
{
    struct storage_id scratch;
    struct storage_id *id = out ? out : &scratch;
    int ret;

    struct_clear(id, sizeof(*id));

    ret = check_args(dst, nblocks);
    if (ret) {
        id->err = (u32)ret;
        return ret;
    }

    ret = storage_open(id);
    if (ret == STORAGE_OK)
        ret = storage_read_sectors_live(lba, dst, nblocks, id);
    storage_close(id);

    id->err = (u32)ret;
    return ret;
}
