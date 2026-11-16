#ifndef A7C_STORAGE_H
#define A7C_STORAGE_H

#include "io.h"

/*
 * eMMC access for the A7C (CXD90045).  Read-only unless the build says
 * otherwise.
 *
 * Identification: CMD0 / CMD1 / CMD2 / CMD3 / CMD9 / CMD7 / CMD13.
 * Data, reads: CMD17 (single block) and CMD18 (multi block).
 * Data, writes: CMD24 (single block) exists only when STORAGE_ALLOW_WRITE is
 * 1 on the compiler command line.  The default build has no write constant,
 * no write function and no write counters at all, so it cannot program the
 * card by construction rather than by a runtime flag.
 *
 * STORAGE_ALLOW_WRITE=1 is only ever set by build.ps1 -Write, which produces
 * a separate test image; it is an authorisation for one named range in
 * fastboot.c's whitelist, not a general capability.
 *
 * Identification runs at 400 kHz or less; the data phase uses the fastest of
 * the dividers storage_open() measured, inside the eMMC default-speed range and
 * with no card-side mode switch -- only the host divider at 0xF0304000 changes.
 * (P12: CAPABILITIES claims a 192 MHz base, but the divided clock behaves as if
 * it were 48 MHz, so earlier builds' "24 MHz" was really 6 MHz. See
 * sdhci_pick_div() in storage.c.)
 *
 * The only stores performed are to SDHCI host registers at 0xF0304000, which
 * any read of the card requires; the previous HOST_CONTROL and CLOCK_CONTROL
 * values are restored before returning.
 */

#define STORAGE_OK           0
#define STORAGE_ERR_INHIBIT  1   /* command/data line never went idle */
#define STORAGE_ERR_TIMEOUT  2   /* no completion interrupt in time */
#define STORAGE_ERR_CMD      3   /* controller raised an error interrupt */
#define STORAGE_ERR_BUSY     4   /* R1b busy stage never finished */
#define STORAGE_ERR_STATE    5   /* CMD13 answered but not TRAN+ready */
#define STORAGE_ERR_ARG      6   /* bad lba/count/buffer argument */
#define STORAGE_ERR_DATA     7   /* data stage failed (BUF_READ_RDY/XFER) */

/* Writes are compiled out unless the build asks for them (build.ps1 -Write,
 * which also names the one range the fastboot layer may write). */
#ifndef STORAGE_ALLOW_WRITE
#define STORAGE_ALLOW_WRITE  0
#endif

/* Upper bound on one read call: the caller's buffer is the real limit.
 * 32 blocks = 16 KiB, which is exactly the upload chunk (fastboot.c). */
#define STORAGE_MAX_BLOCKS   32

struct storage_id {
    u32 caps;             /* CAPABILITIES before touching anything */
    u16 host_version;
    u16 clock_before;     /* CLOCK_CONTROL as the boot chain left it */
    u8  host_before;      /* HOST_CONTROL as the boot chain left it */
    u32 pstate_before;
    u32 ocr;              /* last CMD1 response */
    u32 cid[4];           /* RESPONSE 0x10..0x1F, word 0 = resp[0] */
    u32 csd[4];
    u32 rca;              /* RCA we assigned in CMD3 (host choice) */
    u32 r1;               /* CMD13 status word */
    u32 pstate_after;
    u32 step;             /* 1..7: last command attempted */
    u32 err;              /* STORAGE_* */
    /* P12: data-phase divider calibration performed by storage_open().
     * cal_div[] is the divider tried (4, 2, 1) and cal_ticks[] the one-block
     * read time at it, 0 meaning that divider failed. cal_sel is the divider
     * the session ended up using. */
    u32 cal_div[3];
    u32 cal_ticks[3];
    u32 cal_sel;
    /* P13: CMD18 multi-block reads. multi_sectors counts the sectors that
     * came back through the CMD18 path; multi_err keeps the first CMD18
     * failure, 0 while it has never failed. `oem armed` prints both. */
    u32 multi_sectors;
    u32 multi_err;
#if STORAGE_ALLOW_WRITE
    /* M-7: how many sectors this session programmed, and the first write
     * failure (0 while none has failed).  Only present in a -Write build. */
    u32 write_sectors;
    u32 write_err;
#endif
};

/* Returns STORAGE_OK or the first error; `out` is filled either way. */
int storage_ident(struct storage_id *out);

/*
 * Read `nblocks` 512-byte sectors starting at block address `lba` into `dst`.
 *
 * The identification sequence runs first, because the boot chain does not
 * leave the card selected and a bare CMD17 would come back with no response.
 * `dst` must be 4-byte aligned; `nblocks` is bounded by STORAGE_MAX_BLOCKS.
 *
 * Multi-sector ranges go through one CMD18 (P13) and fall back to a CMD17
 * loop if that fails. There is no register setup here that a write would need
 * either.
 */
int storage_read_sectors(u32 lba, void *dst, u32 nblocks,
                         struct storage_id *out);

/*
 * Session form of the same read, for bulk export.
 *
 * The identification sequence costs about one second (CMD0/1/2/3/9/7/13 at
 * 400 kHz) and dominates a 16 KiB chunk, so an export of N chunks would pay it
 * N times. storage_open() pays it once; afterwards the card stays selected in
 * TRAN and every chunk is one CMD18 (P13), with the CMD17 loop as the fallback
 * for a chunk that fails. storage_open() closes the identification clock and
 * then calibrates the data-phase divider (three one-block reads), which both
 * paths inherit. Read-only throughout.
 *
 * storage_close() restores HOST_CONTROL / CLOCK_CONTROL and must run on every
 * path, including the error ones. P11 keeps the session open *across* fastboot
 * commands (fastboot.c owns it), so `upload` no longer closes on success.
 */
int  storage_open(struct storage_id *out);
int  storage_read_sectors_live(u32 lba, void *dst, u32 nblocks,
                               struct storage_id *out);
void storage_close(struct storage_id *out);

#if STORAGE_ALLOW_WRITE
/*
 * Write `nblocks` 512-byte sectors from `src` to block address `lba`.
 *
 * Only a -Write build has this symbol at all.  It runs inside an open
 * session (storage_open() must have succeeded on the same `id`), programs one
 * sector per CMD24, and after each command waits for the card to leave its
 * programming state (CMD13 READY_FOR_DATA + TRAN) before returning, so a
 * read-back issued right afterwards sees the new bytes.
 *
 * `src` must be 4-byte aligned and `nblocks` is bounded by STORAGE_MAX_BLOCKS.
 * An error stops the loop and is kept in `out->write_err`; the caller owns the
 * decision to retry or to restore from a backup.
 */
int storage_write_sectors_live(u32 lba, const void *src, u32 nblocks,
                               struct storage_id *out);
#endif

#endif /* A7C_STORAGE_H */
