#ifndef A7C_DARWIN_H
#define A7C_DARWIN_H

#include "io.h"

/*
 * Darwin is the A7C's system power MCU. It is reached over SIO channel 3 and
 * a GPIO chip select, both taken from the vendor's own chain:
 *
 *   ud_darwin_init.elf   reads Backup 0xF900D4 (channel 3), 0xF900B1 (CS
 *                        port 5), 0xF9006D (CS bit 14), configures the four
 *                        pins around the SIO pads, then opens /dev/sio3
 *   sio.ko               sio_master_transfer @0x638 plus the load helper
 *                        @0x570 give the register sequence and values below
 *   usb_darwin.ko        gpio_set_data_bit(port 5, bit 14, 0|1) around every
 *                        0x80-byte exchange
 *
 * Read-only by construction: this file has no 0x12 (write block) command, so
 * the only thing it can ask Darwin for is command 0x16 (read).
 */

#define DARWIN_PKT      0x80    /* one exchange, both directions */

/* SIO: per-channel stride 0x200, channel 3 -> 0xF2405000 + 3 * 0x200. The
 * same 0x80-byte buffer at +0x100 carries TX in and RX out. */
#define SIO_BASE        0xF2405600u
#define SIO_CTRL        (SIO_BASE + 0x00)
#define SIO_CTRL2       (SIO_BASE + 0x04)
#define SIO_REG08       (SIO_BASE + 0x08)
#define SIO_LEN         (SIO_BASE + 0x0C)
#define SIO_CFG         (SIO_BASE + 0x10)
#define SIO_RATE_LO     (SIO_BASE + 0x14)
#define SIO_RATE_HI     (SIO_BASE + 0x18)
#define SIO_DATA        (SIO_BASE + 0x100)

/*
 * sio_master_transfer @0x638 is, in order:
 *
 *   sio_load(ctx)   FIFO <- tx, REG08 <- 0, LEN <- (len-1) << 24
 *   CFG      = 0x01000000
 *   CTRL2    = (byte8 << 24) | 0x30000000
 *   RATE_LO  = tx[12] << 24
 *   RATE_HI  = (tx[12] << 16) & 0xFF000000
 *   CTRL     = (byte9 << 24) | 0x28000000   <- written last, starts it
 *
 * With the loader's own words (ioctl 0x40046742 arg 0x13, ioctl 0x40046741
 * arg 0x1000: byte8 = 0x00, byte9 = 0x10, tx[12] = 0x13) that is CFG
 * 0x01000000, CTRL2 0x30000000, RATE_LO 0x13000000, RATE_HI 0, and
 * CTRL 0x38000000 = bit 29 (go) | 28 | 27 (setup) -- setup and go in one
 * register write, and the FIFO filled before the block is enabled.
 *
 * sio_load @0x570 returns -16 while bit 29 is set, so the previous frame has
 * to be acknowledged before the FIFO can be refilled. sio_int_handler @0x10c
 * does that with (ctrl & 0x15000000) | 0xC0000000: bits 31/30 are a
 * write-1-to-clear stop/ack, which also drops the start bit.
 *
 * RATE stays at the A7C loader's own 0x13, not 7M4's 3: this is the value
 * the vendor programs on this SoC for the live Darwin link.
 *
 * SIO_CTRL_IDLE is the value sio_probe() writes while holding the block off.
 */
#define SIO_CFG_ON      0x01000000u
#define SIO_CTRL2_VAL   0x30000000u
#define SIO_RATE_LO_VAL 0x13000000u
#define SIO_RATE_HI_VAL 0x00000000u
#define SIO_CTRL_START  0x38000000u
#define SIO_CTRL_STOP   0xC0000000u
#define SIO_CTRL_IDLE   0xD0000000u
#define SIO_CTRL_BUSY   BIT(29)
#define SIO_LEN_VAL     ((u32)(DARWIN_PKT - 1) << 24)

/* CS: GPIO port 5 bit 14, active low. +0x44 drives high, +0x48 low. */
#define GPIO_PORT5_BASE 0xF2910500u
#define GPIO_CS_SET     (GPIO_PORT5_BASE + 0x44)
#define GPIO_CS_CLR     (GPIO_PORT5_BASE + 0x48)
#define CS_BIT          BIT(14)

/*
 * The vendor's four-round nibble setup writes the same set of port-5 and
 * 0xF2912500 registers once per pin. Bit 14 keeps GPIO function, direction
 * output and level high; bits 15/13/12 are handed back to the SIO block
 * (FUNC_CLR) with their direction and output latches cleared.
 */
#define GPIO_PORT5_FUNC_SET (GPIO_PORT5_BASE + 0x34)
#define GPIO_PORT5_FUNC_CLR (GPIO_PORT5_BASE + 0x38)
#define GPIO_PORT5_DIR_SET  (GPIO_PORT5_BASE + 0x04)
#define GPIO_PORT5_DIR_CLR  (GPIO_PORT5_BASE + 0x08)
#define GPIO_AUX_BASE       0xF2912500u

/* Chip-select timing. The loader spins on an unidentified tick; setup and
 * hold below are the conservative reading, and both are minimums. */
#define CS_SETUP_US     150u
#define CS_HOLD_US      1200u

/*
 * 0x80 bytes is a few hundred microseconds of shifting; 2 ms is slack.
 *
 * A frame gets a fixed delay rather than a poll: the vendor driver never
 * waits for completion (it rides its ISR), and `oem darwin` on 2026-10-27
 * stayed silent for 70 s with the on-chip timer running, which points at the
 * polling read of SIO_CTRL after a start -- a stalled SIO access cannot be
 * escaped by any iteration count. udelay() only reads TIMER0_COUNTER, so the
 * whole exchange no longer touches SIO between the start write and the FIFO
 * read.
 */
#define FRAME_US        2000u

/* Darwin RAM. Command 0x16 bounds reads to [0x20000000, 0x20004000]. */
#define DARWIN_RAM_BASE 0x20000000u

/*
 * The Virtual WDT counter table and per-channel reload bytes, as on 7M4.
 * A7C has not been confirmed yet: the vector table SP here is 0x200026D0,
 * which is the same +0x2C offset 7M4 uses, so these are the first two
 * addresses to check on the bench.
 */
#define VWDT_TABLE      0x200026FCu
#define VWDT_RELOAD     0x2000003Cu

#define DARWIN_ATTEMPTS 8       /* ~11 ms; short enough never to starve USB */
#define PKT_HDR         6       /* opcode + 4-byte address + length */
#define PKT_MAX_PAYLOAD 0x78

/* Configure the four pins and idle the SIO block. Sends nothing. Idempotent. */
void darwin_init(void);

/*
 * The two halves of darwin_init(), exposed so a bench run can find out which
 * half kills the link. `pins` touches GPIO port 5 and the 0xF2912500 block
 * only; `sio` touches 0xF2405600 only.
 */
void darwin_pins(void);
void darwin_sio_idle(void);

/* Read CTRL, CTRL2, REG08, LEN, CFG, RATE_LO, RATE_HI and DATA[0] back. */
void darwin_sio_regs(u32 v[8]);

/*
 * One raw 0x80-byte transaction. Seals tx's CRC in place (tx must be
 * writable), then repeats the exchange until a reply validates and its first
 * `match_len` bytes echo tx, or `attempts` exchanges have gone by.
 */
int darwin_xfer(u8 *tx, u8 *rx, u32 match_len, u32 attempts);

/* Command 0x16: read `len` (<= 0x78) bytes from any Darwin address. */
int darwin_read(u32 addr, u8 *buf, u32 len);

/* Same, but one exchange only -- used by the stage bisect, where a partial
 * exchange leaves the block mid-frame and a retry would run the preamble
 * again. */
int darwin_read_once(u32 addr, u8 *buf, u32 len);

/*
 * Bench bisect for the exchange that stops the whole payload answering.
 * 0 runs everything; 1..7 stop right after that step instead:
 *
 *   1 stop/ack + busy check      2 chip select low + setup delay
 *   3 FIFO + REG08 + LEN         4 CFG + CTRL2 + RATE
 *   5 CTRL start + frame delay   6 chip select high + hold
 *   7 FIFO read back
 *
 * 8 is the same as 0 for a single exchange (the value is simply not matched
 * anywhere in sio_xfer()).
 */
void darwin_set_stage(u32 stage);

/* Read the four live counters and the three reload bytes. Either may be null. */
int darwin_vwdt_state(u8 counters[4], u8 reloads[3]);

/* Diagnostics from the last darwin_xfer(): exchanges spent, sio_xfer()
 * failures, and the final CTRL read. Kept so `oem darwin` can say why a link
 * that used to answer stopped answering. */
u32 darwin_dbg_attempts(void);
u32 darwin_dbg_errors(void);
u32 darwin_dbg_ctrl(void);

#endif /* A7C_DARWIN_H */
