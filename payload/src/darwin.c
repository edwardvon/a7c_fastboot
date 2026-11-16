#include "io.h"
#include "timer.h"
#include "darwin.h"

/*
 * SIO transport for the Darwin link, transcribed from three vendor pieces --
 * the AP-side init helper, sio.ko's transfer primitive and usb_darwin.ko's
 * chip-select calls (see darwin.h for the addresses). SIO+0x100 is one
 * 0x80-byte buffer used in both directions.
 *
 * Retrying is the protocol, not error handling: Darwin answers a command in a
 * later exchange and returns a CRC of 0xFFFF for "nothing ready yet".
 */

/* ---- CRC-16/CCITT-FALSE -------------------------------------------------
 *
 * Init 0xFFFF, poly 0x1021, MSB-first, no reflection, no final XOR, over
 * bytes [0x00..0x7D], stored little-endian at 0x7E.
 */
static u16 darwin_crc16(const u8 *p)
{
    u32 crc = 0xFFFF;
    u32 i, b;

    for (i = 0; i < DARWIN_PKT - 2; i++) {
        crc ^= (u32)p[i] << 8;
        for (b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        crc &= 0xFFFF;
    }
    return (u16)crc;
}

/* ---- one raw exchange ---------------------------------------------------- */

static void sio_load(const u8 *tx)
{
    u32 i;
    for (i = 0; i < DARWIN_PKT; i += 4)
        write32(SIO_DATA + i, (u32)tx[i] | ((u32)tx[i + 1] << 8) |
                              ((u32)tx[i + 2] << 16) | ((u32)tx[i + 3] << 24));
}

static void sio_store(u8 *rx)
{
    u32 i, v;
    for (i = 0; i < DARWIN_PKT; i += 4) {
        v = read32(SIO_DATA + i);
        rx[i]     = (u8)v;
        rx[i + 1] = (u8)(v >> 8);
        rx[i + 2] = (u8)(v >> 16);
        rx[i + 3] = (u8)(v >> 24);
    }
}

static u32 dbg_attempts, dbg_errors, dbg_ctrl;
static u32 dbg_stage;

void darwin_set_stage(u32 stage)
{
    dbg_stage = stage;
}

static int sio_xfer(const u8 *tx, u8 *rx)
{
    /*
     * A7C's own order, from sio_master_transfer @0x638: fill the FIFO, enable
     * the block, then one CTRL write that both sets up and starts. The 7M4
     * sequence (setup first, separate go bit) is not what this driver does.
     *
     * Perform the stop/ack the ISR would do, so sio_load @0x570 finds bit 29
     * clear; it returns -16 instead of loading while the last frame is still
     * unacknowledged.
     */
    write32(SIO_CTRL, SIO_CTRL_STOP);
    write32(SIO_CFG, 0);

    if (read32(SIO_CTRL) & SIO_CTRL_BUSY) {
        dbg_errors++;
        dbg_ctrl = read32(SIO_CTRL);
        return -1;
    }
    if (dbg_stage == 1)
        return 0;

    /* usb_darwin.ko asserts the select around every exchange. */
    write32(GPIO_CS_CLR, CS_BIT);
    udelay(CS_SETUP_US);
    if (dbg_stage == 2)
        return 0;

    sio_load(tx);
    write32(SIO_REG08, 0);
    write32(SIO_LEN,   SIO_LEN_VAL);
    if (dbg_stage == 3)
        return 0;

    write32(SIO_CFG,     SIO_CFG_ON);
    write32(SIO_CTRL2,   SIO_CTRL2_VAL);
    write32(SIO_RATE_LO, SIO_RATE_LO_VAL);
    write32(SIO_RATE_HI, SIO_RATE_HI_VAL);
    if (dbg_stage == 4)
        return 0;

    write32(SIO_CTRL,    SIO_CTRL_START);

    /*
     * Fixed delay instead of polling: the vendor driver leaves completion to
     * its ISR, and a poll of SIO_CTRL right after the start is the one access
     * that has silenced the link before. Anything wrong with the frame is
     * caught upstream by the CRC and header match.
     */
    udelay(FRAME_US);
    if (dbg_stage == 5)
        return 0;

    write32(GPIO_CS_SET, CS_BIT);
    udelay(CS_HOLD_US);
    if (dbg_stage == 6)
        return 0;

    sio_store(rx);
    if (dbg_stage == 7)
        return 0;

    write32(SIO_CTRL, SIO_CTRL_STOP);
    write32(SIO_CFG, 0);
    dbg_ctrl = read32(SIO_CTRL);
    return 0;
}

/* ---- public transport ---------------------------------------------------- */

/*
 * The vendor's four-round nibble setup, one round per pin: bit 15, 14, 13
 * then 12 of GPIO port 5, with the channel number (3) selecting the shift
 * (1 << (3 * 4 + round)). Each round writes the port-5 pinmux/direction/data
 * registers and the 0xF2912500 block in the same order the loader does.
 *
 * Round 2 (bit 14) is the chip select: GPIO function, output, level high.
 * The other three are the SIO clock/data pads and are handed back to the SIO
 * block (FUNC_CLR) -- the same split 7M4 makes for XCS vs SIO_PINS.
 */
static void pin_round(u32 mask, u32 func, u32 aux_first,
                      u32 aux_mid0, u32 aux_mid1, u32 dir, u32 wdata)
{
    write32(func, mask);
    write32(aux_first, mask);
    write32(GPIO_AUX_BASE + 0x54, mask);
    write32(aux_mid0, mask);
    write32(aux_mid1, mask);
    write32(dir, mask);
    write32(wdata, mask);
}

void darwin_pins(void)
{
    pin_round(0x8000u, GPIO_PORT5_FUNC_CLR, GPIO_AUX_BASE + 0x44,
              GPIO_AUX_BASE + 0x04, GPIO_AUX_BASE + 0x18,
              GPIO_PORT5_DIR_CLR, GPIO_CS_CLR);
    pin_round(0x4000u, GPIO_PORT5_FUNC_SET, GPIO_AUX_BASE + 0x48,
              GPIO_AUX_BASE + 0x04, GPIO_AUX_BASE + 0x18,
              GPIO_PORT5_DIR_SET, GPIO_CS_SET);
    pin_round(0x2000u, GPIO_PORT5_FUNC_CLR, GPIO_AUX_BASE + 0x44,
              GPIO_AUX_BASE + 0x04, GPIO_AUX_BASE + 0x18,
              GPIO_PORT5_DIR_CLR, GPIO_CS_CLR);
    pin_round(0x1000u, GPIO_PORT5_FUNC_CLR, GPIO_AUX_BASE + 0x44,
              GPIO_AUX_BASE + 0x08, GPIO_AUX_BASE + 0x14,
              GPIO_PORT5_DIR_CLR, GPIO_CS_CLR);
}

void darwin_sio_idle(void)
{
    /* Park the SIO block the way sio_probe() does. Nothing is sent. */
    write32(SIO_CFG, 0);
    write32(SIO_CTRL, SIO_CTRL_IDLE);
    write32(SIO_CTRL2, SIO_CTRL2_VAL);
    dsb();
}

void darwin_sio_regs(u32 v[8])
{
    v[0] = read32(SIO_CTRL);
    v[1] = read32(SIO_CTRL2);
    v[2] = read32(SIO_REG08);
    v[3] = read32(SIO_LEN);
    v[4] = read32(SIO_CFG);
    v[5] = read32(SIO_RATE_LO);
    v[6] = read32(SIO_RATE_HI);
    v[7] = read32(SIO_DATA);
}

void darwin_init(void)
{
    darwin_pins();
    darwin_sio_idle();
}

int darwin_xfer(u8 *tx, u8 *rx, u32 match_len, u32 attempts)
{
    u32 n;
    u16 crc;

    crc = darwin_crc16(tx);
    tx[DARWIN_PKT - 2] = (u8)crc;
    tx[DARWIN_PKT - 1] = (u8)(crc >> 8);

    dbg_attempts = 0;
    dbg_errors = 0;

    for (n = 0; n < attempts; n++) {
        int rc;

        /* sio_xfer() owns the chip select, so the levels sit inside the
         * block's setup window exactly as on 7M4. */
        rc = sio_xfer(tx, rx);

        dbg_attempts = n + 1;

        if (rc == 0) {
            u16 want = darwin_crc16(rx);
            u16 got  = (u16)(rx[DARWIN_PKT - 2] |
                             ((u16)rx[DARWIN_PKT - 1] << 8));
            u32 i;

            /*
             * A validating reply is not necessarily ours: a call that timed
             * out leaves its reply unconsumed, and both handlers leave the
             * request header intact in the buffer they reply from (0x16
             * bzeroes only from +6), so match_len pins it to this command.
             */
            if (want == got) {
                for (i = 0; i < match_len; i++)
                    if (rx[i] != tx[i])
                        break;
                if (i == match_len)
                    return 0;
            }
        } else {
            dbg_errors++;
        }
    }
    return -1;
}

/* ---- commands ------------------------------------------------------------ */

#define CMD_MEM         0x16

/* The address field is LITTLE-endian. */
static void put_addr(u8 *p, u32 v)
{
    p[0] = (u8)v;
    p[1] = (u8)(v >> 8);
    p[2] = (u8)(v >> 16);
    p[3] = (u8)(v >> 24);
}

static int read_with(u32 addr, u8 *buf, u32 len, u32 attempts)
{
    u8 tx[DARWIN_PKT], rx[DARWIN_PKT];
    u32 i;

    if (len == 0 || len > PKT_MAX_PAYLOAD)
        return -1;

    for (i = 0; i < DARWIN_PKT; i++)
        tx[i] = 0;
    tx[0] = CMD_MEM;
    put_addr(tx + 1, addr);
    tx[5] = (u8)len;            /* bit 7 clear selects the read path */

    /* Match the whole header: opcode, address and length. */
    if (darwin_xfer(tx, rx, PKT_HDR, attempts) != 0)
        return -1;

    /* Darwin answers in place, so the payload comes back at PKT_HDR. */
    for (i = 0; i < len; i++)
        buf[i] = rx[PKT_HDR + i];
    return 0;
}

int darwin_read(u32 addr, u8 *buf, u32 len)
{
    return read_with(addr, buf, len, DARWIN_ATTEMPTS);
}

int darwin_read_once(u32 addr, u8 *buf, u32 len)
{
    return read_with(addr, buf, len, 1);
}

/* ---- the Virtual WDT ----------------------------------------------------- */

/*
 * A reload of 0x0A means the AP populated Darwin's config during a normal
 * boot this power cycle; 0x00 means it did not. A counter of 0xFF is the
 * disabled sentinel; 0x00 means the tick is not running, since a live tick
 * turns 0x00 into 0xFF within one 100 ms period.
 */
int darwin_vwdt_state(u8 counters[4], u8 reloads[3])
{
    if (counters && darwin_read(VWDT_TABLE, counters, 4) != 0)
        return -1;
    if (reloads && darwin_read(VWDT_RELOAD, reloads, 3) != 0)
        return -1;
    return 0;
}

u32 darwin_dbg_attempts(void) { return dbg_attempts; }
u32 darwin_dbg_errors(void)   { return dbg_errors; }
u32 darwin_dbg_ctrl(void)     { return dbg_ctrl; }
