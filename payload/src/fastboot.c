#include "fastboot.h"
#include "darwin.h"
#include "dwc3.h"
#include "io.h"
#include "led.h"
#include "part.h"
#include "storage.h"
#include "timer.h"
#include "usb.h"

/*
 * Minimal fastboot: getvar, download, flash, oem peek, oem storage,
 * oem partition, oem read, upload, oem armed, oem darwin, reboot.
 *
 * Framing is the four-byte tag protocol:
 *   OKAY<result>  FAIL<reason>  INFO<message>  DATA<8 hex digits>
 * Every reply is a single bulk transfer on the IN endpoint.
 *
 * Read-only by construction in the default build: the only data path into the
 * card is CMD17/CMD18 (storage.c), so `upload` can stream a partition out and
 * nothing in the image can stream one back in.  CMD24 exists only in a
 * build.ps1 -Write image, whose flash whitelist names the two zones a session
 * was authorised for (M-7: one sector, M-8: 2 MiB).
 */

#define FB_RESPONSE_MAX  64
#define FB_TAG_LEN       4
#define FB_COMMAND_MAX   64

/*
 * The whole payload must fit in the 64K ESRAM window, and this buffer is the
 * largest thing in it. 16 KB is enough to demonstrate a full transfer round
 * trip; the 1.82 GB image path (C2) will stream rather than buffer.
 */
#define DOWNLOAD_MAX     (16 * 1024)

/* One `upload` chunk; also the largest single card read (see storage.h). */
#define UPLOAD_CHUNK     DOWNLOAD_MAX

static u8 cmd_buf[BULK_MAXPACKET]                     __attribute__((aligned(64)));
static u8 resp_buf[FB_RESPONSE_MAX]                   __attribute__((aligned(64)));
/*
 * usb_bulk_recv() rounds up to a whole packet, so the last chunk can write up
 * to BULK_MAXPACKET-1 bytes past the requested length.
 */
static u8 download_buf[DOWNLOAD_MAX + BULK_MAXPACKET] __attribute__((aligned(64)));
static u32 download_len;

/* An upload chunk has to fit the buffer above and one card read call. */
typedef char upload_chunk_fits[
    (UPLOAD_CHUNK <= sizeof(download_buf) &&
     UPLOAD_CHUNK / PART_SECTOR_SIZE <= STORAGE_MAX_BLOCKS) ? 1 : -1];

/* ---- tiny string helpers (no libc) -------------------------------------- */

static int str_eq(const char *a, const char *b)
{
    while (*a && *b) {
        if (*a != *b)
            return 0;
        a++; b++;
    }
    return *a == *b;
}

/* Does `s` start with `prefix`? Returns the rest of `s`, or 0. */
static const char *str_after(const char *s, const char *prefix)
{
    while (*prefix) {
        if (*s != *prefix)
            return 0;
        s++; prefix++;
    }
    return s;
}

/* Append src at dst[n], bounded by the whole buffer. Returns the new length. */
static u32 str_append(char *dst, u32 n, u32 size, const char *src)
{
    while (*src && n + 1 < size)
        dst[n++] = *src++;
    return n;
}

static u32 str_copy(char *dst, const char *src, u32 max)
{
    u32 n = 0;
    while (src[n] && n < max) {
        dst[n] = src[n];
        n++;
    }
    return n;
}

static const char hexdigits[] = "0123456789abcdef";

static u32 hex_format(char *dst, u64 value, int digits)
{
    int i;
    for (i = digits - 1; i >= 0; i--)
        dst[digits - 1 - i] = hexdigits[(value >> (i * 4)) & 0xf];
    return (u32)digits;
}

/* Parse hex, with or without a 0x prefix. Stops at the first non-hex char. */
static u64 hex_parse(const char *s, const char **end)
{
    u64 v = 0;

    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
        s += 2;

    for (;;) {
        char c = *s;
        u32 d;

        if (c >= '0' && c <= '9')       d = (u32)(c - '0');
        else if (c >= 'a' && c <= 'f')  d = (u32)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F')  d = (u32)(c - 'A' + 10);
        else break;

        v = (v << 4) | d;
        s++;
    }

    if (end)
        *end = s;
    return v;
}

/* ---- responses ---------------------------------------------------------- */

static void fb_reply(const char *tag, const char *msg)
{
    u32 n = str_copy((char *)resp_buf, tag, FB_TAG_LEN);
    n = str_append((char *)resp_buf, n, sizeof(resp_buf), msg);
    usb_bulk_send(resp_buf, n);
}

static void fb_okay(const char *msg) { fb_reply("OKAY", msg ? msg : ""); }
static void fb_fail(const char *msg) { fb_reply("FAIL", msg ? msg : ""); }

/* ---- getvar ------------------------------------------------------------- */

static void cmd_getvar(const char *name)
{
    char line[FB_RESPONSE_MAX];
    u32 n;

    if (str_eq(name, "version")) {
        fb_okay("0.5");
    } else if (str_eq(name, "product")) {
        fb_okay("ILCE-7C");
    } else if (str_eq(name, "serialno")) {
        fb_okay("ILCE7C-fastboot");
    } else if (str_eq(name, "max-download-size") ||
               str_eq(name, "downloadsize")) {
        n = str_append(line, 0, sizeof(line), "0x");
        n += hex_format(line + n, DOWNLOAD_MAX, 8);
        line[n] = 0;
        fb_okay(line);
    } else {
        /* Unknown variables must answer OKAY with an empty value, not FAIL --
         * `fastboot getvar all` walks a list and a FAIL aborts the walk. */
        fb_okay("");
    }
}

/* ---- download ----------------------------------------------------------- */

static void cmd_download(const char *args)
{
    char line[16];
    u32 want = (u32)hex_parse(args, 0);
    u32 got = 0;
    u32 n;

    if (want == 0 || want > DOWNLOAD_MAX) {
        fb_fail("data too large");
        return;
    }

    /* DATA<8 hex> tells the host to start sending. */
    n = str_append(line, 0, sizeof(line), "DATA");
    n += hex_format(line + n, want, 8);
    usb_bulk_send(line, n);

    while (got < want) {
        u32 chunk = want - got;
        u32 n;

        if (chunk > BULK_MAXPACKET * 8)
            chunk = BULK_MAXPACKET * 8;

        n = usb_bulk_recv(download_buf + got, chunk);
        if (n == 0) {
            /* Bus reset, timeout, or a rejected command. Adding 0 in a `while
             * (got < want)` would spin forever; tell the host instead. */
            fb_fail("download interrupted");
            return;
        }
        got += n;
    }

    download_len = want;
    fb_okay("");
}

/* ---- flash (gated twice: compile time, then per-name whitelist) --------- */

/*
 * `flash:<name>` is reserved but disabled by default.  The default image
 * refuses every flash before it looks at the name, and has no card-side write
 * path to call even if the gate below were opened.  What follows is the shape
 * an approved write session takes, plus the ranges that stay off limits no
 * matter what (写入禁止.md 第 8 条).
 *
 * build.ps1 -Write produces a separate test image with FB_ALLOW_WRITE=1 and
 * STORAGE_ALLOW_WRITE=1.  That image carries only the zones a session was
 * authorised for, and it writes what `download:` buffered, reads the same
 * sectors back through CMD17 and compares them byte for byte before answering
 * OKAY.  The host does not have to trust the write path, and a mismatch fails
 * the command while the backup still exists on the host.
 *
 * `flash:<zone>` writes at the start of the zone; `flash:<zone>@<hex offset>`
 * writes one chunk inside it, which is how a range larger than the 16 KiB
 * download buffer goes back (M-8: 2 MiB in 128 chunks).
 */
#ifndef FB_ALLOW_WRITE
#define FB_ALLOW_WRITE  0
#endif

/* A write-capable fastboot layer needs the card side compiled for writing too.
 * The link would fail on the missing symbol anyway; fail here, where the cause
 * is one line away. */
#if FB_ALLOW_WRITE && !STORAGE_ALLOW_WRITE
#error "FB_ALLOW_WRITE=1 needs STORAGE_ALLOW_WRITE=1 (build.ps1 -Write sets both)"
#endif

struct fb_write_zone {
    const char *name;
    u32         lba;        /* byte start / PART_SECTOR_SIZE */
    u32         sectors;
};

/*
 * Empty on purpose in a read-only build: a zone may only appear here as the
 * result of an approved write session that names the exact medium, partition,
 * range and length (写入禁止.md 第 3、4 条).  An authorisation never carries
 * over to the next build or the next partition.
 */
static const struct fb_write_zone write_zones[] = {
#if FB_ALLOW_WRITE
    /* M-7 (2026-10-28, done): nflasha16, byte 0x80600000 = LBA 2105344, 1
     * sector.  The partition is 140 MB of 0x00 marked reserved; that write was
     * a same-value write-back, so its contents never changed. */
    { "nflasha16_s0", 2105344u, 1u },
    /* M-8 (2026-10-28): nflasha23, byte 0x50800000 = LBA 2637824, 2 MiB
     * (4096 sectors).  The user authorised a whole-partition dump / flash /
     * dump round trip: the host writes back exactly the bytes it dumped. */
    { "nflasha23", 2637824u, 4096u },
#endif
    /* no other zone has ever been authorised */
};

#define FB_WRITE_ZONES \
    ((u32)(sizeof(write_zones) / sizeof(write_zones[0])))

/*
 * Forbidden regardless of any authorisation (写入禁止.md 第 8 条). Byte ranges
 * are from partinf.conf; LBA = byte / 512.
 */
#define FB_DENY_TABLE_LBA      0u          /* LBA 0..127: SDM2 table area  */
#define FB_DENY_TABLE_SECT     128u        /* 4 sectors used, rest is slack */
#define FB_DENY_NFLASHA1_LBA   0x80u       /* 0x00010000 / 512: System Group */
#define FB_DENY_NFLASHA1_SECT  0x3F80u     /* 0x007F0000 / 512: Updater     */
#define FB_DENY_NFLASHA5_LBA   0x2C000u    /* 0x05800000 / 512: Warm Boot   */
#define FB_DENY_NFLASHA5_SECT  0x1E000u    /* 0x03C00000 / 512              */

/*
 * Tripwire: a disabled image must not ship with an allow-list. Adding a zone
 * without flipping FB_ALLOW_WRITE fails the build instead of quietly widening
 * what a future session may touch.
 */
#if !FB_ALLOW_WRITE
typedef char write_whitelist_must_be_empty[(FB_WRITE_ZONES == 0) ? 1 : -1];
#endif

/* Do the sector ranges [a, a+n) and [b, b+m) overlap? */
static int ranges_overlap(u32 a, u32 n, u32 b, u32 m)
{
    if (n == 0 || m == 0)
        return 0;
    return (a < b + m) && (b < a + n);
}

static const struct fb_write_zone *find_write_zone(const char *name)
{
    const struct fb_write_zone *z = write_zones;
    const struct fb_write_zone *end = write_zones + FB_WRITE_ZONES;

    for (; z < end; z++) {
        if (str_eq(name, z->name))
            return z;
    }
    return 0;
}

/* A zone must not even partly touch a forbidden range. */
static int zone_forbidden(u32 lba, u32 sectors)
{
    if (ranges_overlap(lba, sectors, FB_DENY_TABLE_LBA, FB_DENY_TABLE_SECT))
        return 1;
    if (ranges_overlap(lba, sectors, FB_DENY_NFLASHA1_LBA,
                       FB_DENY_NFLASHA1_SECT))
        return 1;
    if (ranges_overlap(lba, sectors, FB_DENY_NFLASHA5_LBA,
                       FB_DENY_NFLASHA5_SECT))
        return 1;
    return 0;
}

#if FB_ALLOW_WRITE
static struct storage_id flash_id;
static u8 verify_buf[PART_SECTOR_SIZE] __attribute__((aligned(64)));
static u32 verify_bad;      /* 0 = every sector compared equal, else sector+1 */

/* Defined further down; both are used by the flash path only. */
static void info_hex(const char *key, u32 value, int digits);
static void up_session_drop(void);

/* Read the range just written back through CMD17 and compare it byte for
 * byte.  Returns the first card error; a content mismatch is reported through
 * verify_bad so the reply can name the sector. */
static int verify_written(u32 lba, u32 sectors)
{
    u32 i, b;
    int ret;

    verify_bad = 0;
    for (i = 0; i < sectors; i++) {
        ret = storage_read_sectors_live(lba + i, verify_buf, 1, &flash_id);
        if (ret)
            return ret;
        for (b = 0; b < PART_SECTOR_SIZE; b++) {
            if (verify_buf[b] != download_buf[i * PART_SECTOR_SIZE + b]) {
                verify_bad = i + 1;
                return STORAGE_ERR_DATA;
            }
        }
    }
    return STORAGE_OK;
}
#endif

static void cmd_flash(const char *args)
{
    char name[FB_RESPONSE_MAX];
    const struct fb_write_zone *zone;
    u32 n = 0;
    u32 sectors;

    while (args[n] && args[n] != ':' &&
#if FB_ALLOW_WRITE
           args[n] != '@' &&
#endif
           n + 1 < FB_RESPONSE_MAX)
        n++;

    if (n == 0) {
        fb_fail("flash: partition name required");
        return;
    }

    str_copy(name, args, n);
    name[n] = '\0';

    if (!FB_ALLOW_WRITE) {
        fb_fail("flash disabled by policy");
        return;
    }

    if (download_len == 0) {
        fb_fail("no data downloaded");
        return;
    }

    /* Round up: a partial last sector is still a touched sector. */
    sectors = (download_len + PART_SECTOR_SIZE - 1) / PART_SECTOR_SIZE;

    zone = find_write_zone(name);
    if (!zone) {
        fb_fail("not on the write whitelist");
        return;
    }

    if (download_len > zone->sectors * PART_SECTOR_SIZE) {
        fb_fail("data exceeds zone");
        return;
    }

    if (zone_forbidden(zone->lba, sectors)) {
        fb_fail("range is forbidden");
        return;
    }

#if FB_ALLOW_WRITE
    {
        u32 off = 0;    /* byte offset inside the zone (M-8 chunked form) */
        u32 lba;
        int ret;

        /* A short tail would program whatever happened to sit behind
         * download_len in the buffer; the rounded-up count above only guards
         * the whitelist. */
        if (download_len < PART_SECTOR_SIZE ||
            (download_len & (PART_SECTOR_SIZE - 1))) {
            fb_fail("download is not a whole number of sectors");
            return;
        }

        if (args[n] == '@') {
            const char *end = 0;

            off = (u32)hex_parse(args + n + 1, &end);
            if (end == args + n + 1 || *end != '\0') {
                fb_fail("bad offset");
                return;
            }
            if (off & (PART_SECTOR_SIZE - 1)) {
                fb_fail("offset is not sector aligned");
                return;
            }
        }

        /* Offset plus length must stay inside the zone; both are 32-bit and
         * a zone is at most a few MiB, so compare as bytes without wrapping. */
        if (off > zone->sectors * PART_SECTOR_SIZE ||
            download_len > zone->sectors * PART_SECTOR_SIZE - off) {
            fb_fail("data exceeds zone");
            return;
        }

        lba = zone->lba + off / PART_SECTOR_SIZE;

        if (zone_forbidden(lba, sectors)) {
            fb_fail("range is forbidden");
            return;
        }

        /* This command starts with CMD0, so any export session is over. */
        up_session_drop();

        ret = storage_open(&flash_id);
        if (ret == STORAGE_OK) {
            ret = storage_write_sectors_live(lba, download_buf,
                                             sectors, &flash_id);
            if (ret == STORAGE_OK)
                ret = verify_written(lba, sectors);
            storage_close(&flash_id);
        }

        /* Every number the host needs to tell "wrote nothing", "wrote and the
         * card disagrees" and "wrote and it matches" apart. */
        info_hex("lba", lba, 8);
        info_hex("off", off, 6);
        info_hex("n", sectors, 4);
        info_hex("w_n", flash_id.write_sectors, 4);
        info_hex("w_err", flash_id.write_err, 1);
        info_hex("v_bad", verify_bad, 4);
        info_hex("err", flash_id.err, 1);

        if (ret == STORAGE_OK && verify_bad == 0)
            fb_okay("write+verify ok");
        else if (verify_bad)
            fb_fail("read-back mismatch (see v_bad)");
        else
            fb_fail("write failed (see w_err/err)");
    }
#else
    /*
     * Unreachable while FB_ALLOW_WRITE is 0, and still a refusal: the default
     * build has no CMD24, so an approved session needs code that is not in
     * this image.
     */
    fb_fail("no write path in this read-only image");
#endif
}

/* ---- oem (read-only) ---------------------------------------------------- */

/*
 * `oem peek:<addr>[:<len>]` -- read-only register/memory dump.
 *
 * This exists so the storage medium can be settled without writing anything:
 * the SDHCI capability registers at 0xF0304000 decide whether the A7C has an
 * eMMC at all. There is deliberately no `oem poke`, and the whitelist below
 * only covers windows already known to be mapped, because an unaligned or
 * unmapped read aborts the core.
 */
#define FB_PEEK_MAX      16
#define FB_PEEK_LINE     64

struct fb_win {
    u32 base;
    u32 size;
};

static const struct fb_win peek_windows[] = {
    { 0xF0000000u, 0x01000000u },   /* peripherals: DWC3, SDHCI */
    { 0xF2000000u, 0x01000000u },   /* timer, USB PHY, aux block */
    { 0xF5000000u, 0x01000000u },   /* NAND controller */
    { 0xF8000000u, 0x00100000u },   /* system controller (+0x620/+0x628/+0x634) */
    { 0xFE000000u, 0x00040000u },   /* eSRAM window (payload lives here) */
};

static int peek_allowed(u32 addr, u32 len)
{
    u64 a = addr;
    u64 e = (u64)addr + len;
    u32 i;

    for (i = 0; i < sizeof(peek_windows) / sizeof(peek_windows[0]); i++) {
        u32 base = peek_windows[i].base;
        u32 end  = base + peek_windows[i].size;

        if (a >= base && e <= end)
            return 1;
    }
    return 0;
}

static void cmd_peek(const char *args)
{
    const char *p;
    u64 addr;
    u32 len, off;
    char line[FB_PEEK_LINE];

    addr = hex_parse(args, &p);
    len  = 4;
    if (*p == ':')
        len = (u32)hex_parse(p + 1, 0);

    if (len == 0 || len > FB_PEEK_MAX) {
        fb_fail("bad length (max 0x10)");
        return;
    }

    /* An unaligned MMIO read faults. */
    addr &= ~3ull;
    len = (len + 3) & ~3u;

    if (addr > 0xFFFFFFFFull || !peek_allowed((u32)addr, len)) {
        fb_fail("address outside read-only whitelist");
        return;
    }

    for (off = 0; off < len; off += 16) {
        u32 n = 0;
        u32 i;

        n += hex_format(line + n, addr + off, 8);
        line[n++] = ':';

        for (i = 0; i < 16 && off + i < len; i += 4) {
            u32 word = read32((u32)(addr + off + i));
            line[n++] = ' ';
            n += hex_format(line + n, word, 8);
        }
        line[n] = 0;
        fb_reply("INFO", line);
    }

    fb_okay("");
}

/* ---- oem storage (read-only card identification) ------------------------ */

/*
 * `oem storage` -- run the identification sequence in storage.c and print
 * every register the answer depends on. Read-only by construction: the only
 * MMC commands storage.c can issue are CMD0/1/2/3/7/9/13, there is no data
 * path at all, so neither CMD24 nor CMD25 (block write) exists in the image.
 */
static struct storage_id storage_result;

/* P11: the MMC session outlives the command that opened it. `oem read` opens it
 * once and `upload` keeps it across chunks, so the card is identified once per
 * export instead of twice per 16 KiB chunk (P10 measured 39.07 ms/片, 56 % of
 * the wall clock). Declared here, not next to the read/upload state, because
 * `oem storage` and `oem partition` have to drop it before they re-identify. */
static struct storage_id up_id;
static int up_sess;

/* P14: the SDM2 table cannot change under this read-only payload, and every
 * `oem read` used to re-read its four sectors (0.72 ms of the 1.12 ms arm
 * term). One load per MMC session is enough; anything that drops the session --
 * `oem storage` starts with CMD0, `oem partition` re-identifies, `reboot`
 * releases the host -- clears the flag so the next export reads the card
 * again. */
static int part_cached;

static void up_session_drop(void)
{
    if (up_sess) {
        storage_close(&up_id);
        up_sess = 0;
    }
    part_cached = 0;
}

static void info_hex(const char *key, u32 value, int digits)
{
    char line[FB_PEEK_LINE];
    u32 n;

    n = str_append(line, 0, sizeof(line), key);
    line[n++] = '=';
    n += hex_format(line + n, value, digits);
    line[n] = 0;
    fb_reply("INFO", line);
}

static void info_words(const char *key, const u32 *w, int count)
{
    char line[FB_PEEK_LINE];
    u32 n = str_append(line, 0, sizeof(line), key);
    int i;

    line[n++] = '=';
    for (i = 0; i < count; i++) {
        if (i)
            line[n++] = ' ';
        n += hex_format(line + n, w[i], 8);
    }
    line[n] = 0;
    fb_reply("INFO", line);
}

static void cmd_storage(void)
{
    struct storage_id *id = &storage_result;
    int ret;

    up_session_drop();    /* P11: this command starts with CMD0 */
    ret = storage_ident(id);

    info_hex("caps", id->caps, 8);
    info_hex("hv", id->host_version, 4);
    info_hex("clk0", id->clock_before, 4);
    info_hex("hc0", id->host_before, 2);
    info_hex("ps0", id->pstate_before, 8);
    info_hex("ps1", id->pstate_after, 8);
    info_hex("step", id->step, 1);
    info_hex("err", id->err, 1);
    info_hex("ocr", id->ocr, 8);
    info_words("cid", id->cid, 4);
    info_words("csd", id->csd, 4);
    info_hex("rca", id->rca, 8);
    info_hex("r1", id->r1, 8);
    info_hex("state", (id->r1 >> 9) & 0xf, 1);

    if (ret == STORAGE_OK)
        fb_okay("");
    else
        fb_fail("storage identification failed (see err)");
}

/* ---- oem partition (read-only SDM2 table dump) -------------------------- */

/*
 * `oem partition` -- read the four table sectors at block address 0 and print
 * every slot the header claims, valid or not, so the host can diff the whole
 * table against partinf.conf instead of only the live partitions.
 *
 * The payload has no CMD24/CMD25, so this can read the card and print it; it
 * cannot alter it. start / count are printed as the bytes stored in the
 * table (see part.h: they are byte offsets, not sectors).
 */
static struct part_table part_result;
static struct storage_id part_id;

/* Decimal, no division: the image is linked -nostdlib, so a / or % by 10 can
 * pull in __aeabi_uidivmod (which the linker then cannot find). */
static u32 dec_format(char *dst, u32 v)
{
    static const u32 pow10[10] = {
        1000000000u, 100000000u, 10000000u, 1000000u, 100000u,
        10000u, 1000u, 100u, 10u, 1u
    };
    u32 n = 0, started = 0, i;

    for (i = 0; i < 10; i++) {
        u32 d = 0;

        while (v >= pow10[i]) {
            v -= pow10[i];
            d++;
        }
        if (d || started || pow10[i] == 1u) {
            dst[n++] = (char)('0' + d);
            started = 1;
        }
    }
    return n;
}

/* "<name padded to 11> <start8> <count8> <type8> <flag8>" -- 46 chars, so it
 * fits the 60 available after the 4-byte tag. */
static void part_line(u32 slot, const struct part_entry *e)
{
    char line[FB_PEEK_LINE];
    u32 n = 0;

    n = str_append(line, n, sizeof(line), "nflasha");
    n += dec_format(line + n, slot);
    while (n < 11)
        line[n++] = ' ';
    n += hex_format(line + n, e->start, 8);
    line[n++] = ' ';
    n += hex_format(line + n, e->count, 8);
    line[n++] = ' ';
    n += hex_format(line + n, e->type, 8);
    line[n++] = ' ';
    n += hex_format(line + n, e->flag, 8);
    line[n] = 0;
    fb_reply("INFO", line);
}

static void cmd_partition(void)
{
    struct part_table *t = &part_result;
    char line[FB_PEEK_LINE];
    u32 n = 0, i;
    int ret;

    up_session_drop();    /* P11: part_load() identifies on its own */
    ret = part_load(t, &part_id);

    n = str_append(line, n, sizeof(line), "sdm2 magic=");
    n += hex_format(line + n, t->magic, 8);
    n = str_append(line, n, sizeof(line), " count=");
    n += hex_format(line + n, t->count, 4);
    n = str_append(line, n, sizeof(line), " valid=");
    n += hex_format(line + n, t->valid, 2);
    n = str_append(line, n, sizeof(line), " extent=");
    n += hex_format(line + n, t->extent, 8);
    line[n] = 0;
    fb_reply("INFO", line);

    for (i = 0; i < t->slots; i++)
        part_line(i + 1, &t->entry[i]);

    info_hex("step", part_id.step, 2);
    info_hex("err", part_id.err, 1);

    if (ret == PART_OK)
        fb_okay("");
    else if (ret == PART_ERR_MAGIC)
        fb_fail("partition table: bad magic");
    else
        fb_fail("partition table read failed (see err)");
}

/* ---- oem read / upload (read-only partition streaming) ------------------ */

/*
 * One command per piece since P14: `oem read` arms the slice and streams it
 * (DATA<total>, the bytes, OKAY). It used to arm here and stream in a second
 * `upload` command with an INFO line in between -- one extra host round trip
 * per 16 KiB piece. `upload` still re-streams what is armed, which is the
 * pre-P14 sequence and the way to repeat a piece without re-arming.
 *
 * The slice is in bytes and must be 512-aligned, because the table stores byte
 * offsets (part.h) while the card is addressed in sectors. `len` defaults to
 * the rest of the partition, rounded down to a whole sector.
 *
 * A failed read or a failed bulk IN part-way through can only return silently:
 * the host has already been told DATA<total>, and a FAIL in the middle of a
 * data stream is not something the protocol can express. The host therefore
 * has to check that it received `total` bytes; `oem armed` reports where the
 * device stopped for the next round trip.
 */
static u32 up_lba, up_sectors, up_done, up_stage, up_err, up_step;
/* Latched inside up_stream() on the failing call: the main loop may run
 * usb_bulk_recover() before the host gets to ask `oem armed`, and that clears
 * usb_bulk_error. up_epch separates "bus reset" from "bulk IN deadline". */
static u32 up_arm_epoch, up_xfer, up_uerr, up_epch;
/* Round 86: timer sanity. dg_ut is the tick delta the payload itself spent
 * inside up_stream(); the host knows that window in wall seconds, so
 * dg_ut / seconds must come out at TIMER0_HZ (4e6). If it comes out at 1e6 the
 * 3882-tick abort wait was never 20 ms and route B was measuring the wrong
 * clock. */
static u32 up_t0, up_t1;
/* P10: split dg_ut into the pieces the host cannot see. Plain tick deltas
 * taken at command boundaries only -- nothing is inserted into read_block()
 * or into the command/data wait loops, so the read path stays identical.
 *   up_tk_arm  cmd_read() -> session open (first chunk only) + table load
 *              (P14: a cache hit after the first piece, so it should read as
 *              the two timer reads and nothing else)
 *   up_tk_open up_stream() -> session open (first chunk only)
 *   up_tk_read sum of storage_read_sectors_live()
 *   up_tk_send sum of usb_bulk_send()
 * P11: once the session is held across commands both open terms drop to the
 * timer reads alone (tens of ticks), which is what the 39.07 ms was. */
static u32 up_tk_arm, up_tk_open, up_tk_read, up_tk_send;

/* "nflasha<dec>" -> 1-based slot number. Returns 1 on success. */
static int parse_slot(const char *s, u32 *slot, const char **end)
{
    u32 v = 0;
    int digits = 0;

    if ((s = str_after(s, "nflasha")) == 0)
        return 0;

    while (*s >= '0' && *s <= '9') {
        v = v * 10 + (u32)(*s - '0');
        if (v > PART_MAX_ENTRIES)
            return 0;
        digits++;
        s++;
    }

    if (digits == 0)
        return 0;

    *slot = v;
    if (end)
        *end = s;
    return 1;
}

/* P14: `oem read` streams now, so it calls the body that still lives next to
 * `upload` further down. */
static void up_stream(void);

static void cmd_read(const char *args)
{
    struct part_table *t = &part_result;
    struct part_entry *e;
    const char *p;
    u32 slot, off, len;
    int ret;

    if (!parse_slot(args, &slot, &p) || *p != ':') {
        fb_fail("bad slot (use oem read:nflasha<N>:<off>)");
        return;
    }

    off = (u32)hex_parse(p + 1, &p);
    len = 0;
    if (*p == ':')
        len = (u32)hex_parse(p + 1, &p);
    if (*p) {
        fb_fail("trailing characters after length");
        return;
    }

    {
        u32 t_arm = timer_ticks();

        ret = STORAGE_OK;
        if (!up_sess) {
            ret = storage_open(&up_id);
            if (ret == STORAGE_OK)
                up_sess = 1;
        }
        /* P14: the table cannot change here, and re-reading its four sectors
         * was 0.72 ms of the 1.12 ms arm term. Nothing in this image writes the
         * card, so one load per session is enough. */
        if (ret == STORAGE_OK && !part_cached)
            ret = part_load_live(t, &up_id);
        if (ret == STORAGE_OK)
            part_cached = 1;
        up_tk_arm = timer_ticks() - t_arm;
    }
    if (ret != PART_OK) {
        up_session_drop();
        if (ret == PART_ERR_MAGIC)
            fb_fail("partition table: bad magic");
        else
            fb_fail("partition table read failed (see err)");
        return;
    }

    if (slot == 0 || slot > t->slots) {
        fb_fail("slot out of range");
        return;
    }

    e = &t->entry[slot - 1];

    if (!(e->flag & PART_FLAG_VALID)) {
        fb_fail("slot not marked valid");
        return;
    }
    if ((e->start | off) & (PART_SECTOR_SIZE - 1)) {
        fb_fail("start/off not 512-aligned");
        return;
    }
    if (off >= e->count) {
        fb_fail("offset beyond partition");
        return;
    }

    if (len == 0)
        len = (e->count - off) & ~(u32)(PART_SECTOR_SIZE - 1);
    if (len == 0) {
        fb_fail("nothing to read");
        return;
    }
    if ((len & (PART_SECTOR_SIZE - 1)) || len > e->count - off) {
        fb_fail("length not 512-aligned or beyond partition");
        return;
    }

    up_lba     = e->start / PART_SECTOR_SIZE + off / PART_SECTOR_SIZE;
    up_sectors = len / PART_SECTOR_SIZE;
    up_done    = 0;
    up_stage   = 0;
    up_err     = 0;
    up_step    = 0;
    up_arm_epoch = usb_bulk_epoch();
    up_xfer      = 0;
    up_uerr      = 0;
    up_epch      = 0;

    /* P14: arm and download in one command. The old `INFO armed ...` reply is
     * gone on purpose -- it was one more bulk IN per piece; `oem armed` still
     * reports arm_lba/arm_n afterwards. */
    up_stream();
}

/* P14: the stream body, shared by `oem read` (arm + download in one command)
 * and by `upload` (re-stream whatever is armed).
 *
 * P10: keep the out-of-line split r97 already had. The tick reads below push
 * fastboot_loop past GCC's growth budget, and one of the things it then does is
 * inline this function -- which buries the whole chunk loop inside the command
 * dispatcher and makes the two disassemblies impossible to compare. */
__attribute__((noinline))
static void up_stream(void)
{
    char line[16];
    u32 total, done = 0, n;
    int ret;

    total = up_sectors * PART_SECTOR_SIZE;

    n = str_append(line, 0, sizeof(line), "DATA");
    n += hex_format(line + n, total, 8);
    usb_bulk_send(line, n);

    up_stage = 0;
    up_done  = 0;
    up_err   = 0;
    up_step  = 0;
    up_t0    = timer_ticks();
    up_t1    = up_t0;
    up_tk_open = 0;
    up_tk_read = 0;
    up_tk_send = 0;

    /* One identification for the whole export. It is ~1 s at 400 kHz and used
     * to be paid by every 16 KiB chunk; the card now stays selected in TRAN
     * and each chunk is a CMD17 loop at the 24 MHz data clock. Read-only. */
    up_stage = 1;
    {
        u32 t_open = timer_ticks();

        ret = STORAGE_OK;
        if (!up_sess) {
            ret = storage_open(&up_id);
            if (ret == STORAGE_OK)
                up_sess = 1;
        }
        up_tk_open = timer_ticks() - t_open;
    }
    if (ret != STORAGE_OK) {
        up_err  = (u32)ret;
        up_step = up_id.step;
        up_t1   = timer_ticks();
        return;                         /* host sees a short transfer */
    }

    while (done < total) {
        u32 chunk = total - done, sent;

        if (chunk > UPLOAD_CHUNK)
            chunk = UPLOAD_CHUNK;

        up_stage = 1;
        {
            u32 t_read = timer_ticks();

            ret = storage_read_sectors_live(up_lba + done / PART_SECTOR_SIZE,
                                            download_buf, chunk / PART_SECTOR_SIZE,
                                            &up_id);
            up_tk_read += timer_ticks() - t_read;
        }
        if (ret != STORAGE_OK) {
            up_err  = (u32)ret;
            up_step = up_id.step;
            up_t1   = timer_ticks();
            up_session_drop();
            return;                     /* host sees a short transfer */
        }

        up_stage = 2;
        {
            u32 t_send = timer_ticks();

            sent = usb_bulk_send(download_buf, chunk);
            up_tk_send += timer_ticks() - t_send;
        }
        if (sent != chunk) {
            up_xfer = sent;
            up_uerr = (u32)usb_bulk_error;
            up_epch = usb_bulk_epoch() - up_arm_epoch;
            up_err  = 0x100u;           /* bulk IN timeout, reset or short */
            up_step = done / PART_SECTOR_SIZE;
            up_t1   = timer_ticks();
            up_session_drop();
            return;                     /* host sees a short transfer */
        }

        done += chunk;
        up_done = done;
    }

    up_stage = 3;
    up_t1    = timer_ticks();
    /* P11: the session deliberately stays open. The next `oem read` streams
     * through it and finds the table already loaded (P14), so the card is
     * identified once per export rather than once per chunk pair. */
    fb_okay("");
}

/* `upload` on its own: re-stream what is armed. P14 keeps it for the host's
 * `upload` verb and for a post-mortem repeat of one piece; the export path goes
 * straight through cmd_read(). */
__attribute__((noinline))
static void cmd_upload(void)
{
    if (up_sectors == 0) {
        fb_fail("nothing armed (use oem read:nflasha<N>:<off>)");
        return;
    }
    up_stream();
}

/* `oem armed` -- why did the last upload stop? Separate round trip because a
 * truncated stream cannot carry the reason itself. */
static void cmd_armed(void)
{
    if (up_sectors == 0) {
        fb_fail("nothing armed");
        return;
    }

    info_hex("arm_lba", up_lba, 8);
    info_hex("arm_n", up_sectors, 8);
    info_hex("up_done", up_done, 8);
    info_hex("up_stage", up_stage, 1);
    info_hex("up_err", up_err, 3);
    info_hex("up_step", up_step, 8);
    /* Three-way split of up_err=0x100:
     *   uerr=0x0D, epch=0  -> bulk IN deadline expired
     *   uerr=0,    epch>0  -> bus reset cancelled the transfer
     *   uerr=0,    epch=0  -> short send with no error code
     */
    info_hex("up_xfer", up_xfer, 8);
    info_hex("up_uerr", up_uerr, 2);
    info_hex("up_epch", up_epch, 8);
    info_hex("up_sess", (u32)up_sess, 1);  /* P11: 1 = session held open */

    /*
     * Round 73: the deadline snapshot. Only meaningful when a bulk IN landed
     * on the deadline; after a clean upload these keep the values of the last
     * failure, which is exactly what the next post-mortem wants.
     */
    {
        const struct usb_bulk_diag *dg = usb_bulk_diag();

        info_hex("dg_hwo", dg->hwo, 1);     /* 1 = TRB never fetched */
        info_hex("dg_tsz", dg->tsz, 8);     /* == len when it never started */
        info_hex("dg_cmd", dg->cmd, 4);     /* bit10 CMDACT + status */
        info_hex("dg_gev", dg->gev, 4);     /* pending event bytes */
        info_hex("dg_evt", dg->evt, 4);     /* EP_IN events during the send */
        info_hex("dg_typ", dg->typ, 2);     /* 1 = complete, 2 = in progress */
        info_hex("dg_late", dg->late, 8);   /* ticks past the deadline, 0 = never */
        /* Round 82: did ENDTRANSFER actually free the endpoint? */
        info_hex("dg_xst", dg->end_st, 1);  /* 0 = accepted, else DEPCMD status */
        info_hex("dg_xhwo", dg->end_hwo, 1);/* 1 = TRB still controller-owned */
        /* hex_format() prints only the LOW `digits` nibbles. Width 4 truncated
         * 0x13888 (80008 ticks) to "3888" in rounds 85/86 and made a full 20 ms
         * wait look like an early exit. Timer values get 8 digits, always. */
        info_hex("dg_xwt", dg->end_wt, 8);  /* ticks waited (limit = hung) */
        /* Round 86: how the wait ended. xclr=0 means HWO never read zero twice
         * (the 3882 exit was a single sample); xre!=0 means the released TRB was
         * re-armed again, i.e. a stale transfer was still queued. */
        info_hex("dg_xclr", dg->end_clr, 8);
        info_hex("dg_xre", dg->end_re, 8);
        /* Round 86: payload-side duration of the last upload, in timer ticks.
         * Host: compare with the wall seconds of that dump -- the ratio must be
         * TIMER0_HZ (4e6); 1e6 would mean the 4 MHz assumption is wrong. */
        info_hex("dg_ut", up_t1 - up_t0, 8);
        /* P10: where those ticks went. arm is cmd_read()'s identification and
         * is NOT part of dg_ut; open/read/send are. Everything the host pays
         * outside these four numbers is its own USB round trips. */
        info_hex("up_tk_arm", up_tk_arm, 8);
        info_hex("up_tk_open", up_tk_open, 8);
        info_hex("up_tk_read", up_tk_read, 8);
        info_hex("up_tk_send", up_tk_send, 8);
        /* P13: CMD18 is live again, so these count the sectors the multi-block
         * path actually moved (one CMD18 per 16 KiB piece) and the first error
         * it hit before falling back to CMD17. */
        info_hex("up_multi", up_id.multi_sectors, 8);
        info_hex("up_merr", up_id.multi_err, 2);
        /* P12: data-phase divider calibration. up_c4/c2/c1 are the one-block
         * read time at div 4/2/1 in timer ticks (0 = that divider failed);
         * up_cdiv is the divider the session ended up using. */
        info_hex("up_cdiv", up_id.cal_sel, 1);
        info_hex("up_c4", up_id.cal_ticks[0], 8);
        info_hex("up_c2", up_id.cal_ticks[1], 8);
        info_hex("up_c1", up_id.cal_ticks[2], 8);
    }

    if (up_err == 0)
        fb_okay("");
    else
        fb_fail("upload failed (see up_err/up_step)");
}

/* ---- oem darwin --------------------------------------------------------- */

/*
 * `oem darwin` -- one read-only conversation with the system power MCU over
 * SIO channel 3, then report the Virtual WDT rows. This is the community
 * command: it observes, it never writes Darwin memory, and there is no
 * command 0x12 anywhere in the image.
 *
 * It was also the experiment for "does a periodic exchange keep the SoC from
 * resetting after ~244 s". Measured 2026-10-27: no. One payload instance
 * stayed up 51 min across 20 s `info` polling, 31 s of 40 back-to-back probes
 * and a 605 s fully idle window, with only four `darwin` runs in between, so
 * this command is an observation tool, not a keepalive.
 *
 *   oem darwin                     link state + VWDT counters/reloads
 *   oem darwin pins                only the GPIO half of darwin_init()
 *   oem darwin sio                 only the SIO half, then read it back
 *   oem darwin peek:<addr>[:<len>] raw Darwin read (command 0x16)
 */
static u32 darwin_seq;
static int darwin_ready;

static void darwin_show_bytes(const char *what, const u8 *b, u32 n)
{
    char line[FB_PEEK_LINE];
    u32 i, k = 0;

    k = str_append(line, k, sizeof(line), what);
    for (i = 0; i < n; i++) {
        line[k++] = ' ';
        k += hex_format(line + k, b[i], 2);
    }
    line[k] = 0;
    fb_reply("INFO", line);
}

static void cmd_darwin_state(void)
{
    u8 counters[4], reloads[3];
    int ret;

    if (!darwin_ready) {
        darwin_init();
        darwin_ready = 1;
    }

    /*
     * Printed before the first read, so a hang inside it still leaves a trace.
     * `darwin_seq` lives in .bss (starts at zero) and is only bumped here, so
     * it is the one free instrument for "did the payload reset": while it keeps
     * counting up, the instance never restarted.
     */
    darwin_seq++;
    info_hex("dw_seq", darwin_seq, 8);

    ret = darwin_read(VWDT_TABLE, counters, 4);
    if (ret != 0) {
        fb_fail("no reply from darwin (table)");
        return;
    }
    darwin_show_bytes("counters @200026fc:", counters, 4);

    ret = darwin_read(VWDT_RELOAD, reloads, 3);
    if (ret != 0) {
        fb_fail("no reply from darwin (reload)");
        return;
    }
    darwin_show_bytes("reloads  @2000003c:", reloads, 3);

    fb_okay("");
}

static void cmd_darwin_peek(const char *args)
{
    const char *p;
    u32 addr, len, off;
    u8 buf[PKT_MAX_PAYLOAD];

    if (!darwin_ready) {
        darwin_init();
        darwin_ready = 1;
    }

    addr = (u32)hex_parse(args, &p);
    if (p == args) {
        fb_fail("usage: oem darwin peek:<addr>[:<len>]");
        return;
    }
    len = 16;
    if (*p == ':')
        len = (u32)hex_parse(p + 1, 0);
    if (len == 0 || len > sizeof(buf)) {
        fb_fail("bad length (max 0x78)");
        return;
    }

    /* Echo the request first: a hang inside the read leaves nothing else. */
    info_hex("dw_req", addr, 8);
    info_hex("dw_len", len, 2);

    if (darwin_read(addr, buf, len) != 0) {
        fb_fail("no reply from darwin (link down?)");
        return;
    }

    for (off = 0; off < len; off += 8) {
        char line[FB_PEEK_LINE];
        u32 i, k = 0;

        k += hex_format(line + k, addr + off, 8);
        line[k++] = ':';
        for (i = 0; i < 8 && off + i < len; i++) {
            line[k++] = ' ';
            k += hex_format(line + k, buf[off + i], 2);
        }
        line[k] = 0;
        fb_reply("INFO", line);
    }
    fb_okay("");
}

/* Bisect helpers: which half of darwin_init() takes the link down. */
static void cmd_darwin_pins(void)
{
    darwin_pins();
    fb_okay("");
}

static void cmd_darwin_sio(void)
{
    u32 v[8];

    darwin_sio_idle();
    darwin_sio_regs(v);

    info_hex("sio_ctrl", v[0], 8);
    info_hex("sio_ctrl2", v[1], 8);
    info_hex("sio_reg08", v[2], 8);
    info_hex("sio_len", v[3], 8);
    info_hex("sio_cfg", v[4], 8);
    info_hex("sio_rate_lo", v[5], 8);
    info_hex("sio_rate_hi", v[6], 8);
    info_hex("sio_data0", v[7], 8);
    fb_okay("");
}

static void cmd_darwin(const char *args)
{
    const char *rest;

    /* The dispatcher hands over the remainder with its leading space. */
    while (*args == ' ')
        args++;

    if (str_eq(args, "")) {
        cmd_darwin_state();
        return;
    }
    if (str_eq(args, "pins")) {
        cmd_darwin_pins();
        return;
    }
    if (str_eq(args, "sio")) {
        cmd_darwin_sio();
        return;
    }
    if ((rest = str_after(args, "peek:")) != 0) {
        cmd_darwin_peek(rest);
        return;
    }
    fb_fail("usage: oem darwin [pins|sio|peek:a[:n]]");
}

/* ---- reboot ------------------------------------------------------------- */

/*
 * There is no known SoC reset register for the CXD90045, so this cannot
 * reboot. It answers the host, drops the pullup so the host sees a clean
 * unplug, and then parks with a distinct light code.
 */
static void cmd_reboot(void)
{
    up_session_drop();    /* P11: put HOST_CONTROL / CLOCK_CONTROL back */

    fb_okay("");

    mdelay(50);
    dwc3_disconnect();
    mdelay(10);

    led_code(0x20);
}

/* ---- command dispatch --------------------------------------------------- */

static void fastboot_command(const char *cmd)
{
    const char *rest;

    if ((rest = str_after(cmd, "getvar:")) != 0) {
        cmd_getvar(rest);
    } else if ((rest = str_after(cmd, "download:")) != 0) {
        cmd_download(rest);
    } else if ((rest = str_after(cmd, "flash:")) != 0) {
        cmd_flash(rest);
    } else if ((rest = str_after(cmd, "oem peek:")) != 0) {
        cmd_peek(rest);
    } else if (str_eq(cmd, "oem storage")) {
        cmd_storage();
    } else if (str_eq(cmd, "oem partition")) {
        cmd_partition();
    } else if ((rest = str_after(cmd, "oem read:")) != 0) {
        cmd_read(rest);
    } else if (str_eq(cmd, "oem armed")) {
        cmd_armed();
    } else if ((rest = str_after(cmd, "oem darwin")) != 0) {
        cmd_darwin(rest);
    } else if (str_after(cmd, "oem ") != 0) {
        /* Only the read-only half exists; say so instead of silently failing. */
        fb_fail("unknown oem command (peek, storage, partition, read, armed, darwin)");
    } else if (str_eq(cmd, "reboot")) {
        cmd_reboot();
    } else if (str_after(cmd, "reboot-") != 0) {
        /* reboot-bootloader and friends name a target we cannot deliver.
         * Fail rather than quietly doing a plain reboot. */
        fb_fail("only plain `reboot` is supported");
    } else if (str_eq(cmd, "upload")) {
        cmd_upload();
    } else {
        fb_fail("unknown command");
    }
}

void fastboot_loop(void)
{
    for (;;) {
        u32 n;

        /* Deferred from SET_CONFIGURATION; safe to do here. */
        usb_bulk_enable_pending();

        /* A host that stopped reading mid-reply is recoverable. Anything else
         * keeps pumping events rather than stopping, since a loop that does
         * not takes ep0 down with it. */
        if (usb_bulk_error && usb_bulk_recover() != 0) {
            usb_event_pump();
            continue;
        }

        n = usb_bulk_recv(cmd_buf, FB_COMMAND_MAX);

        if (n == 0)
            continue;
        if (n >= sizeof(cmd_buf))
            n = sizeof(cmd_buf) - 1;
        cmd_buf[n] = 0;

        fastboot_command((const char *)cmd_buf);
    }
}
