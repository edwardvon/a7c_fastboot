#ifndef A7C_PART_H
#define A7C_PART_H

#include "io.h"
#include "storage.h"

/*
 * SDM2 partition table, read from the user area at block address 0.
 *
 * Layout (ZV-E10 measurement of the same format; no A7C dump exists yet):
 *
 *   offset 0..3     magic, either "6428" (the ZV-E10 sample starts with
 *                   "642800.1") or "8246" (what the ILCE-7M4 port checks).
 *                   Both are accepted because the A7C has not been read.
 *   offset 8        entry count, little endian (0x1e = 30 on the ZV-E10)
 *   offset 12       header flag word (1 on the ZV-E10)
 *   offset 32+i*16  entry i: start, count, type, flag, little endian
 *
 * `start` and `count` are BYTE offsets, not sectors. The A7C partinf.conf
 * confirms this: nflasha1 is 0x10000 + 0x7F0000 = 0x800000, exactly where
 * nflasha2 begins. Read as sectors the two would overlap by 512x, which the
 * flash layout cannot do.
 *
 * The 7M4 port feeds le32(entry) straight into mmc_read_blocks() as a block
 * address; that is a 512x error on its side. Anything here that needs to read
 * a partition converts with `start / PART_SECTOR_SIZE`, and `start & 511`
 * must be zero or the slot does not describe a readable range.
 */

#define PART_SECTOR_SIZE     512
#define PART_TABLE_SECTORS   4
#define PART_MAX_ENTRIES     ((PART_TABLE_SECTORS * PART_SECTOR_SIZE - 32) / 16)

#define PART_MAGIC_6428      0x38323436u   /* '6','4','2','8' */
#define PART_MAGIC_8246      0x36343238u   /* '8','2','4','6' */
#define PART_FLAG_VALID      1u

#define PART_OK              0
#define PART_ERR_MAGIC       8   /* first four bytes are neither magic */
#define PART_ERR_SHORT       9   /* fewer than 32 bytes to hold a header */

struct part_entry {
    u32 start;      /* byte offset into the card, not a sector number */
    u32 count;      /* size in bytes */
    u32 type;
    u32 flag;       /* bit 0 = sdM2 label valid */
};

struct part_table {
    u32 magic;      /* entry 0..3 as a little-endian word */
    u32 count;      /* entry count claimed by the header */
    u32 slots;      /* entries actually parsed: count, clamped to the buffer */
    u32 valid;      /* slots with PART_FLAG_VALID */
    u32 extent;     /* max(start + count) over valid slots, in bytes */
    u32 hdr_flags;  /* word at offset 12 */
    struct part_entry entry[PART_MAX_ENTRIES];
};

/* Pure parser: no hardware, no globals. Returns PART_OK / PART_ERR_*. */
int part_parse(const void *raw, u32 len, struct part_table *out);

/* Read the four table sectors from block address 0, then parse them.
 * Returns PART_OK, PART_ERR_*, or a STORAGE_ERR_*; `id` reports the read. */
int part_load(struct part_table *out, struct storage_id *id);

/* Same table read, but on a session the caller already opened with
 * storage_open(). Does not identify the card and does not close it (P11). */
int part_load_live(struct part_table *out, struct storage_id *id);

#endif /* A7C_PART_H */
