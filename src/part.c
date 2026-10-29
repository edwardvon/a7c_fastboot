#include "part.h"
#include "io.h"
#include "storage.h"

static u32 le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) |
           ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

/* The four table sectors. 4-byte aligned because the read path moves whole
 * words out of the SDHCI buffer. */
static u32 pt_raw[PART_TABLE_SECTORS * PART_SECTOR_SIZE / sizeof(u32)]
    __attribute__((aligned(4)));

static void part_table_clear(struct part_table *t)
{
    u32 i;

    for (i = 0; i < sizeof(*t) / sizeof(u32); i++)
        ((volatile u32 *)t)[i] = 0;
}

int part_parse(const void *raw, u32 len, struct part_table *out)
{
    const u8 *p = (const u8 *)raw;
    u32 i, n;

    part_table_clear(out);

    if (len < 32)
        return PART_ERR_SHORT;

    out->magic = le32(p);
    if (out->magic != PART_MAGIC_6428 && out->magic != PART_MAGIC_8246)
        return PART_ERR_MAGIC;

    out->count     = le32(p + 8);
    out->hdr_flags = le32(p + 12);

    n = out->count;
    if (n > PART_MAX_ENTRIES)
        n = PART_MAX_ENTRIES;             /* header claims more than fits */
    if (n > (len - 32) / 16)
        n = (len - 32) / 16;              /* short image */

    out->slots = n;
    for (i = 0; i < n; i++) {
        const u8 *e = p + 32 + i * 16;
        u32 end;

        out->entry[i].start = le32(e + 0);
        out->entry[i].count = le32(e + 4);
        out->entry[i].type  = le32(e + 8);
        out->entry[i].flag  = le32(e + 12);

        if (!(out->entry[i].flag & PART_FLAG_VALID))
            continue;
        out->valid++;
        end = out->entry[i].start + out->entry[i].count;
        if (end > out->extent)
            out->extent = end;
    }

    return PART_OK;
}

int part_load(struct part_table *out, struct storage_id *id)
{
    int ret;

    part_table_clear(out);

    ret = storage_read_sectors(0, pt_raw, PART_TABLE_SECTORS, id);
    if (ret != STORAGE_OK)
        return ret;

    dsb();
    return part_parse(pt_raw, sizeof(pt_raw), out);
}

/* P11: the card is already selected in TRAN and running at the data clock, so
 * the four table sectors are just four CMD17s. No identification, no close. */
int part_load_live(struct part_table *out, struct storage_id *id)
{
    int ret;

    part_table_clear(out);

    ret = storage_read_sectors_live(0, pt_raw, PART_TABLE_SECTORS, id);
    if (ret != STORAGE_OK)
        return ret;

    dsb();
    return part_parse(pt_raw, sizeof(pt_raw), out);
}
