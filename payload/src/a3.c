#include "a3.h"

extern const u8 _a3_hdr[];

/*
 * CRC-32/ISO-HDLC, bitwise so it matches Python's zlib.crc32() without a
 * 1 KB table in a 64 KB image.
 */
static u32 crc32(const u8 *p, u32 len)
{
    u32 c = 0xFFFFFFFFu;

    while (len--) {
        u32 i;

        c ^= *p++;
        for (i = 0; i < 8; i++)
            c = (c >> 1) ^ (0xEDB88320u & (u32)-(int)(c & 1u));
    }
    return c ^ 0xFFFFFFFFu;
}

int a3_verify(void)
{
    const a3_header_t *h = (const a3_header_t *)_a3_hdr;

    if (h->magic != A3_MAGIC)
        return 1;
    if (h->version != A3_VERSION || h->hdr_size != A3_HDR_SIZE ||
        h->load_addr != A3_LOAD_ADDR || h->entry_off != A3_ENTRY_OFF)
        return 2;
    if (h->image_size <= A3_HDR_SIZE || h->image_size > A3_MAX_IMAGE)
        return 5;
    if (crc32((const u8 *)h, A3_HDR_SIZE - 4) != h->hdr_crc)
        return 3;
    if (crc32(_a3_hdr + A3_HDR_SIZE, h->image_size - A3_HDR_SIZE) != h->payload_crc)
        return 4;
    return 0;
}
