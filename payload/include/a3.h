#ifndef A7C_A3_H
#define A7C_A3_H

#include "io.h"

/*
 * A3 container: a 32-byte header followed by 32 bytes of guard and then the
 * flat payload, all living at the BootROM load address.  _start sits at
 * entry_off (64), not directly behind the header, so a stray push below the
 * entry point cannot reach the header's CRC word.
 *
 *   0x00  magic        'A3C1'
 *   0x04  version      1
 *   0x08  header_size  32
 *   0x0c  load_addr    0xFE020000
 *   0x10  image_size   total bytes, header included
 *   0x14  entry_off    64
 *   0x18  payload_crc  CRC-32 over [header_size, image_size)
 *   0x1c  header_crc   CRC-32 over [0, 0x1c)
 */
#define A3_MAGIC        0x31433341u   /* 'A3C1' little-endian */
#define A3_VERSION      1u
#define A3_HDR_SIZE     32u
#define A3_GUARD_SIZE   32u
#define A3_ENTRY_OFF    (A3_HDR_SIZE + A3_GUARD_SIZE)
#define A3_LOAD_ADDR    0xFE020000u
#define A3_MAX_IMAGE    (64u * 1024u)

typedef struct {
    u32 magic;
    u32 version;
    u32 hdr_size;
    u32 load_addr;
    u32 image_size;
    u32 entry_off;
    u32 payload_crc;
    u32 hdr_crc;
} a3_header_t;

/* 0 = good; 1 magic, 2 header fields, 3 header CRC, 4 payload CRC, 5 size. */
int a3_verify(void);

#endif /* A7C_A3_H */
