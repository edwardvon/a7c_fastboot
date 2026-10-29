#!/usr/bin/env python3
"""Wrap a flat ARM payload in the 32-byte A3 container.

The loader expects the header at offset 0 and the first instruction at
offset 64 (see payload.ld's ASSERT and src/a3.c's a3_verify): the 32 bytes
behind the header are a guard, because the BootROM hands over with the SP of
its own loader and a push below the entry point would otherwise overwrite the
header CRC word at +0x1c. Both CRCs use zlib's polynomial, which is what the
on-target bitwise implementation computes.

    python tools/a3image.py out/payload.bin -o out/payload.a3 --verify
"""

import argparse
import struct
import sys
import zlib
from pathlib import Path

A3_MAGIC = 0x31433341          # 'A3C1', little endian
A3_VERSION = 1
A3_HDR_SIZE = 32
A3_GUARD_SIZE = 32             # kept zero; entry point sits behind it
A3_ENTRY_OFF = A3_HDR_SIZE + A3_GUARD_SIZE
A3_LOAD_ADDR = 0xFE020000
A3_MAX_IMAGE = 64 * 1024

HDR_FIELDS = "<7I"             # magic .. payload_crc, everything but hdr_crc


def _crc(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def build(image: bytes) -> bytes:
    if len(image) < A3_HDR_SIZE + 2:
        raise SystemExit(f"image is only {len(image)} B, nothing to wrap")
    if len(image) > A3_MAX_IMAGE:
        raise SystemExit(f"image is {len(image)} B, over the 64K ESRAM window")
    if image[:A3_HDR_SIZE] != b"\x00" * A3_HDR_SIZE:
        raise SystemExit("first 32 bytes are not zero: already wrapped?")

    payload = image[A3_HDR_SIZE:]
    head = struct.pack(
        HDR_FIELDS,
        A3_MAGIC,
        A3_VERSION,
        A3_HDR_SIZE,
        A3_LOAD_ADDR,
        len(image),            # image_size includes the header and the guard
        A3_ENTRY_OFF,          # _start sits behind the guard
        _crc(payload),
    )
    return head + struct.pack("<I", _crc(head)) + payload


def verify(image: bytes) -> None:
    if len(image) < A3_HDR_SIZE:
        raise SystemExit("image shorter than the header")

    (magic, version, hdr_size, load_addr, image_size,
     entry_off, payload_crc, hdr_crc) = struct.unpack_from("<8I", image, 0)

    if magic != A3_MAGIC:
        raise SystemExit(f"bad magic 0x{magic:08X}")
    if version != A3_VERSION or hdr_size != A3_HDR_SIZE:
        raise SystemExit(f"bad version/header size {version}/{hdr_size}")
    if load_addr != A3_LOAD_ADDR:
        raise SystemExit(f"bad load address 0x{load_addr:08X}")
    if entry_off != A3_ENTRY_OFF:
        raise SystemExit(f"bad entry offset {entry_off}, expected {A3_ENTRY_OFF}")
    if not A3_HDR_SIZE < image_size <= A3_MAX_IMAGE:
        raise SystemExit(f"bad image size {image_size}")
    if image_size != len(image):
        raise SystemExit(f"header says {image_size} B, file is {len(image)} B")
    if _crc(image[:A3_HDR_SIZE - 4]) != hdr_crc:
        raise SystemExit("header CRC mismatch")
    if _crc(image[A3_HDR_SIZE:image_size]) != payload_crc:
        raise SystemExit("payload CRC mismatch")


def main() -> int:
    ap = argparse.ArgumentParser(description="wrap a flat payload as .a3")
    ap.add_argument("image", help="flat binary produced by objcopy -O binary")
    ap.add_argument("-o", "--output", help="output path (default: <image>.a3)")
    ap.add_argument("--verify", action="store_true",
                    help="re-read the output and check both CRCs")
    args = ap.parse_args()

    src = Path(args.image)
    data = src.read_bytes()
    wrapped = build(data)

    dst = Path(args.output) if args.output else src.with_suffix(".a3")
    dst.write_bytes(wrapped)

    print(f"[a3] {src.name}: {len(data)} B -> {dst.name}: {len(wrapped)} B "
          f"(payload crc 0x{struct.unpack_from('<I', wrapped, 24)[0]:08X})")

    if args.verify:
        verify(dst.read_bytes())
        print(f"[a3] verify ok, entry 0x{A3_LOAD_ADDR + A3_ENTRY_OFF:08X} "
              f"(thumb: 0x{A3_LOAD_ADDR + A3_ENTRY_OFF + 1:08X})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
