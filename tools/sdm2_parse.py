#!/usr/bin/env python3
"""Offline side of the A7C SDM2 partition table (O-3).

The payload command `oem partition` reads the four 512-byte table sectors at
block address 0 and prints one INFO line per slot.  This script turns the
firmware's `partinf.conf` into the table we expect to see, parses a dump or a
captured log, and diffs the two so M-4 is a one-line comparison instead of an
eyeball exercise.

    python sdm2_parse.py expected <partinf.conf> [--csv FILE]
    python sdm2_parse.py capture --raw <dump.bin> [--csv FILE]
    python sdm2_parse.py capture --log <info.txt> [--csv FILE]
    python sdm2_parse.py compare <partinf.conf> (--log <info.txt> | --raw <dump.bin>)
    python sdm2_parse.py selftest

Table format (ZV-E10 sample plus the A7C's `oem partition` log):

    offset 0..3     magic, "6428" (ZV-E10 sample starts "642800.1") or
                    "8246" (what the ILCE-7M4 port checks, and what the A7C
                    reports) -- both accepted.  The payload prints this field
                    as the LE32 integer, so "8246" shows up as 36343238.
    offset 8        LE32 entry count
    offset 12       LE32 header flags (1 on the ZV-E10)
    offset 32+i*16  LE32 start, count, type, flag

`start` and `count` are BYTE offsets, not sectors.  The A7C partinf.conf
settles it: nflasha1 at 0x10000 with size 0x7F0000 ends exactly at 0x800000,
where nflasha2 starts.  Read as sectors the two would overlap by 512x.  The
ILCE-7M4 port passes the byte value straight to mmc_read_blocks() as a block
address; that is a bug on its side, and this script prints `start_lba` so the
conversion is explicit.

Still for `oem partition` to be run on the camera.
"""
import argparse
import csv
import io
import re
import sys
from pathlib import Path

SECTOR = 512
TABLE_SECTORS = 4
TABLE_BYTES = SECTOR * TABLE_SECTORS
MAX_ENTRIES = (TABLE_BYTES - 32) // 16          # 126
MAGICS = (b'6428', b'8246')
VERSION_TAIL = b'00.1'
FLAG_VALID = 1

COLUMNS = ('slot', 'name', 'start', 'count', 'type', 'flag', 'valid',
           'start_lba', 'end', 'note')

CONF_LINE = re.compile(
    r'^\s*0x([0-9A-Fa-f]+)\s*,\s*0x([0-9A-Fa-f]+)\s*,'
    r'\s*0x([0-9A-Fa-f]+)\s*,\s*0x([0-9A-Fa-f]+)\s*$')
CONF_NAME = re.compile(r'/dev/(nflasha\d+)')
HDR_LINE = re.compile(r'sdm2\s+magic=([0-9A-Fa-f]+)\s+count=([0-9A-Fa-f]+)'
                      r'\s+valid=([0-9A-Fa-f]+)\s+extent=([0-9A-Fa-f]+)')


def magic_text(magic):
    """The four magic bytes as they sit in the table: '6428' or '8246'."""
    return magic.decode('latin-1')


def magic_hex(magic):
    """What `oem partition` prints: the same four bytes as an LE32 in %08x.

    '8246' -> '36343238'.  Reading that hex string as big-endian ASCII yields
    '6428', which is why the two sides used to look like different magics.
    """
    return '%08x' % int.from_bytes(magic, 'little')


def parse_magic(token):
    """Accept both renderings of the magic: '8246' or '36343238'."""
    if len(token) == 4 and token.encode('latin-1') in MAGICS:
        return token.encode('latin-1')
    if len(token) == 8:
        magic = int(token, 16).to_bytes(4, 'little')
        if magic in MAGICS:
            return magic
    raise SystemExit('unrecognised magic %r (expected %s or their LE32 hex)'
                     % (token, ' / '.join(m.decode() for m in MAGICS)))


class Row:
    __slots__ = ('slot', 'name', 'start', 'count', 'type', 'flag', 'note')

    def __init__(self, slot, name, start, count, type_, flag, note=''):
        self.slot = slot
        self.name = name
        self.start = start
        self.count = count
        self.type = type_
        self.flag = flag
        self.note = note

    @property
    def valid(self):
        return 1 if self.flag & FLAG_VALID else 0

    def fields(self):
        return (self.slot, self.name, self.start, self.count, self.type,
                self.flag, self.valid, self.start // SECTOR,
                self.start + self.count, self.note)

    def key(self):
        return (self.start, self.count, self.type, self.flag)


# ---- partinf.conf ---------------------------------------------------------

def parse_partinf(path):
    """Return (rows, notes).  Trailing zero-size slots are dropped.

    The A7C file lists 30 slots; 24..30 all have size 0, so the live table is
    23 slots -- which is what the plan expects the SDM2 count field to say.
    """
    rows = []
    note = ''
    text = Path(path).read_text(encoding='utf-8', errors='replace')
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith('#'):
            name = CONF_NAME.search(stripped)
            if name:
                note = name.group(1)
                # "7.9375MB(System Group  SYSTEM (Updater)  /dev/nflasha1  )"
                rest = stripped[name.end():].strip()
                cut = rest.rfind('/dev/')
                if cut > 0:
                    note = rest[:cut].rstrip(' )') or note
            continue
        match = CONF_LINE.match(stripped)
        if not match:
            raise SystemExit('%s: cannot parse line: %s' % (path, stripped))
        start, count, type_, flag = (int(v, 16) for v in match.groups())
        slot = len(rows) + 1
        rows.append(Row(slot, 'nflasha%d' % slot, start, count, type_, flag,
                        note))

    while rows and rows[-1].count == 0:
        rows.pop()
    return rows


# ---- raw table ------------------------------------------------------------

def parse_raw(data):
    """Mirror of part_parse() in src/part.c."""
    if len(data) < 32:
        raise SystemExit('raw dump is %d bytes, need at least 32' % len(data))
    magic = data[0:4]
    if magic not in MAGICS:
        raise SystemExit('bad magic %r (expected %s)'
                         % (magic, ' or '.join(m.decode() for m in MAGICS)))
    count = int.from_bytes(data[8:12], 'little')
    hdr_flags = int.from_bytes(data[12:16], 'little')

    slots = min(count, MAX_ENTRIES, (len(data) - 32) // 16)
    rows = []
    for i in range(slots):
        e = data[32 + i * 16:32 + (i + 1) * 16]
        rows.append(Row(i + 1, 'nflasha%d' % (i + 1),
                        int.from_bytes(e[0:4], 'little'),
                        int.from_bytes(e[4:8], 'little'),
                        int.from_bytes(e[8:12], 'little'),
                        int.from_bytes(e[12:16], 'little')))
    return table_from_rows(rows, count=count, hdr_flags=hdr_flags, magic=magic)


def table_from_rows(rows, count=None, hdr_flags=1, magic=MAGICS[0]):
    if count is None:
        count = len(rows)
    valid = sum(r.valid for r in rows)
    extent = max((r.start + r.count for r in rows if r.valid), default=0)
    return {
        'magic': magic,
        'count': count,
        'hdr_flags': hdr_flags,
        'slots': len(rows),
        'valid': valid,
        'extent': extent,
        'rows': rows,
    }


def build_raw(rows, magic=MAGICS[0]):
    """The inverse of parse_raw(): a 2048-byte table image.

    Used by selftest and by `--write-raw`, so the payload side can be exercised
    against the same bytes without a camera.
    """
    if len(rows) > MAX_ENTRIES:
        raise SystemExit('%d entries do not fit in %d' % (len(rows), MAX_ENTRIES))
    buf = bytearray(TABLE_BYTES)
    buf[0:4] = magic
    buf[4:8] = VERSION_TAIL
    buf[8:12] = len(rows).to_bytes(4, 'little')
    buf[12:16] = (1).to_bytes(4, 'little')
    for i, row in enumerate(rows):
        off = 32 + i * 16
        buf[off:off + 4] = row.start.to_bytes(4, 'little')
        buf[off + 4:off + 8] = row.count.to_bytes(4, 'little')
        buf[off + 8:off + 12] = row.type.to_bytes(4, 'little')
        buf[off + 12:off + 16] = row.flag.to_bytes(4, 'little')
    return bytes(buf)


# ---- the payload's INFO lines --------------------------------------------

def emit_log(table):
    """Render exactly what `oem partition` prints, tag stripped.

    Byte-identical to the header + part_line() of cmd_partition() in
    src/fastboot.c (lower-case hex, 11-char name field).  The payload also
    emits `step=`/`err=` lines; parse_log() ignores them, so they are omitted.

    The header magic goes out as magic_hex(), because fastboot.c prints
    `t->magic` -- the LE32 value -- not the four characters.
    """
    out = ['sdm2 magic=%s count=%04x valid=%02x extent=%08x'
           % (magic_hex(table['magic']), table['count'],
              table['valid'], table['extent'])]
    for row in table['rows']:
        name = row.name
        if len(name) < 11:
            name = name.ljust(11)
        out.append('%s%08x %08x %08x %08x'
                   % (name, row.start, row.count, row.type, row.flag))
    return out


def parse_log(lines):
    header = None
    rows = []
    for raw in lines:
        line = raw.rstrip('\r\n')
        if not line.strip():
            continue
        if line.startswith('INFO') and not line.startswith('INFO '):
            line = line[4:]
        elif line.startswith('INFO '):
            line = line[5:]
        if not line:
            continue
        match = HDR_LINE.search(line)
        if match:
            header = {
                'magic': parse_magic(match.group(1)),
                'count': int(match.group(2), 16),
                'valid': int(match.group(3), 16),
                'extent': int(match.group(4), 16),
            }
            continue
        if not line.startswith('nflasha'):
            continue
        parts = line.split()
        if len(parts) < 5:
            raise SystemExit('short slot line: %r' % raw)
        rows.append(Row(len(rows) + 1, parts[0],
                        int(parts[1], 16), int(parts[2], 16),
                        int(parts[3], 16), int(parts[4], 16)))
    if header is None:
        raise SystemExit('no "sdm2 magic=..." header line in the log')
    table = table_from_rows(rows, count=header['count'],
                            magic=header['magic'])
    table['hdr_valid'] = header['valid']
    table['hdr_extent'] = header['extent']
    return table


# ---- output ---------------------------------------------------------------

def write_csv(table, stream):
    writer = csv.writer(stream, lineterminator='\n')
    writer.writerow(COLUMNS)
    for row in table['rows']:
        writer.writerow(row.fields())


def print_summary(label, table):
    print('%s: magic=%s (le32 %s) count=%d slots=%d valid=%d extent=0x%08x'
          % (label, magic_text(table['magic']), magic_hex(table['magic']),
             table['count'], table['slots'], table['valid'], table['extent']))


# ---- compare --------------------------------------------------------------

def live_rows(rows):
    """Drop trailing zero-size slots, the same rule parse_partinf() applies.

    The A7C's table stores count=30: partinf.conf lists 30 slots and slots
    24..30 are zero-size, so 23 are live.  Comparing the stored count against
    the 23 live partinf rows would report a difference that is not one.
    """
    out = list(rows)
    while out and out[-1].count == 0:
        out.pop()
    return out


def compare(expected_rows, captured):
    """Return (expected_table, diffs, notes).

    `notes` are observations that are not differences: a stored count that
    only covers trailing zero-size slots, or a magic that differs between the
    two accepted values.
    """
    diffs = []
    notes = []
    exp = table_from_rows(expected_rows)
    got_rows = live_rows(captured['rows'])
    if captured['count'] != len(expected_rows):
        padding = len(captured['rows']) - len(got_rows)
        if (captured['count'] == len(captured['rows'])
                and len(got_rows) == len(expected_rows)):
            notes.append('count: table stores %d slots, %d of them trailing '
                         'zero-size; partinf keeps %d live -> not a difference'
                         % (captured['count'], padding, len(expected_rows)))
        else:
            diffs.append('count: partinf has %d slots, table says %d'
                         % (len(expected_rows), captured['count']))
    if captured['magic'] != exp['magic']:
        notes.append('magic: %s vs %s -- both accepted, not a difference'
                     % (magic_text(exp['magic']), magic_text(captured['magic'])))
    by_slot = {row.slot: row for row in got_rows}
    for want in expected_rows:
        got = by_slot.get(want.slot)
        if got is None:
            diffs.append('slot %d (%s): missing from the table'
                         % (want.slot, want.name))
            continue
        for name, a, b in (('start', want.start, got.start),
                           ('count', want.count, got.count),
                           ('type', want.type, got.type),
                           ('flag', want.flag, got.flag)):
            if a != b:
                diffs.append('slot %d (%s) %s: expected 0x%08x, got 0x%08x'
                             % (want.slot, want.name, name, a, b))
    for got in got_rows:
        if got.slot > len(expected_rows):
            diffs.append('slot %d (%s): present in the table, absent from '
                         'partinf' % (got.slot, got.name))
    return exp, diffs, notes


# ---- selftest -------------------------------------------------------------

def selftest():
    root = Path(__file__).resolve().parents[2]
    conf = root / 'a7c_fw_key_artifacts' / 'partinf.conf'
    if not conf.exists():
        raise SystemExit('missing fixture: %s' % conf)

    expected = parse_partinf(conf)
    checks = []

    def check(name, ok, detail=''):
        checks.append((name, bool(ok), detail))
        print('%-4s %s%s' % ('PASS' if ok else 'FAIL', name,
                             (' -- ' + detail) if detail and not ok else ''))

    # 1. partinf.conf -> 23 live slots, 16 valid, extent from nflasha23.
    check('partinf yields 23 slots', len(expected) == 23, str(len(expected)))
    check('partinf has 16 valid slots',
          sum(r.valid for r in expected) == 16,
          str(sum(r.valid for r in expected)))
    check('extent is 0x50a00000',
          max(r.start + r.count for r in expected if r.valid) == 0x50A00000)
    check('slot 1 is nflasha1 0x10000+0x7f0000',
          expected[0].key() == (0x10000, 0x7F0000, 1, 0xFFFFFFFF))
    check('slot 23 is PMBP 0x50800000+0x200000',
          expected[22].key() == (0x50800000, 0x200000, 1, 0xFFFFFFFF))

    # 2. Byte-offset proof: slot 1 ends exactly where slot 2 starts.
    check('nflasha1 ends where nflasha2 starts (byte units)',
          expected[0].start + expected[0].count == expected[1].start,
          '0x%x vs 0x%x' % (expected[0].start + expected[0].count,
                            expected[1].start))
    #   If start were an LBA, CMD17 would read byte 0x10000*512 = 0x2000000,
    #   which lands inside nflasha3 (0x01C00000+0x03000000) -- i.e. the read
    #   silently returns another partition's data, 512x too far out.
    check('sector reading of nflasha1 would land in nflasha3 (512x)',
          expected[2].start <= expected[0].start * SECTOR
          < expected[2].start + expected[2].count,
          'lba-read 0x%x vs nflasha3 0x%x+0x%x' % (
              expected[0].start * SECTOR, expected[2].start,
              expected[2].count))
    check('nflasha1 byte 0x10000 is LBA 128',
          expected[0].start // SECTOR == 128)

    #   INFO tag (4) + header + slot line must fit FB_RESPONSE_MAX = 64.
    log_lines = emit_log(table_from_rows(expected))
    check('log lines fit the 64-byte reply buffer',
          max(len(line) for line in log_lines) + 4 <= 64,
          'longest %d' % max(len(line) for line in log_lines))

    # 3. Synthetic 4-sector image -> parse_raw -> same fields.
    for magic in MAGICS:
        raw = build_raw(expected, magic)
        check('%s image is %d bytes' % (magic.decode(), len(raw)),
              len(raw) == TABLE_BYTES)
        table = parse_raw(raw)
        check('%s magic parsed back' % magic.decode(),
              table['magic'] == magic)
        check('%s count 23, valid 16, extent 0x50a00000'
              % magic.decode(),
              (table['count'], table['valid'], table['extent'])
              == (23, 16, 0x50A00000),
              '%r' % ((table['count'], table['valid'],
                       table['extent']),))
        check('%s round trip is field-identical' % magic.decode(),
              [r.key() for r in table['rows']] == [r.key() for r in expected])

        # 4. Same image through the payload's log format and back.
        logged = parse_log(emit_log(table))
        check('%s log round trip is field-identical' % magic.decode(),
              [r.key() for r in logged['rows']] == [r.key() for r in expected])
        check('%s log lines are 46 chars or less' % magic.decode(),
              all(len(line) <= 46 for line in emit_log(table)[1:]))

        # 5. compare() on the synthetic capture must be clean.
        _, diffs, _ = compare(expected, logged)
        check('%s compare is clean' % magic.decode(), not diffs,
              '; '.join(diffs[:3]))

    # 6. Both renderings of a magic must name the same four bytes.
    for magic in MAGICS:
        name = magic.decode()
        check('%s name and LE32 hex are the same magic' % name,
              magic_text(magic) == name
              and parse_magic(name) == parse_magic(magic_hex(magic)) == magic)

    # 7. Trailing zero-size slots: the A7C's table stores count=30 while
    #    partinf keeps 23 live rows; that alone must be a note, not a DIFF.
    padded = expected + [Row(len(expected) + i + 1,
                             'nflasha%d' % (len(expected) + i + 1),
                             0, 0, 0, 0) for i in range(7)]
    stored30 = parse_raw(build_raw(padded))
    _, diffs30, notes30 = compare(expected, stored30)
    check('trailing zero-size slots are not a diff', not diffs30,
          '; '.join(diffs30[:3]))
    check('trailing zero-size slots are reported as a note', bool(notes30))

    # 8. Bad magic must be rejected, not silently accepted.
    bad = bytearray(build_raw(expected))
    bad[0:4] = b'XXXX'
    try:
        parse_raw(bytes(bad))
        check('bad magic rejected', False)
    except SystemExit:
        check('bad magic rejected', True)

    failed = [name for name, ok, _ in checks if not ok]
    print()
    print('%d checks, %d failed' % (len(checks), len(failed)))
    return 1 if failed else 0


# ---- main -----------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='verb', required=True)

    p_exp = sub.add_parser('expected', help='partinf.conf -> expected table CSV')
    p_exp.add_argument('conf')
    p_exp.add_argument('--csv')

    p_cap = sub.add_parser('capture', help='dump or log -> table CSV')
    src = p_cap.add_mutually_exclusive_group(required=True)
    src.add_argument('--raw', help='2048-byte table image')
    src.add_argument('--log', help='captured `oem partition` output')
    p_cap.add_argument('--csv')

    p_cmp = sub.add_parser('compare', help='partinf.conf vs a capture')
    p_cmp.add_argument('conf')
    src = p_cmp.add_mutually_exclusive_group(required=True)
    src.add_argument('--raw')
    src.add_argument('--log')

    p_syn = sub.add_parser('synth',
                           help='write a synthetic 2048-byte table image')
    p_syn.add_argument('conf')
    p_syn.add_argument('--out', required=True)
    p_syn.add_argument('--magic', default='6428', choices=('6428', '8246'))

    sub.add_parser('selftest', help='offline round trip against partinf.conf')
    args = parser.parse_args()

    if args.verb == 'selftest':
        return selftest()

    if args.verb == 'synth':
        rows = parse_partinf(args.conf)
        raw = build_raw(rows, args.magic.encode())
        Path(args.out).write_bytes(raw)
        print('wrote %s (%d bytes, %d slots, magic %s)'
              % (args.out, len(raw), len(rows), args.magic))
        return 0

    def save_csv(table, path):
        buf = io.StringIO()
        write_csv(table, buf)
        Path(path).write_text(buf.getvalue(), encoding='utf-8')
        print('csv -> %s' % path)

    if args.verb == 'expected':
        table = table_from_rows(parse_partinf(args.conf))
        print_summary('expected', table)
        if args.csv:
            save_csv(table, args.csv)
    elif args.verb == 'capture':
        if args.raw:
            table = parse_raw(Path(args.raw).read_bytes())
        else:
            table = parse_log(Path(args.log).read_text(
                encoding='utf-8', errors='replace').splitlines())
        print_summary('captured', table)
        if args.csv:
            save_csv(table, args.csv)
    else:
        expected_rows = parse_partinf(args.conf)
        if args.raw:
            captured = parse_raw(Path(args.raw).read_bytes())
        else:
            captured = parse_log(Path(args.log).read_text(
                encoding='utf-8', errors='replace').splitlines())
        exp, diffs, notes = compare(expected_rows, captured)
        print_summary('expected', exp)
        print_summary('captured', captured)
        for line in notes:
            print('note: %s' % line)
        for line in diffs:
            print('DIFF %s' % line)
        print('%s: %d difference(s)'
              % ('MATCH' if not diffs else 'MISMATCH', len(diffs)))
        return 0 if not diffs else 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
