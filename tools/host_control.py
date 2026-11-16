#!/usr/bin/env python3
"""host_control.py — PC-side controller for the uart_boot firmware (RP2040).

Protocol lineage: the ILCE-7M4-RE uart_boot host tool. The A7C variant adds
CMD_TIMING (0x07) and its firmware source lives in ../rp2040/firmware.

Packet format: [type:1][len:2 little-endian][data:len]

Commands:
  ping                      check the RP2040 firmware is alive
  reset                     pulse MULTI_RST (only useful if GPIO27 is wired)
  carrier on|off            start/stop the boot carrier waveform
  carrier on --hold [S]     keep re-arming the carrier (firmware drops it
                            after 1.2 s without a blob); no value = Ctrl+C
  blob DATA                 send one record as an 11-bit UART frame burst
  baud N                    switch RP2040 Serial1 to the camera (0 = off)
  uart DATA                 send raw bytes to the camera (needs UART_TX build)
  monitor [--duration S]    print EVT_UART/EVT packets from the RP2040
  push FILE                 full bootloader payload push (see options)

DATA args: hex string ("01 A2 03") or @file.bin.

Cold start (no RST wired): use tools/a3push.py --cold instead.
  The old replay-while-you-insert loop here is not safe for the A7C: the ROM
  (0xffff0802) treats a record that repeats the group's first type as the end
  of the group, so adjacent MEM_WRITEs end it with flag0==flag1==0 and answer
  0x2004 (1 long, 4 short).  a3push holds the carrier instead and sends once.
"""

import argparse
import enum
import re
import struct
import sys
import time
import zlib

import serial
from serial.tools import list_ports

# rp2040 protocol details (must match uart_boot.ino)

class RpCmd(enum.IntEnum):
    PING    = 0x01
    RESET   = 0x02
    CARRIER = 0x03
    BLOB    = 0x04
    BAUD    = 0x05
    UART    = 0x06
    TIMING  = 0x07
    SENSE   = 0x0B
    PULL    = 0x0C


class RpEvt(enum.IntEnum):
    ACK  = 0x81
    ERR  = 0x82
    UART = 0x83


RP_VERSION = b"uart_boot 1"

# bootrom serial boot protocol (same as scripts/push_payload.py)

class SerCmd(enum.IntEnum):
    SET_UNLOCK = 0x0000
    MEM_WRITE  = 0x0001
    END_MARKER = 0x0123
    SET_ENTRY  = 0xFFFF


LOAD_ADDR = 0xFE020000
MAX_RECORD_PAYLOAD = 512   # true max 4086; chunking keeps progress smooth
TRANSMIT_START = 0.5
RECORD_GAP = 0.0144

# A7C (CXD90045): the unlock value must equal the token's V field, otherwise
# the ROM flashes 0x2002 and never jumps. Verified on hardware 2026-10-20.
A7C_UNLOCK = 0x8CB102DC
# 20833 ns/bit = 48001 baud.  slow=12 puts the 60-edge preamble block at
# 250 us, inside the ROM's 909..1184 tick sync window (225.4..293.6 us).
# slow=11 is 229.2 us -- only 1.65 % above the floor -- and fails on hardware
# with 1 long + 4 short (0x2004). Verified 2026-10-21: slow=12 -> 1 long 48.
TIMING_Q, TIMING_SLOW, TIMING_FAST = 20833, 12, 129


def record(rtype: int, value: int, payload=b"") -> bytes:
    body = struct.pack("<IHI", 10 + len(payload), rtype, value) + payload
    return body + struct.pack("<I", zlib.crc32(body))


def boot_script(payload: bytes, addr=LOAD_ADDR, entry=None, unlock_code=None):
    if entry is None:
        entry = addr
    if unlock_code is None:
        unlock_code = A7C_UNLOCK

    # First and last record must share a type, so the session is both opened
    # and closed with SET_UNLOCK (same layout as the verified A7C run).
    recs = [record(SerCmd.SET_UNLOCK, unlock_code)]

    for off in range(0, len(payload), MAX_RECORD_PAYLOAD):
        recs.append(record(SerCmd.MEM_WRITE, addr + off,
                           payload[off:off + MAX_RECORD_PAYLOAD]))

    recs.append(record(SerCmd.SET_ENTRY, entry))
    recs.append(record(SerCmd.SET_UNLOCK, unlock_code))
    return recs


def parse_records(spec: str):
    """Build a record list from a short spec, for protocol bisection.

    end                  -> END_MARKER, value 0
    entry:ADDR           -> SET_ENTRY
    unlock:CODE          -> SET_UNLOCK
    write:ADDR:HEXDATA   -> MEM_WRITE
    """
    recs = []
    for tok in spec.split(","):
        tok = tok.strip()
        if not tok:
            continue
        if tok == "end":
            recs.append(record(SerCmd.END_MARKER, 0))
            continue
        name, _, rest = tok.partition(":")
        if name == "entry":
            recs.append(record(SerCmd.SET_ENTRY, int(rest, 0)))
        elif name == "unlock":
            recs.append(record(SerCmd.SET_UNLOCK, int(rest, 0)))
        elif name == "write":
            addr_s, _, hex_s = rest.partition(":")
            recs.append(record(SerCmd.MEM_WRITE, int(addr_s, 0), parse_data(hex_s)))
        else:
            raise Error(f"bad record spec: {tok}")
    return recs


class Error(Exception):
    pass


def pick_port(explicit):
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    if not ports:
        sys.exit("no serial ports found; plug in the RP2040 or pass -p COMx")
    for p in ports:
        if p.vid == 0x2E8A:  # Raspberry Pi (RP2040 CDC)
            return p.device
    for p in ports:
        desc = (p.description or "") + " " + (getattr(p, "product", "") or "")
        if "RP2040" in desc or "Pico" in desc:
            return p.device
    return ports[0].device


class Device:
    def __init__(self, port, timeout=5.0):
        self.ser = serial.Serial(port, 115200, timeout=timeout)

    def close(self):
        self.ser.close()

    def send(self, cmd: int, data=b""):
        self.ser.write(bytes([cmd]) + struct.pack("<H", len(data)) + data)

    def _read_exact(self, n):
        out = b""
        while len(out) < n:
            chunk = self.ser.read(n - len(out))
            if not chunk:
                return None
            out += chunk
        return out

    def recv(self):
        hdr = self._read_exact(3)
        if hdr is None:
            return None
        (n,) = struct.unpack("<H", hdr[1:3])
        data = self._read_exact(n) if n else b""
        if data is None:
            return None
        return RpEvt(hdr[0]), data

    def ack(self):
        return self.ack_quiet()

    def ack_quiet(self, *ignore):
        """Wait for ACK; drop EVT_ERR messages whose text is in `ignore`."""
        while True:
            pkt = self.recv()
            if pkt is None:
                raise Error("timed out waiting for the device")
            evt, data = pkt
            if evt == RpEvt.ACK:
                return data
            if evt == RpEvt.ERR:
                msg = data.decode("utf-8", "replace")
                if msg in ignore:
                    continue
                raise Error(msg)
            if evt == RpEvt.UART:
                dump_uart(data)

    def ping(self):
        self.send(RpCmd.PING)
        ver = self.ack()
        if ver != RP_VERSION:
            raise Error(f"bad version: {ver}")
        print(f"[ok] ping -> {ver.decode()}")

    def reset(self):
        self.send(RpCmd.RESET)
        self.ack()
        print("[ok] reset pulse sent")

    def carrier(self, on, quiet=False):
        self.send(RpCmd.CARRIER, bytes([1 if on else 0]))
        self.ack()
        if not quiet:
            print(f"[ok] carrier {'on' if on else 'off'}")

    def blob(self, rec: bytes):
        self.send(RpCmd.BLOB, rec)
        self.ack()

    def baud(self, baud):
        self.send(RpCmd.BAUD, struct.pack("<I", baud))
        self.ack()
        print(f"[ok] uart mode baud={baud}" if baud else "[ok] uart mode off")

    def timing(self, q, slow, fast, quiet=False):
        """Retune the boot waveform; the A7C-verified values are 20833/12/129."""
        self.send(RpCmd.TIMING, struct.pack("<IBB", q, slow, fast))
        body = self.ack()
        qo, so, fo = struct.unpack("<IBB", body)
        if not quiet:
            print(f"[ok] timing q={qo} ns slow={so} fast={fo} "
                  f"({qo / 1000.0:.1f} us/bit, {1e9 / qo:.0f} baud)")
        return qo, so, fo

    def uart(self, data: bytes):
        self.send(RpCmd.UART, data)
        self.ack()

    def sense(self):
        """True if the data line went high at any point in the last 2 ms."""
        self.send(RpCmd.SENSE)
        return self.ack() == b"\x01"

    def pull(self, up=False):
        """Internal pull-up on the data line; only allowed while idle."""
        self.send(RpCmd.PULL, bytes([1 if up else 0]))
        self.ack()


def hold_carrier(dev, seconds=None, interval=0.5):
    """Keep the carrier running.

    The firmware drops carrier mode after STARVE_MS (1200 ms) without a blob,
    so a bare `carrier on` dies ~1.2 s later. Re-sending CMD_CARRIER(1) only
    refreshes the timer (mode is already CARRIER), and if it did starve it
    restarts the PIO program.
    """
    t_end = None if seconds is None else time.time() + seconds
    n = 0
    dev.send(RpCmd.CARRIER, b"\x01")
    dev.ack_quiet("starved")
    print(f"[hold] carrier on, re-arming every {interval * 1000:.0f} ms "
          f"(Ctrl+C to release)")
    try:
        while t_end is None or time.time() < t_end:
            time.sleep(interval)
            dev.send(RpCmd.CARRIER, b"\x01")
            dev.ack_quiet("starved")
            n += 1
            elapsed = n * interval
            tail = f" / {seconds:.0f}s" if seconds else ""
            print(f"\r[hold] carrier alive {elapsed:.1f}s{tail}", end="", flush=True)
    except KeyboardInterrupt:
        pass
    print("\n[hold] releasing carrier")
    dev.carrier(0)


def dump_uart(data: bytes):
    hexs = " ".join(f"{b:02X}" for b in data[:64])
    more = " ..." if len(data) > 64 else ""
    text = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in data[:64])
    print(f"<< {hexs}{more}  |{text}|")


def monitor(dev, duration=None):
    print("--- monitoring EVT packets (Ctrl+C to stop) ---")
    t_end = time.time() + duration if duration else None
    try:
        while t_end is None or time.time() < t_end:
            pkt = dev.recv()
            if pkt is None:
                continue
            evt, data = pkt
            if evt == RpEvt.UART:
                dump_uart(data)
            elif evt == RpEvt.ERR:
                print(f"[ERR] {data.decode('utf-8', 'replace')}")
            elif evt == RpEvt.ACK and data:
                print(f"[ACK] {hexline(data)}")
    except KeyboardInterrupt:
        pass


def hexline(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def parse_data(s: str) -> bytes:
    if s.startswith("@"):
        with open(s[1:], "rb") as f:
            return f.read()
    hx = re.sub(r"(0x|[^0-9a-fA-F])", "", s)
    if len(hx) % 2:
        raise Error("hex data must have an even number of digits")
    return bytes.fromhex(hx)


def cmd_push(dev, args):
    if not args.records and not args.payload:
        raise Error("push needs a payload file or --records")
    if args.records:
        recs = parse_records(args.records)
        unlock = None
    else:
        with open(args.payload, "rb") as f:
            payload = f.read()
        unlock = int(args.unlock, 0) if args.unlock else None
        entry = int(args.entry, 0) if args.entry else None
        recs = boot_script(payload, args.addr, entry, unlock)
    total = sum(len(r) for r in recs)
    print(f"[push] {len(recs)} records, {total} B on the wire")
    if args.records:
        for i, r in enumerate(recs):
            ln, rtype, val = struct.unpack_from("<IHI", r)
            print(f"[push]   #{i + 1}: type=0x{rtype:04X} value=0x{val:08X} "
                  f"len={ln} crc=0x{struct.unpack_from('<I', r, len(r) - 4)[0]:08X}")

    dev.ping()
    if args.manual:
        print("[push] 1) pull the camera battery  "
              "2) press Enter the same instant you re-insert it")
        input("[push] insert battery + Enter NOW ... ")
        dev.carrier(1)              # re-arm: it may have starved while waiting
        time.sleep(args.delay)      # bootrom opens its RX ~0.5 s after power-on
        print(f"[push] sending {len(recs)} records after {args.delay:g}s")
        for i, rec in enumerate(recs):
            dev.blob(rec)
            print(f"\r[push] {i + 1}/{len(recs)}", end="", flush=True)
            if i + 1 < len(recs):
                time.sleep(RECORD_GAP)
        print()
        dev.carrier(0)
    elif args.cold:
        print(f"[push] COLD START: insert the camera battery NOW; "
              f"streaming for {args.cold_window}s")
        dev.carrier(1)
        time.sleep(TRANSMIT_START)
        deadline = time.time() + args.cold_window
        n_pass = 0
        while time.time() < deadline:
            for i, rec in enumerate(recs):
                dev.blob(rec)
                if time.time() >= deadline:
                    break
                time.sleep(RECORD_GAP)
            n_pass += 1
            print(f"[push] pass {n_pass} done ({time.time():.0f}s remain)")
        dev.carrier(0)
    else:
        dev.reset()
        dev.carrier(1)
        time.sleep(TRANSMIT_START)
        print(f"[push] sending {len(recs)} records")
        for i, rec in enumerate(recs):
            dev.blob(rec)
            print(f"\r[push] {i + 1}/{len(recs)}", end="", flush=True)
            if i + 1 < len(recs):
                time.sleep(RECORD_GAP)
        print()
        dev.carrier(0)

    print("[push] done. Camera state: "
          "if it failed to boot, the trigger likely landed; "
          "a normal boot means the waveform missed the window.")

    if args.baud:
        dev.baud(args.baud)
        try:
            monitor(dev, args.monitor_duration)
        finally:
            dev.baud(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-p", "--port", help="COM port (default: autodetect RP2040)")
    ap.add_argument("-t", "--timeout", type=float, default=5.0)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping")
    sub.add_parser("reset")

    p = sub.add_parser("carrier")
    p.add_argument("state", choices=["on", "off"])
    p.add_argument("--hold", nargs="?", const=-1.0, type=float, default=0.0,
                   help="keep re-arming the carrier: no value = until Ctrl+C, "
                        "N = N seconds (needed to outlive the 1.2 s starve timeout)")

    p = sub.add_parser("blob")
    p.add_argument("data", help="hex or @file")

    p = sub.add_parser("baud")
    p.add_argument("rate", type=lambda s: int(s, 0), help="0 = off")

    p = sub.add_parser("uart")
    p.add_argument("data", help="hex or @file (requires UART_TX firmware build)")

    p = sub.add_parser("monitor")
    p.add_argument("--duration", type=float, default=None)

    p = sub.add_parser("push")
    p.add_argument("payload", nargs="?", help="payload bin (e.g. out/payload.bin)")
    p.add_argument("--records", default=None,
                   help="raw record list instead of a payload, e.g. "
                        "'end' or 'end,entry:0xFE020000,end' or "
                        "'write:0xFE020000:0011'")
    p.add_argument("-a", "--addr", type=lambda s: int(s, 0), default=LOAD_ADDR)
    p.add_argument("-e", "--entry", type=lambda s: int(s, 0), default=None)
    p.add_argument("-u", "--unlock", default=None,
                   help=f"unlock code (default 0x{A7C_UNLOCK:08X}, the A7C token V)")
    p.add_argument("--cold", action="store_true",
                   help="legacy blind replay (A7C-dangerous: adjacent MEM_WRITEs "
                        "make the ROM answer 0x2004); use a3push.py --cold")
    p.add_argument("--cold-window", type=float, default=25.0)
    p.add_argument("--manual", action="store_true",
                   help="no RST wired: press Enter on battery insert, then send "
                        "one clean pass (mimics the original reset timing)")
    p.add_argument("--delay", type=float, default=TRANSMIT_START,
                   help="seconds between Enter and the first record (default 0.5)")
    p.add_argument("-b", "--baud", type=int, default=0,
                   help="camera uart baud after push (0 = leave idle)")
    p.add_argument("--monitor-duration", type=float, default=None)
    args = ap.parse_args()

    port = pick_port(args.port)
    print(f"# port: {port}")
    dev = Device(port, args.timeout)
    try:
        if args.cmd == "ping":
            dev.ping()
        elif args.cmd == "reset":
            dev.reset()
        elif args.cmd == "carrier":
            if args.state == "on" and args.hold:
                hold_carrier(dev, None if args.hold < 0 else args.hold)
            else:
                dev.carrier(args.state == "on")
        elif args.cmd == "blob":
            dev.carrier(1)  # firmware rejects CMD_BLOB unless carrier mode
            dev.blob(parse_data(args.data))
            dev.carrier(0)
        elif args.cmd == "baud":
            dev.baud(args.rate)
        elif args.cmd == "uart":
            dev.uart(parse_data(args.data))
        elif args.cmd == "monitor":
            monitor(dev, args.duration)
        elif args.cmd == "push":
            cmd_push(dev, args)
    except Error as e:
        print(f"[dev error] {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        dev.close()


if __name__ == "__main__":
    main()
