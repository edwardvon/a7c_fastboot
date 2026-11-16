#!/usr/bin/env python3
"""Push an A3 image to the A7C BootROM over the RP2040 UART bridge.

Reuses host_control.py's record framing and serial Device class (both live in
this directory; the RP2040 firmware source is in ../rp2040/firmware). Data
goes out in 4086-byte MEM_WRITEs: with the 10-byte record header and 4-byte
CRC that is exactly the RP2040's 4100-byte MAX_RECORD, so no chunk is
rejected.

Nothing is transmitted unless --send is given; the default run only prints the
record plan.

Both modes need pyserial (pip install pyserial): this script imports
host_control, which imports it unconditionally. Plan mode never opens the port.

    python tools\a3push.py out\payload.a3                  # plan only
    python tools\a3push.py out\payload.a3 --send --cold

--cold reproduces the A6700 community procedure (bootrom 0xffff0802 /
0xffff100a), which replaces every timing guess with a hardware trigger:

  * The carrier is armed *before* the battery goes in.  Every carrier edge is
    "too early" (0x3003), which resets the bootrom's consecutive-slow-edge
    counter and re-arms its edge watchdog, so the ROM parks in the sync FSM
    with its counter at 0 instead of timing out (0x3002) and booting normally.
  * CMD_SENSE just samples GP28.  It reads low while the camera is unpowered;
    fitting the battery powers the camera up and a pull-up appears on D-, so
    the sample going high means "the camera is powered", not "the ROM released
    the line".  That edge is the trigger, not a stopwatch.
  * Only after that trigger, 0.5 s later, the whole record group is sent
    exactly once, starting with SET_UNLOCK.  The ROM then counts the 60 slow
    preamble blocks from zero and starts parsing exactly on the first
    SET_UNLOCK.

An already-high baseline aborts the run with nothing sent: it means the test
board is still attached, or the camera is powered and driving the line.

Three hard rules, all verified in the ROM image:

  * Never go silent for >~294 us between arming the carrier and the last
    record: a single 0x3002 makes the ROM return 0x3001, which is a tail jump
    to [0xFE00500C] -- i.e. it just boots the camera.  (The silence threshold
    is 0x4a0 = 1184 ticks of the ROM's 0xF2403000 timer; one tick is 0.248 us,
    see the tick scale below.)
  * The group must be [SET_UNLOCK, MEM_WRITE..., SET_ENTRY, SET_UNLOCK].
    0xffff0802 reads records until one repeats the *first* type (cmp sl,r0),
    so it closes only on a second SET_UNLOCK; adjacent MEM_WRITEs are fine.
    Closing with flag0==flag1==0 is the 1-long-4-short (0x2004) death loop.
  * Every preamble edge must be 909..1184 ticks apart (225.4..293.6 us).  A
    shorter edge is 0x3003 "too early": non-fatal, but it zeroes the
    consecutive-edge counter, so a whole run of them never reaches 60 and the
    1.45 s timer A expires as 0x2004.  Each record entry (0xffff0828) resets
    that counter, so *every* record needs its own 60-edge preamble.

Tick scale: the ROM samples bit k of a byte at ROM-table offsets
[0xffff2f84 + 4k] (strap bit5 = 1) = 0x1c, 0x70, 0xc4, ... i.e. a first
sample at 28 ticks and a constant 84-tick step -- one bit period.  With
--q 20833 ns (one bit) one tick = 20833/84 = 248 ns, which is what turns the
909/1184/830/1802 constants into microseconds.

That makes the slow preamble block the one truly critical number: slow_q * q
must stay inside 225.4..293.6 us.  --slow 11 (229.2 us) is only 1.65 % above
the floor and fails on hardware with 1 long + 4 short; --slow 12 (250.0 us)
is the value verified on 2026-10-21 and is also host_control.TIMING_SLOW, so
the defaults are good.  slow_q only changes the preamble block, never the bit
period, so it can be swept alone.
The byte-tail window is much wider -- 830..1802 ticks = 206..447 us per 11-bit
frame -- so a 250 us frame has ample margin there.

Timer A (0x15F900<<2 = 5,760,000 ticks = ~1.45 s) bounds the sync phase: it is
armed when the ROM enters 0xffff0802 and is NOT refreshed by the "too early"
(0x3003) edges the carrier produces.  So the 60 slow preamble blocks must land
within 1.45 s of the ROM starting to listen, and the preamble length equals the
ROM's 60-edge requirement exactly (zero phase margin) -- which is why
--probe exists: it tells us *when* the ROM starts listening instead of guessing.

--cold is a lottery, not a broken mode.  It sleeps 0.5 s after the camera
powers up, and one full group already takes ~1.35 s of wire time (a 4100-byte
record alone is 0.94 s at 229 us/byte), so the group ends within ~100 ms of
the 1.45 s deadline while the moment the ROM actually starts listening is not
under our control.  --stream instead repeats the whole group back to back so
the battery may go in at any moment and a fresh group always lands inside the
window.  The camera has a single USB port, so nothing can enumerate while this
bridge is wired: --stream stops on a timer and the result is read from the LED
code the payload paints.  Switching the port over is a separate, manual step
(pull the whole test board first, then move the data cable from the Pico end
to the camera -- see the a6700 manual).
"""

import argparse
import ctypes
import struct
import sys
import time
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import host_control as hc          # noqa: E402  (same directory)
import a3image as a3               # noqa: E402  (same directory)

LOAD_ADDR = hc.LOAD_ADDR           # 0xFE020000
MAX_CHUNK = 4086                   # 4100-byte record limit on the RP2040
ARM_POLL = 0.02                    # carrier refresh + sense poll period
SENSE_DELAY = 0.5                  # wait after the trigger, before record #1


def build_records(image: bytes, chunk: int, unlock_code=None, cycles=1,
                  entry=None, crc_init=0xFFFFFFFF):
    """Mirror host_control.boot_script(), but with the tighter chunk size.

    The group must start and end with SET_UNLOCK and hold exactly one SET_ENTRY,
    because 0xffff0802 parses records until one repeats the type of the first
    record after sync (any other repeat closes the group early).
    """
    if chunk < 1 or chunk > MAX_CHUNK:
        raise SystemExit(f"chunk must be 1..{MAX_CHUNK}, got {chunk}")

    if entry is None:
        entry_off = struct.unpack_from("<I", image, 20)[0]
        entry = (LOAD_ADDR + entry_off) | 1        # Thumb

    if unlock_code is None:
        # The ROM compares this against the token's V field; only the real
        # value lets SET_ENTRY through (0xDEADBEEF was a 7M4 stack-smash
        # trick and is not accepted here -- verified on hardware 2026-10-20).
        unlock_code = hc.A7C_UNLOCK

    # First and last record must share a type: SET_UNLOCK brackets the push.
    recs = [hc.record(hc.SerCmd.SET_UNLOCK, unlock_code)]

    for _ in range(cycles):
        for off in range(0, len(image), chunk):
            recs.append(hc.record(hc.SerCmd.MEM_WRITE, LOAD_ADDR + off,
                                  image[off:off + chunk]))

    recs.append(hc.record(hc.SerCmd.SET_ENTRY, entry))
    recs.append(hc.record(hc.SerCmd.SET_UNLOCK, unlock_code))

    # Diagnostic knob: the ROM presets its CRC register with #-1 (0xFFFFFFFF)
    # and inverts at the end, i.e. plain crc32.  Any other preset makes every
    # record fail with 0x2003 instead, which tells us whether the ROM ever got
    # as far as parsing record #1 (0x2003) or never synced at all (0x2004).
    if crc_init != 0xFFFFFFFF:
        recs = [r[:-4] + struct.pack("<I", zlib.crc32(r[:-4],
                                                     (~crc_init) & 0xFFFFFFFF))
                for r in recs]
    return recs, entry


def describe(recs, size):
    total = sum(len(r) for r in recs)
    print(f"[a3] image {size} B, {len(recs)} records, {total} B on the wire")
    for i, rec in enumerate(recs):
        ln, rtype, val = struct.unpack_from("<IHI", rec)
        crc = struct.unpack_from("<I", rec, len(rec) - 4)[0]
        print(f"[a3]   #{i + 1:<3} type=0x{rtype:04X} value=0x{val:08X} "
              f"len={ln:<5} crc=0x{crc:08X}")
    biggest = max(len(r) for r in recs)
    print(f"[a3] largest record {biggest} B (limit {MAX_CHUNK + 14})")


_iface_api = None


def _load_iface_api():
    """cfgmgr32 device-interface enumeration (present USB devices only)."""
    global _iface_api
    if _iface_api is not None:
        return _iface_api or None
    try:
        class GUID(ctypes.Structure):
            _fields_ = [("Data1", ctypes.c_ulong), ("Data2", ctypes.c_ushort),
                        ("Data3", ctypes.c_ushort), ("Data4", ctypes.c_ubyte * 8)]

        guid = GUID()
        guid.Data1, guid.Data2, guid.Data3 = 0xA5DCBF10, 0x6530, 0x11D2
        # GUID_DEVINTERFACE_USB_DEVICE
        guid.Data4[:] = (0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED)

        cm = ctypes.WinDLL("cfgmgr32")
        size_fn = cm.CM_Get_Device_Interface_List_SizeW
        size_fn.argtypes = [ctypes.POINTER(ctypes.c_ulong),
                            ctypes.POINTER(GUID), ctypes.c_wchar_p,
                            ctypes.c_ulong]
        size_fn.restype = ctypes.c_ulong
        list_fn = cm.CM_Get_Device_Interface_ListW
        list_fn.argtypes = [ctypes.POINTER(GUID), ctypes.c_wchar_p,
                            ctypes.c_wchar_p, ctypes.c_ulong, ctypes.c_ulong]
        list_fn.restype = ctypes.c_ulong
        _iface_api = (size_fn, list_fn, guid)
    except (OSError, AttributeError):
        _iface_api = False
    return _iface_api or None


def usb_present(vid, pid):
    """True if Windows currently has a USB device with this VID/PID attached.

    The cfgmgr32 interface list costs microseconds, so the cold loop can poll
    it between records; `Get-PnpDevice` takes ~2 s and would miss the boot.
    """
    api = _load_iface_api()
    if not api:
        return False
    size_fn, list_fn, guid = api
    need = ctypes.c_ulong(0)
    if size_fn(ctypes.byref(need), ctypes.byref(guid), None, 0) != 0:
        return False
    if not need.value:
        return False
    buf = ctypes.create_unicode_buffer(need.value)
    if list_fn(ctypes.byref(guid), None, buf, need.value, 0) != 0:
        return False
    return f"VID_{vid:04X}&PID_{pid:04X}".lower() in buf[:].lower()


def _cold_sense(dev, recs, timeout):
    """A6700 procedure: arm the carrier, then let the camera pick the moment.

    The carrier is armed before the battery goes in and re-sent every 20 ms
    (which also keeps the firmware's 1.2 s starve guard alive).  GP28 is
    sampled between refreshes: it reads low while the camera is unpowered, and
    the first high sample means the camera is powered (a pull-up on D-), not
    that the ROM released the line.  Only then is the group sent, exactly once.

    Returns the time the group started.  The line is never released until the
    last record is out (a >~300 us gap is a 0x3002 -> 0x3001 normal boot).
    """
    dev.carrier(0, quiet=True)
    dev.pull(False)
    dev.carrier(1, quiet=True)
    time.sleep(0.01)

    try:
        if dev.sense():
            raise hc.Error("baseline already HIGH -- test board still attached "
                           "or the camera is powered. Nothing was sent.")

        print("[a3] ARMED: baseline low.  Fit the battery, switch the camera ON, "
              "then leave it alone.", flush=True)
        t_arm = time.time()
        while not dev.sense():
            if time.time() - t_arm >= timeout:
                raise hc.Error(f"camera activation not detected in {timeout:g}s; "
                               f"nothing was sent")
            dev.send(hc.RpCmd.CARRIER, b"\x01")
            dev.ack_quiet("starved")
            time.sleep(ARM_POLL)

        print(f"\r[a3] camera powered up after {time.time() - t_arm:.1f}s "
              f"-- sending the group once")
        time.sleep(SENSE_DELAY)

        t_start = time.time()
        for i, rec in enumerate(recs):
            dev.blob(rec)
            print(f"\r[a3] {i + 1}/{len(recs)}", end="", flush=True)
            if i + 1 < len(recs):
                time.sleep(hc.RECORD_GAP)
        print()
        # Give the ROM a moment to consume the last record, then release the
        # line so the payload's USB PHY owns it.
        time.sleep(0.1)
    finally:
        dev.carrier(0, quiet=True)
        dev.pull(False)
    return t_start


def _stream_loop(dev, recs, seconds, gap):
    """Repeat the whole group until `seconds` elapse; no USB probe.

    The carrier hums from the start, so the battery may go in at any moment:
    one round is ~1.35 s, which is shorter than the ROM's 1.45 s sync window,
    so a fresh group start is always less than one round away -- i.e. always
    inside the window, whenever the ROM happens to enter 0xffff0802.
    Every round is a complete [SET_UNLOCK ... SET_ENTRY SET_UNLOCK]
    group, so once the ROM syncs on record #1 it closes the group and jumps
    into the payload while the remaining rounds are ignored.

    The camera exposes a single USB port, so nothing can enumerate while the
    bridge is still wired; the result is the LED code the payload paints.
    """
    # Ask first, arm second: the firmware drops the carrier (STARVE_MS = 1.2 s)
    # if no blob follows CMD_CARRIER, so nothing may block between the two.
    print(f"[a3] the group repeats for {seconds:g}s, so being a few seconds off "
          f"does not matter.", flush=True)
    input("[a3] press Enter the instant you fit the battery ... ")

    dev.carrier(0, quiet=True)
    dev.pull(False)
    dev.carrier(1, quiet=True)
    time.sleep(0.01)

    rounds = 0
    try:
        if dev.sense():
            raise hc.Error("baseline already HIGH -- test board still attached "
                           "or the camera is powered. Nothing was sent.")

        print("[a3] ARMED: baseline low, carrier humming, streaming now.",
              flush=True)

        t0 = time.time()
        while time.time() - t0 < seconds:
            for i, rec in enumerate(recs):
                dev.blob(rec)
                print(f"\r[a3] round {rounds + 1}  {i + 1}/{len(recs)}  "
                      f"{time.time() - t0:5.1f}s", end="", flush=True)
                if i + 1 < len(recs):
                    time.sleep(gap)
            rounds += 1
            # Re-arm the firmware's 1.2 s starve guard between rounds.
            dev.send(hc.RpCmd.CARRIER, b"\x01")
            dev.ack_quiet("starved")
        elapsed = time.time() - t0
    finally:
        print()
        # The firmware may already have dropped the carrier on its own (starve
        # guard), in which case both of these answer ERR -- that is fine here.
        try:
            dev.carrier(0, quiet=True)
            dev.pull(False)
        except hc.Error:
            pass
    return rounds, elapsed


def send(dev, recs, args):
    gap = hc.RECORD_GAP

    def stream():
        for i, rec in enumerate(recs):
            dev.blob(rec)
            print(f"\r[a3] {i + 1}/{len(recs)}", end="", flush=True)
            if i + 1 < len(recs):
                time.sleep(gap)
        print()

    if args.stream:
        rounds, elapsed = _stream_loop(dev, recs, args.stream, gap)
        print(f"[a3] {rounds} rounds in {elapsed:.1f}s -- read the LED code")
    elif args.manual:
        print("[a3] 1) pull the camera battery  "
              "2) press Enter the same instant you re-insert it")
        input("[a3] insert battery + Enter NOW ... ")
        dev.carrier(1)
        time.sleep(args.delay)
        print(f"[a3] sending after {args.delay:g}s")
        stream()
        dev.carrier(0)
    elif args.cold:
        expect = None
        if args.expect.lower() != "none":
            v, _, p = args.expect.partition(":")
            expect = (int(v, 16), int(p, 16))
        detect = expect is not None and _load_iface_api() is not None

        print("[a3] COLD START: battery out, camera OFF, test board wired but")
        print("[a3] NOT attached to the camera yet.")
        print("[a3] The carrier is armed first; the camera itself reports when")
        print("[a3] its ROM is listening, and only then is the group sent once.")
        if args.probe:
            print("[a3] PROBE: 1 long + 3 short  = ROM was parsing (trial hit)")
            print("[a3]        normal boot       = ROM was not listening yet")
        if args.entry_test:
            print("[a3] ENTRY-TEST: 1 long + 5 short (0x2005) = group close +")
            print("[a3]        entry jump OK, no payload code ran")
            print("[a3]        1 long + 1 short (0x1001) = group closed with flag1=0")
            print("[a3]        1 long + 4 short (0x2004) = group never closed")
        if args.entry:
            print(f"[a3] LONG-TEST entry 0x{int(args.entry, 0):08X}: the full "
                  f"image is written, then ROM code runs")
            print("[a3]        1 long + 5 short (0x2005) = all records written "
                  "and CRC-OK")
            print("[a3]        1 long + 3 short (0x2003) = a record CRC differed "
                  "(bit sync / feed)")
            print("[a3]        1 long + 4 short (0x2004) = record too long or a "
                  "1.44 s per-record timeout")
            print("[a3]        1 long + 1 short (0x1001) = group never closed")
        input("[a3] press Enter to arm the carrier (nothing is sent yet) ... ")

        t_start = _cold_sense(dev, recs, args.wait)

        hit = False
        if detect:
            deadline = time.time() + args.cold_window
            while time.time() < deadline:
                if usb_present(*expect):
                    hit = True
                    break
                time.sleep(0.05)

        if hit:
            print(f"\n[a3] SUCCESS: {expect[0]:04X}:{expect[1]:04X} "
                  f"enumerated after {time.time() - t_start:.1f}s")
        elif expect is None:
            print(f"\n[a3] group sent after {time.time() - t_start:.1f}s "
                  f"-- read the LED code")
        else:
            print(f"\n[a3] gave up after {time.time() - t_start:.1f}s "
                  f"without seeing {args.expect}")
    else:
        dev.reset()
        dev.carrier(1)
        time.sleep(hc.TRANSMIT_START)
        stream()
        dev.carrier(0)


def probe_records():
    """Two SET_UNLOCK records, the first with a deliberately wrong CRC.

    0xffff0802 checks the record CRC before dispatching on the type, so a bad
    CRC is rejected with 0x2003 (1 long + 3 short) and nothing is written --
    not even the caller's unlock slot.  If the ROM is not yet listening the
    record group just passes by and the release produces 0x3002 -> 0x3001
    (normal boot), which makes this a clean "was the ROM parsing at time T?"
    probe.
    """
    good = hc.record(hc.SerCmd.SET_UNLOCK, hc.A7C_UNLOCK)
    bad = good[:-4] + struct.pack("<I", struct.unpack_from("<I", good, len(good) - 4)[0]
                                  ^ 0xFFFFFFFF)
    return [bad, good]


def entry_test_records():
    """[SET_ENTRY -> 0xFFFF1001, SET_UNLOCK, SET_ENTRY -> 0xFFFF1001].

    0xffff1000 is the ROM's own `movw r0,#0x2005; bl 0xffff0d0a`, i.e. flash
    0x2005 (1 long + 5 short) and never return.  Nothing of ours executes, so
    this is a pure read-only test of the chain "group closes -> flag1 set ->
    bx [0xFE00500C]":

      * 1 long + 5 short (0x2005) -> the chain works end to end.
      * 1 long + 1 short (0x1001) -> flag1 was 0 when the group closed.
      * 1 long + 4 short (0x2004) -> the group never closed (sync/phase).

    The record order is deliberate: SET_ENTRY first and last means every sync
    phase closes on a repeated ENTRY, and the SET_UNLOCK in the middle always
    sets flag0, so a replay boundary (ENTRY -> ENTRY) still closes with both
    flags set instead of falling into the flag1==0 -> 0x1001 path.
    """
    entry = 0xFFFF1001
    return [hc.record(hc.SerCmd.SET_ENTRY, entry),
            hc.record(hc.SerCmd.SET_UNLOCK, hc.A7C_UNLOCK),
            hc.record(hc.SerCmd.SET_ENTRY, entry)]


def main() -> int:
    ap = argparse.ArgumentParser(description="push an A3 image to the A7C")
    ap.add_argument("image", nargs="?", help="A3 image (omit with --probe)")
    ap.add_argument("--send", action="store_true",
                    help="actually transmit (default: print the plan only)")
    ap.add_argument("-p", "--port", help="COM port (default: autodetect RP2040)")
    ap.add_argument("-t", "--timeout", type=float, default=5.0)
    ap.add_argument("--chunk", type=int, default=MAX_CHUNK)
    ap.add_argument("--unlock", help="override the unlock code (still writes it)")
    ap.add_argument("--entry", help="override the SET_ENTRY target with a raw "
                                    "address, e.g. 0xFFFF1001: the whole image "
                                    "is still written, but the ROM then runs "
                                    "its own 1-long-5-short handler instead of "
                                    "the payload (read-only discriminator for "
                                    "the long-record path)")
    ap.add_argument("--probe", action="store_true",
                    help="send a bad-CRC SET_UNLOCK instead of an image: "
                         "1 long + 3 short means the ROM was parsing")
    ap.add_argument("--entry-test", action="store_true",
                    help="send [SET_ENTRY(ROM 0xFFFF1001), SET_UNLOCK, "
                         "SET_ENTRY] instead of an image: 1 long + 5 short means "
                         "group-close + entry jump work (no payload code runs)")
    ap.add_argument("--delay", type=float, default=hc.TRANSMIT_START,
                    help="seconds between battery insertion and the first record")
    ap.add_argument("--manual", action="store_true",
                    help="wait for Enter, then transmit once")
    ap.add_argument("--cold", action="store_true",
                    help="power-up path: arm the carrier, wait for the camera to "
                         "power up, sleep --delay, then send the group once.  One "
                         "group already fills ~1.35 s of the ROM's 1.45 s window, "
                         "so this only works if the timing happens to line up; "
                         "--stream is the reliable path")
    ap.add_argument("--stream", type=float, metavar="SECONDS", default=None,
                    help="power-up path: arm the carrier, then repeat the whole "
                         "group until SECONDS elapse (default carrier behaviour: "
                         "the battery may go in at any moment).  Stops on a timer, "
                         "not on USB enumeration -- the camera has a single port "
                         "and nothing can enumerate while the bridge is wired")
    ap.add_argument("--wait", type=float, default=900.0, metavar="SECONDS",
                    help="cold mode: how long to wait for the camera to power up "
                         "before giving up, sending nothing (default 900)")
    ap.add_argument("--cold-window", type=float, default=10.0,
                    help="cold mode: seconds to wait for --expect after the group "
                         "(default 10)")
    ap.add_argument("--q", type=int, default=hc.TIMING_Q,
                    help="quantum / bit cell in ns (default 20833, hardware-verified)")
    ap.add_argument("--slow", type=int, default=hc.TIMING_SLOW)
    ap.add_argument("--fast", type=int, default=hc.TIMING_FAST)
    ap.add_argument("--crc-init", type=lambda s: int(s, 0), default=0xFFFFFFFF,
                    metavar="VALUE",
                    help="diagnostic: preset the record CRC register with this "
                         "value instead of 0xFFFFFFFF (default), so every record "
                         "trips the ROM's 0x2003 path; used to tell 'ROM parsed "
                         "our records' apart from 'ROM never synced'")
    ap.add_argument("--cycles", type=int, default=1,
                    help="repeat the image this many times before SET_ENTRY "
                         "(default 1; >1 only adds extra MEM_WRITEs, which the "
                         "ROM accepts -- the group closes on the repeated "
                         "first type, not on adjacent same-type records)")
    ap.add_argument("--expect", default=None,
                    help="VID:PID that means the payload is running "
                         "(default 18d1:4ee0, or 'none' with --probe)")
    args = ap.parse_args()

    if not 1 <= args.wait <= 3600:
        ap.error("--wait must be 1..3600 seconds")

    if args.probe and args.entry_test:
        ap.error("--probe and --entry-test are mutually exclusive")
    if args.entry_test and args.image:
        ap.error("--entry-test takes no image")
    if args.entry and (args.probe or args.entry_test):
        ap.error("--entry only applies to an image push")

    if args.expect is None:
        args.expect = "none" if (args.probe or args.entry_test) else "18d1:4ee0"

    if args.entry_test:
        recs = entry_test_records()
        describe(recs, 0)
        entry = None
    elif args.probe:
        recs = probe_records()
        describe(recs, 0)
        entry = None
    else:
        if not args.image:
            ap.error("an image is required unless --probe is given")
        path = Path(args.image)
        image = path.read_bytes()
        a3.verify(image)

        unlock = int(args.unlock, 0) if args.unlock else None
        cycles = max(1, args.cycles)
        entry_override = int(args.entry, 0) if args.entry else None
        recs, entry = build_records(image, args.chunk, unlock, cycles,
                                    entry=entry_override,
                                    crc_init=args.crc_init)
        describe(recs, len(image))
        if args.crc_init != 0xFFFFFFFF:
            print(f"[a3] CRC preset 0x{args.crc_init:08X} -- every record is "
                  f"deliberately invalid (expect 0x2003 / 1 long 3 short)")
        print(f"[a3] entry 0x{entry:08X}")

    if not args.send:
        print("[a3] plan only -- pass --send to transmit")
        return 0

    port = hc.pick_port(args.port)
    dev = hc.Device(port, args.timeout)
    try:
        dev.ping()
        dev.timing(args.q, args.slow, args.fast)
        send(dev, recs, args)
    finally:
        dev.close()

    if not args.cold:
        print("[a3] done. Expect the camera to enumerate as 18d1:4ee0; "
              "a normal boot instead means the waveform missed the window.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
