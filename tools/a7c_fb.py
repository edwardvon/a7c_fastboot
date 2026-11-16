#!/usr/bin/env python3
"""Host side of the A7C payload's fastboot stub.

The payload implements `getvar`, `oem peek`, `oem storage`, `oem partition`,
`oem read`, `upload`, `oem armed`, and a `download`/`flash` pair that refuses
everything unless the injected image was built by build.ps1 -Write.  This tool
has no poke or exec verb.  `reboot` writes nothing: the payload answers OKAY,
drops the D+ pullup and parks with a light code, so the verb's real job is to
record what the host sees next.  `flash` is the only verb that programs the
card, and every image except the -Write test build rejects it outright.  What
it can do:

    python a7c_fb.py devices
    python a7c_fb.py info
    python a7c_fb.py getvar product
    python a7c_fb.py peek 0xf0304040:4 0xf0304024:4 0xf03040fe:4
    python a7c_fb.py storage
    python a7c_fb.py partition
    python a7c_fb.py dump nflasha23 0 0x100000 -o out\\nflasha23.bin
    python a7c_fb.py upload out\\nflasha23.bin
    python a7c_fb.py flash nflasha16_s0 -f out\\m7_before.bin
    python a7c_fb.py flash-part nflasha23 -f out\\m8_before.bin
    python a7c_fb.py armed
    python a7c_fb.py reboot
    python a7c_fb.py selftest

`peek` takes one or more `addr[:len]` pairs and runs them in a single device
session, which keeps the M-2 medium decision down to one open/close.

`storage` runs the payload's eMMC identification (CMD0/1/2/3/9/7/13 only --
there is no data path in the image) and prints the registers it read.

`partition` reads the SDM2 table from user-area LBA 0 with CMD17 (4 sectors,
read-only) and prints one line per slot.  Redirect it and diff against the
expected table:

    python a7c_fb.py partition > out\\parts.txt
    python tools\\sdm2_parse.py compare ..\\a7c_fw_key_artifacts\\partinf.conf ^
        --log out\\parts.txt

`dump` sends one `oem read:<name>:<off>[:<len>]` per 16 KiB piece.  Since P14
that single command arms the slice and streams it back (DATA<total>, the
bytes, OKAY), so a piece no longer pays for a separate `upload` round trip.
A slice longer than one 16 KiB transfer is still exported one piece at a
time, because the payload's second `usb_bulk_send()` inside one export never
returns (round 88: up_stage=2, up_uerr=0x0d, up_xfer=0; rounds 97/98: 24 MHz
breaks on the 3rd or 4th 16 KiB chunk and may take the whole device down).
Keep `--slice` at or below 16384.  `--single` restores the one-shot
path and `--slice N` changes the piece size.  `upload` on its own streams
whatever the payload is already armed with -- the arm lives in payload memory,
so it survives across host invocations but disappears when the payload resets.
`armed` reports where the last upload stopped, which is how a truncated stream
is told apart from a card read that timed out.

A dead piece no longer costs the whole file.  A broken bulk transfer detaches
the camera for 2-3 s; it then re-enumerates with the same payload state (round
98 / P4: `oem armed` still reported arm_lba/arm_n and `oem darwin` still
answered, so no second injection is needed).  The exporter therefore closes the
dead session, sleeps `--retry-wait` and re-arms the same offset up to
`--retries` times.  `--resume` restarts from the length of an existing output
file instead of truncating it, so an interrupted run costs at most one piece.

`flash <zone> -f <file>` is the M-7 verb: one `download:<hex len>`, the bytes,
then `flash:<zone>`.  It only speaks to the zones the -Write image carries in
payload/src/fastboot.c's whitelist (`nflasha16_s0` = LBA 2105344, one 512-byte sector),
and it only reads the payload from under the payload directory.  The image
writes that sector, reads it back through CMD17 and compares -- so OKAY means
the card returned exactly the bytes that were sent, and FAIL carries
`w_n/w_err/v_bad/err` INFO lines that say which half broke.

`flash-part <zone> -f <file>` is the M-8 verb and does the same for a whole
zone, one 16 KiB piece at a time: `download:<hex len>` then
`flash:<zone>@<hex off>`.  Pieces are ordered, so a failure leaves a prefix
written and the rest untouched -- the file on the host stays the authoritative
copy, and `dump` can prove the two agree afterwards.  The host refuses a size
that is not a whole number of sectors or does not fit the zone (listed in
ZONE_BYTES, which mirrors write_zones[] in payload/src/fastboot.c) before it opens the
camera at all.

`selftest` needs no camera: it drives the framing helpers with a fake endpoint
pair and checks the 16 KiB chunk boundaries, the unarmed FAIL, the truncation
path and the download/flash handshake.

Everything this tool writes to disk lands under the payload directory; the one
command that writes to the camera is `flash`, gated twice on the device.

On Windows the device must be bound to WinUSB or libusbK (Zadig) before libusb
can open it; a bare `usb.core.find` hit with `open()` failing means exactly
that.
"""
import argparse
import collections
import contextlib
import io
import os
import sys
import time
from pathlib import Path

import usb.core
import usb.util

VID = 0x18d1
PID = 0x4ee0
READ_SIZE = 64           # payload replies are capped at FB_RESPONSE_MAX
UPLOAD_READ = 16384      # must match UPLOAD_CHUNK in payload/src/fastboot.c
DOWNLOAD_MAX = 16384     # must match DOWNLOAD_MAX in payload/src/fastboot.c

ROOT = Path(__file__).resolve().parent.parent

# pyusb versions differ on whether the timeout subclass exists.
USB_TIMEOUT = getattr(usb.core, 'USBTimeoutError', usb.core.USBError)


def get_backend():
    """Prefer the bundled libusb DLL; fall back to whatever pyusb finds."""
    try:
        import libusb_package
        return libusb_package.get_libusb1_backend()
    except Exception:
        pass
    try:
        return usb.backend.libusb1.get_backend()
    except Exception:
        return None


# ---- framing (module level: testable without a device) --------------------

def read_reply(ep_in, timeout, on_info=print):
    """Read one reply; print INFO lines; return (status, message)."""
    while True:
        resp = bytes(ep_in.read(READ_SIZE, timeout=timeout))
        if len(resp) < 4:
            return 'BAD', resp
        tag, msg = resp[:4], resp[4:]
        text = msg.decode('utf-8', 'replace').rstrip('\x00')
        if tag == b'INFO':
            on_info(text)
            continue
        if tag in (b'OKAY', b'FAIL'):
            return tag.decode(), text
        return tag.decode('latin-1'), text


def clear_halt(ep):
    """Drop ENDPOINT_HALT on one endpoint; True when the request went out.

    The payload answers CLEAR_FEATURE with a bare OK (payload/src/usb.c), so
    this may not reach the device state at all -- it still releases the host
    side's halt latch, which is what makes the retry in write_out() worth one
    attempt.
    """
    dev = getattr(ep, 'device', None) or getattr(ep, 'dev', None)
    if dev is None:
        return False
    try:
        dev.clear_halt(ep)
        return True
    except Exception:
        pass
    try:
        dev.ctrl_transfer(0x02, 0x01, 0x0000, ep.bEndpointAddress, None)
        return True
    except Exception:
        return False


def write_out(ep_out, data, timeout, on_info=print):
    """Write one bulk OUT transfer; on EPIPE clear the halt and retry once.

    Ledger 5.59: after a failed upload, three of five retries died in
    ep_out.write() with Errno 32 while EP_IN was merely short -- whether that
    stall is host-side or device-side is exactly what this retry separates.
    """
    try:
        return ep_out.write(data, timeout=timeout)
    except usb.core.USBError as exc:
        if getattr(exc, 'errno', None) != 32:
            raise
        first = exc

    on_info('INFO EPIPE on bulk OUT -> clear_halt, retrying once')
    if not clear_halt(ep_out):
        raise first
    return ep_out.write(data, timeout=timeout)


def send_command(ep_out, ep_in, cmd, timeout, on_info=print):
    """Write one command and read its reply, printing any INFO lines."""
    if isinstance(cmd, str):
        cmd = cmd.encode()
    write_out(ep_out, cmd, timeout, on_info)
    return read_reply(ep_in, timeout, on_info)


def receive_upload(ep_out, ep_in, out, timeout, on_info=print, cmd=b'upload'):
    """Send `cmd` and stream its DATA payload into `out`.

    `cmd` is `upload` (replay whatever is armed) or, since P14, the arm itself
    (`oem read:...`), which now answers DATA directly instead of OKAY.

    Returns (status, message, received, total).  The payload cannot answer FAIL
    once DATA is on the wire, so a device that dies mid-stream just stops; the
    only way to see that is the byte count, reported as status 'SHORT'.

    Every read asks for at most UPLOAD_READ: the payload queues one chunk per
    bulk transfer, and a read that spans two transfers would sit until the
    timeout.
    """
    if isinstance(cmd, str):
        cmd = cmd.encode()
    write_out(ep_out, cmd, timeout, on_info)
    status, msg = read_reply(ep_in, timeout, on_info)
    if status != 'DATA':
        return status, msg, 0, 0

    total = int(msg or '0', 16)
    got = 0

    while got < total:
        want = min(UPLOAD_READ, total - got)

        while want:
            try:
                data = bytes(ep_in.read(want, timeout=timeout))
            except usb.core.USBError as exc:
                return ('SHORT', '%d of %d bytes (%s)' % (got, total, exc),
                        got, total)
            if not data:
                return 'SHORT', '%d of %d bytes' % (got, total), got, total
            out.write(data)
            got += len(data)
            want -= len(data)

    # The stream ends with a 4-byte OKAY (payload/src/fastboot.c: up_stage 3 ->
    # fb_okay("")).  Leaving it unread parks a TRB the controller still owns:
    # 0.5 s later the payload latches a bulk IN deadline, waits out its 1.5 s
    # late window and aborts -- a 2 s tail plus a false dg_hwo=1 in `oem armed`.
    # Drain it here; a missing tail does not undo the bytes already written.
    try:
        status, msg = read_reply(ep_in, min(timeout, 2000), on_info)
    except usb.core.USBError as exc:
        return 'OKAY', 'tail missing (%s)' % exc, got, total
    if status != 'OKAY':
        return status, msg, got, total

    return 'OKAY', '', got, total


def check_output_path(path):
    """Workspace rule: exports stay under the payload directory."""
    p = Path(path).resolve()
    root = os.path.normcase(str(ROOT))
    here = os.path.normcase(str(p))
    if not (here == root or here.startswith(root + os.sep)):
        raise SystemExit('%s: refusing to write outside %s' % (path, ROOT))
    if not p.parent.is_dir():
        raise SystemExit('%s: directory does not exist' % p.parent)
    return p


def check_input_path(path):
    """Workspace rule: a file sent to the camera comes from the payload dir."""
    p = Path(path).resolve()
    root = os.path.normcase(str(ROOT))
    here = os.path.normcase(str(p))
    if not here.startswith(root + os.sep):
        raise SystemExit('%s: refusing to read from outside %s' % (path, ROOT))
    if not p.is_file():
        raise SystemExit('%s: not a file' % p)
    return p


def send_download(ep_out, ep_in, data, timeout, on_info=print):
    """Send `download:<hex len>` then the bytes; return (status, message).

    The payload answers DATA<8 hex> before it collects anything, so the bytes
    must not go out until that arrives.  It then reads exactly `want` bytes and
    answers OKAY/FAIL -- a FAIL here means the buffer was never staged, so
    `flash:` afterwards would write nothing (the payload keeps `download_len`
    from the last successful download).
    """
    status, msg = send_command(ep_out, ep_in, 'download:%x' % len(data),
                               timeout, on_info)
    if status != 'DATA':
        return status, msg
    want = int(msg or '0', 16)
    if want != len(data):
        return 'FAIL', 'payload asked for %d bytes, not %d' % (want, len(data))
    write_out(ep_out, data, timeout, on_info)
    return read_reply(ep_in, timeout, on_info)


def list_devices(out=print):
    """Print every 18d1:4ee0 without opening it; returns how many were seen."""
    backend = get_backend()
    try:
        found = usb.core.find(find_all=True, idVendor=VID, idProduct=PID,
                               backend=backend)
    except usb.core.USBError as exc:
        raise SystemExit('cannot enumerate USB devices: %s' % exc)

    count = 0
    for dev in found:
        count += 1
        out('%04x:%04x bus=%s addr=%s' % (dev.idVendor, dev.idProduct,
                                          dev.bus, dev.address))
    if not count:
        out('no 18d1:4ee0 device')
    return count


class Device:
    def __init__(self, timeout=5000):
        self.timeout = timeout
        backend = get_backend()
        self.dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
        if self.dev is None:
            raise SystemExit(
                'no A7C fastboot device (18d1:4ee0) -- inject the payload first'
            )

        # Never issue a second SET_CONFIGURATION: the payload arms its bulk
        # endpoints from that request and re-sending it has hung EP0 before.
        # Only fall back to one if the host somehow left the device unconfigured.
        try:
            cfg = self.dev.get_active_configuration()
        except usb.core.USBError:
            try:
                self.dev.set_configuration()
            except usb.core.USBError as exc:
                raise SystemExit('device found but cannot be configured: %s' % exc)
            cfg = self.dev.get_active_configuration()

        self.intf = cfg[(0, 0)]
        try:
            usb.util.claim_interface(self.dev, self.intf)
        except usb.core.USBError as exc:
            raise SystemExit(
                'found 18d1:4ee0 but cannot open it (%s).\n'
                'On Windows bind WinUSB/libusbK to this device with Zadig first.'
                % exc
            )

        self.ep_out = usb.util.find_descriptor(
            self.intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
            == usb.util.ENDPOINT_OUT)
        self.ep_in = usb.util.find_descriptor(
            self.intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
            == usb.util.ENDPOINT_IN)
        if self.ep_out is None or self.ep_in is None:
            raise SystemExit('bulk endpoints missing from the active configuration')

    def close(self):
        # Both calls can throw once the device has detached; a close that
        # raises must never hide the transfer error that caused it.
        try:
            usb.util.release_interface(self.dev, self.intf)
        except Exception:
            pass
        try:
            usb.util.dispose_resources(self.dev)
        except Exception:
            pass

    def command(self, cmd):
        """Send one command; print INFO lines; return (status, message)."""
        return send_command(self.ep_out, self.ep_in, cmd, self.timeout)

    def upload(self, out):
        """Replay what is already armed (`upload`)."""
        return receive_upload(self.ep_out, self.ep_in, out, self.timeout)

    def piece(self, spec, out):
        """Arm one slice and stream it: P14 merges arm and download."""
        return receive_upload(self.ep_out, self.ep_in, out, self.timeout,
                              cmd=spec)

    def download(self, data):
        """Stage `data` in the payload's download buffer (`download:<len>`)."""
        return send_download(self.ep_out, self.ep_in, data, self.timeout)


# Same windows the payload's peek_allowed() accepts.
SAFE_WINDOWS = (
    (0xF0000000, 0x01000000),   # peripherals: DWC3, SDHCI
    (0xF2000000, 0x01000000),   # timer, USB PHY, aux block
    (0xF5000000, 0x01000000),   # NAND controller
    (0xF8000000, 0x00100000),   # system controller: +0x620 timer, +0x628 reason
    (0xFE000000, 0x00040000),   # eSRAM window
)
PEEK_MAX = 16

SECTOR = 512


def parse_peek(text):
    """Validate one `addr[:len]` the way the payload will, so a typo dies on
    the host instead of turning into an MMIO abort on the camera."""
    head, _, tail = text.partition(':')
    addr = int(head, 0)
    length = int(tail, 0) if tail else 4
    if addr < 0 or addr > 0xFFFFFFFF:
        raise SystemExit('%s: address is wider than 32 bits' % text)
    addr &= ~3
    length = (length + 3) & ~3
    if length == 0 or length > PEEK_MAX:
        raise SystemExit('%s: length must be 4..%d bytes' % (text, PEEK_MAX))
    if not any(lo <= addr and addr + length <= lo + size
               for lo, size in SAFE_WINDOWS):
        raise SystemExit('%s: outside the payload whitelist' % text)
    return addr, length


def parse_slice(name, off, length):
    """Format `oem read:` the way the payload parses it: hex, 512-aligned."""
    if off < 0 or (length is not None and length <= 0):
        raise SystemExit('offset/length must be positive')
    if off % SECTOR or (length is not None and length % SECTOR):
        raise SystemExit('offset/length must be multiples of %d' % SECTOR)
    spec = 'oem read:%s:0x%x' % (name, off)
    if length is not None:
        spec += ':0x%x' % length
    return spec


def say(text):
    """Print a line the console can always encode.

    A fastboot reply is bytes off the wire: after a stalled transfer the payload
    can answer with something that is not valid text, and printing it used to
    raise UnicodeEncodeError under the Windows GBK console -- the run then exited
    1 (or died mid-command) even though the failure had already been observed.
    """
    enc = sys.stdout.encoding or 'utf-8'
    sys.stdout.write(text.encode(enc, 'backslashreplace').decode(enc) + '\n')


def run(dev, cmd):
    try:
        status, msg = dev.command(cmd)
    except USB_TIMEOUT as exc:
        print('TIMEOUT: device present but silent (%s)' % exc)
        print('         the camera detaches for 2-3 s after a broken transfer; '
              'wait and retry')
        return False
    except usb.core.USBError as exc:
        print('USB: %s' % exc)
        return False
    if status == 'BAD':
        print('EMPTY: reply was %d byte(s) %r -- the device may have detached'
              % (len(msg), bytes(msg)))
        return False
    say('%-4s %s' % (status, msg))
    return status == 'OKAY'


def observe_reboot(settle=2.0):
    """Report what the host sees after `reboot` -- the P7 observation, zero writes.

    The payload answers OKAY, drops the D+ pullup and parks with light code 0x20
    ([fastboot.c L1109-L1118]); it does not touch any reset register.  So the
    expected result is 'device gone'.  A device that comes back means the SoC
    really reset and the ROM re-enumerated, which is the opposite finding.
    """
    time.sleep(settle)
    print('--- %0.1f s later ---' % settle)
    if list_devices():
        print('device is back -> the SoC really reset and the ROM re-enumerated')
        return 0
    print('device is gone -> expected: D+ dropped and the payload is parked '
          '(check the light code; P7 predicts 0x20)')
    return 0


def stream(dev, arm_cmd, path):
    """Arm (optional), then stream into `path`. 0 = full, 2 = truncated."""
    path = check_output_path(path)

    with open(path, 'wb') as fh:
        try:
            if arm_cmd is not None:
                # P14: `oem read:` arms and streams in one command.
                status, msg, got, total = dev.piece(arm_cmd, fh)
            else:
                status, msg, got, total = dev.upload(fh)
        except usb.core.USBError as exc:
            print('USB: %s' % exc)
            print('truncated at %d bytes -> %s (see `armed`)' % (fh.tell(), path))
            return 2

    if status == 'SHORT':
        say('SHORT %s' % msg)
        print('truncated at %d of %d bytes -> %s (see `armed`)'
              % (got, total, path))
        return 2
    if status != 'OKAY':
        say('%-4s %s' % (status, msg))
        return 1

    print('OKAY %d bytes -> %s' % (got, path))
    if msg:
        say('note: %s' % msg)
    return 0


def reopen_device(old, open_dev, wait=3.0, tries=3):
    """Drop a dead session, wait out the detach window, open a fresh one.

    P4 / round 98: a broken bulk transfer takes the camera off the bus and it
    re-enumerates 2-3 s later with the payload still armed, so a plain re-open
    is enough -- the payload never resets and must not be re-injected.
    """
    close = getattr(old, 'close', None)
    if close is not None:
        try:
            close()
        except Exception:
            pass

    last = None
    for _ in range(tries):
        time.sleep(wait)
        try:
            return open_dev()
        except SystemExit as exc:          # find() saw no device yet
            last = exc
    raise SystemExit('device did not come back after %d x %.1f s (%s)'
                     % (tries, wait, last))


def stream_slices(name, off, length, path, piece=UPLOAD_READ, timeout=30000,
                  retries=3, retry_wait=3.0, resume=False, open_dev=None):
    """Export `length` bytes as one arm + upload per `piece` bytes.

    The payload's second `usb_bulk_send()` in the same export never comes back
    (round 88: up_stage=2, up_xfer=0, up_uerr=0x0d), so every piece is armed
    and uploaded on its own and the file is appended across pieces.

    A piece that dies is re-armed on the same offset up to `retries` times,
    each attempt after `reopen_device()` has waited out the detach window.
    `resume` starts from the length of an existing `path` instead of cutting it
    back to zero.

    0 = complete, 1 = arm refused, 2 = truncated.
    """
    if piece < SECTOR or piece % SECTOR:
        raise SystemExit('--slice must be a positive multiple of %d' % SECTOR)
    path = check_output_path(path)
    open_dev = open_dev or (lambda: Device(timeout))

    done, mode = 0, 'wb'
    if resume and path.exists():
        have = path.stat().st_size
        if have % SECTOR or have > length:
            raise SystemExit('--resume: %s holds %d bytes, which is not a '
                             '512-byte point inside a %d-byte export'
                             % (path, have, length))
        if have:
            done, mode = have, 'r+b'
            print('resume at +0x%x: %d of %d bytes already in %s'
                  % (off + have, have, length, path))

    t0 = time.time()

    def rate(nbytes):
        """Wall-clock seconds and ms per 512 B block, for the ledger."""
        elapsed = time.time() - t0
        blocks = max(1, nbytes // SECTOR)
        return '%.1f s, %.1f ms/block' % (elapsed, elapsed * 1000.0 / blocks)

    if done >= length:
        print('OKAY %d bytes -> %s (already complete)' % (done, path))
        return 0

    dev = open_dev()
    try:
        with open(path, mode) as fh:
            fh.seek(done)
            while done < length:
                want = min(piece, length - done)
                spec = parse_slice(name, off + done, want)
                attempt = 0

                while True:
                    got, total = 0, want
                    try:
                        # P14: one command per piece -- `oem read:` arms and
                        # streams, so there is no second `upload` round trip.
                        status, msg, got, total = dev.piece(spec, fh)
                    except usb.core.USBError as exc:
                        status, msg = 'SHORT', str(exc)

                    if status == 'OKAY' and got == want:
                        break
                    if status == 'FAIL':
                        say('%-4s %s' % (status, msg))
                        print('arm refused at +0x%x (%d of %d bytes) -> %s'
                              % (off + done, done, length, path))
                        return 1

                    # Roll the partial piece back so a retry does not append
                    # the same bytes twice.
                    fh.truncate(done)
                    fh.seek(done)
                    problem = ('%s %s' % (status, msg) if status != 'OKAY'
                               else '%d of %d bytes' % (got, want))
                    if attempt >= retries:
                        say('SHORT %s' % problem)
                        print('truncated at %d of %d bytes -> %s (%s, see `armed`)'
                              % (done, length, path, rate(done)))
                        return 2
                    attempt += 1
                    print('piece +0x%06x: %s -- retry %d/%d in %.0f s'
                          % (off + done, problem, attempt, retries, retry_wait))
                    dev = reopen_device(dev, open_dev, retry_wait)

                done += got
                print('piece +0x%06x %6d B OK  %d/%d'
                      % (off + done - got, got, done, length))
    finally:
        close = getattr(dev, 'close', None)
        if close is not None:
            try:
                close()
            except Exception:
                pass

    print('OKAY %d bytes -> %s (%s)' % (done, path, rate(done)))
    return 0


def flash_zone(dev, zone, path, timeout=30000):
    """Stage one file, then `flash:<zone>`; 0 = both answered OKAY.

    The file must be whole sectors and no larger than the payload's download
    buffer; the zone must be the one name the injected image carries.  Every
    check the host can make happens here, because the device can only refuse
    what it is told -- and refusing after the buffer was staged still costs
    nothing, since nothing is programmed until `flash:` runs.
    """
    src = check_input_path(path)
    data = src.read_bytes()
    if not data or len(data) % SECTOR or len(data) > DOWNLOAD_MAX:
        raise SystemExit('%s: %d bytes, want 512..%d in 512-byte steps'
                         % (src, len(data), DOWNLOAD_MAX))

    status, msg = dev.download(data)
    if status != 'OKAY':
        say('%-4s %s' % (status, msg))
        print('download refused -> nothing was written')
        return 1
    print('download OK: %d bytes (%d sector%s) staged'
          % (len(data), len(data) // SECTOR,
             '' if len(data) == SECTOR else 's'))

    if not run(dev, 'flash:' + zone):
        return 1
    print('zone %s: the image wrote the sector, read it back through CMD17 '
          'and compared it -- OKAY above is that comparison' % zone)
    return 0


# Zone name -> bytes, mirroring write_zones[] in payload/src/fastboot.c.  Keeping the
# two lists in step is what turns "the device refused" into "the host never
# asked": a size that cannot fit is rejected before the camera is opened.
ZONE_BYTES = {
    'nflasha16_s0': SECTOR,     # M-7: one sector at LBA 2105344
    'nflasha23': 2 << 20,       # M-8: 4096 sectors at LBA 2637824
}


def flash_part(dev, zone, path, timeout=30000, chunk=DOWNLOAD_MAX):
    """Write a whole zone back in pieces of at most `chunk` bytes (M-8).

    `flash:<zone>@<hex off>` exists for ranges larger than the payload's 16 KiB
    buffer: each piece gets its own `download:` and is committed at its offset
    inside the zone, where the image writes it, reads it back through CMD17 and
    compares before answering OKAY.  Pieces run in order, so a failure leaves a
    prefix written and the rest untouched -- and the host file is still the
    authoritative copy, because nothing here deletes it.
    """
    if zone not in ZONE_BYTES:
        raise SystemExit('%s: not a zone this tool knows (%s) -- add it to '
                         'ZONE_BYTES and payload/src/fastboot.c together'
                         % (zone, ', '.join(sorted(ZONE_BYTES))))
    src = check_input_path(path)
    data = src.read_bytes()
    size = ZONE_BYTES[zone]
    if not data or len(data) % SECTOR or len(data) > size:
        raise SystemExit('%s: %d bytes, want 512..%d in 512-byte steps'
                         % (src, len(data), size))

    total = (len(data) + chunk - 1) // chunk
    print('zone %s: %d bytes in %d piece%s of at most %d B'
          % (zone, len(data), total, '' if total == 1 else 's', chunk))

    for i in range(total):
        off = i * chunk
        piece = data[off:off + chunk]
        status, msg = dev.download(piece)
        if status != 'OKAY':
            say('%-4s %s' % (status, msg))
            print('piece %d/%d at +0x%06x refused -> nothing written at or '
                  'past 0x%06x' % (i + 1, total, off, off))
            return 1
        if not run(dev, 'flash:%s@%x' % (zone, off)):
            print('piece %d/%d at +0x%06x failed -> stopped; the host file is '
                  'still the authoritative copy' % (i + 1, total, off))
            return 1
        print('piece %d/%d +0x%06x %6d B OK' % (i + 1, total, off, len(piece)))

    print('OKAY %d bytes -> zone %s (%d piece%s)'
          % (len(data), zone, total, '' if total == 1 else 's'))
    return 0


# ---- selftest: no camera, fake endpoints ---------------------------------

class FakePayload:
    """Stand-in device: enough of the fastboot side to test the host.

    Replies are queued one bulk transfer at a time, and upload data is served
    in 16 KiB transfers, so a host read that straddled two transfers would be
    caught here rather than on the camera.
    """

    def __init__(self, budget=None):
        self.frames = collections.deque()
        self.current = b''
        self.armed = 0
        self.upload_left = 0
        self.sent = 0
        self.budget = budget            # bytes this fake will actually send
        self.commands = []
        self.max_read = 0
        self.download_left = 0          # bytes still expected for `download:`
        self.downloaded = bytearray()   # what the host actually staged

    def write(self, data, timeout=None):
        raw = bytes(data)
        if self.download_left:
            # Mid-download every write is payload data, not a command.
            take = min(self.download_left, len(raw))
            self.downloaded += raw[:take]
            self.download_left -= take
            if not self.download_left:
                self.frames.append(b'OKAY')   # buffer staged (cmd_download)
            return len(raw)

        text = raw.decode('latin-1').strip('\x00')
        self.commands.append(text)

        if text.startswith('getvar:product'):
            self.frames.append(b'OKAYILCE-7C')
        elif text.startswith('download:'):
            self.downloaded = bytearray()
            self.download_left = int(text[len('download:'):], 16)
            self.frames.append(b'DATA%08x' % self.download_left)
        elif text.startswith('flash:'):
            if self.download_left or not self.downloaded:
                self.frames.append(b'FAILnothing staged')
            else:
                self.frames.append(b'OKAYwrite+verify ok')
        elif text.startswith('oem read:'):
            name, _, rest = text[len('oem read:'):].partition(':')
            off, _, length = rest.partition(':')
            self.armed = int(length, 16) if length else 0
            # P14: the arm itself answers DATA -- no INFO line, no `upload`.
            self.frames.append(b'DATA%08x' % self.armed)
            self.upload_left = self.armed
            self.sent = 0
        elif text == 'upload':
            if not self.armed:
                self.frames.append(b'FAILnothing armed')
            else:
                self.frames.append(b'DATA%08x' % self.armed)
                self.upload_left = self.armed
                self.sent = 0
        else:
            self.frames.append(b'FAILunknown command')
        return len(data)

    def read(self, size, timeout=None):
        self.max_read = max(self.max_read, size)

        if not self.current:
            if self.frames:
                self.current = self.frames.popleft()
            elif self.upload_left:
                if size > UPLOAD_READ:
                    raise AssertionError(
                        'host asked for %d B, the chunk is %d B'
                        % (size, UPLOAD_READ))
                if self.budget is not None and self.sent >= self.budget:
                    raise USB_TIMEOUT('fake: stopped after %d B' % self.sent)
                n = min(size, self.upload_left, UPLOAD_READ)
                if self.budget is not None:
                    n = min(n, self.budget - self.sent)
                self.current = b'\x5a' * n
                self.upload_left -= n
                self.sent += n
                if self.upload_left == 0:
                    self.frames.append(b'OKAY')     # the payload's terminal reply
            else:
                raise USB_TIMEOUT('fake: nothing queued')

        n = min(size, len(self.current))
        data, self.current = self.current[:n], self.current[n:]
        return data


def selftest():
    checks = []

    def check(name, ok, detail=''):
        checks.append((name, bool(ok), detail))

    # 1. plain framing: one OKAY, message intact
    fake = FakePayload()
    status, msg = send_command(fake, fake, 'getvar:product', 1000,
                               on_info=lambda s: None)
    check('getvar framing', status == 'OKAY' and msg == 'ILCE-7C',
          '%s %r' % (status, msg))

    # 2. upload with nothing armed must FAIL, not stream
    fake = FakePayload()
    status, msg, got, total = receive_upload(fake, fake, _Sink(),
                                             1000, on_info=lambda s: None)
    check('unarmed upload FAILs', status == 'FAIL' and got == 0,
          '%s %r' % (status, msg))

    # 3. whole 16 KiB chunk: one `oem read:` arms and streams it
    for size in (UPLOAD_READ, UPLOAD_READ + 1024):
        fake = FakePayload()
        sink = _Sink()
        arm = parse_slice('nflasha23', 0, size)
        status, msg, got, total = receive_upload(fake, fake, sink, 1000,
                                                 on_info=lambda s: None, cmd=arm)
        check('piece %d B (%d chunk%s)'
              % (size, (size + UPLOAD_READ - 1) // UPLOAD_READ,
                 '' if size <= UPLOAD_READ else 's'),
              status == 'OKAY' and got == size and len(sink.data) == size
              and total == size and fake.max_read <= UPLOAD_READ
              and not fake.frames,
              '%s got=%d total=%d maxread=%d left=%d'
              % (status, got, total, fake.max_read, len(fake.frames)))

    # 4. device dies mid-stream -> SHORT with the exact count
    size, budget = 3 * UPLOAD_READ, UPLOAD_READ + 1024
    fake = FakePayload(budget=budget)
    sink = _Sink()
    status, msg, got, total = receive_upload(
        fake, fake, sink, 1000, on_info=lambda s: None,
        cmd=parse_slice('nflasha23', 0, size))
    check('truncation is SHORT', status == 'SHORT' and got == budget
          and total == size and len(sink.data) == budget,
          '%s got=%d total=%d' % (status, got, total))

    # 5. one EPIPE on bulk OUT -> clear_halt, then the write is retried once
    fake = FakePayload()
    stall = _Stall(fake)
    status, msg = send_command(stall, fake, 'getvar:product', 1000,
                               on_info=lambda s: None)
    check('EPIPE -> clear_halt -> retry',
          status == 'OKAY' and msg == 'ILCE-7C'
          and stall.halts == 1 and stall.writes == 2,
          '%s %r halts=%d writes=%d'
          % (status, msg, stall.halts, stall.writes))

    # 6. devices lists the payload's IDs without opening anything
    real_find = usb.core.find

    class _Dev:
        idVendor, idProduct, bus, address = VID, PID, 1, 7

    usb.core.find = lambda **kw: [_Dev()] if kw.get('find_all') else _Dev()
    lines = []
    try:
        list_devices(out=lines.append)
    finally:
        usb.core.find = real_find
    check('devices lists 18d1:4ee0', any('18d1:4ee0' in l for l in lines),
          repr(lines))

    # 7. a slice longer than one chunk is exported as one arm + upload per piece
    out = ROOT / 'out' / '_selftest_slice.bin'
    fake = FakePayload()
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            code = stream_slices('nflasha23', 0, 3 * UPLOAD_READ, out,
                                 retries=0, open_dev=lambda: _FakeDev(fake))
        size = out.stat().st_size
        arms = sum(1 for c in fake.commands if c.startswith('oem read:'))
        check('sliced export: 3 arms, 1 chunk each',
              code == 0 and size == 3 * UPLOAD_READ and arms == 3
              and fake.max_read <= UPLOAD_READ and not fake.frames,
              'code=%s size=%d arms=%d maxread=%d left=%d'
              % (code, size, arms, fake.max_read, len(fake.frames)))

        # 8. a piece that breaks stops the export -- no further arm, and the
        #    partial piece is cut back out of the file
        fake = FakePayload(budget=UPLOAD_READ // 16)
        with contextlib.redirect_stdout(io.StringIO()):
            code = stream_slices('nflasha23', 0, 3 * UPLOAD_READ, out,
                                 retries=0, open_dev=lambda: _FakeDev(fake))
        size = out.stat().st_size
        arms = sum(1 for c in fake.commands if c.startswith('oem read:'))
        check('sliced export drops a short piece entirely',
              code == 2 and size == 0 and arms == 1,
              'code=%s size=%d arms=%d' % (code, size, arms))

        # 9. a dead piece is retried: fresh open, same offset, no duplicates
        flaky = _FlakyDev(fails=2)
        with contextlib.redirect_stdout(io.StringIO()):
            code = stream_slices('nflasha23', 0, 2 * UPLOAD_READ, out,
                                 retries=3, retry_wait=0,
                                 open_dev=lambda: flaky)
        size = out.stat().st_size
        check('short piece -> reopen -> same offset re-armed',
              code == 0 and size == 2 * UPLOAD_READ and flaky.attempts == 4,
              'code=%s size=%d attempts=%d' % (code, size, flaky.attempts))

        # 10. --resume keeps what the interrupted run already wrote
        out.write_bytes(b'\xa5' * UPLOAD_READ)
        fake = FakePayload()
        with contextlib.redirect_stdout(io.StringIO()):
            code = stream_slices('nflasha23', 0, 3 * UPLOAD_READ, out,
                                 retries=0, resume=True,
                                 open_dev=lambda: _FakeDev(fake))
        size = out.stat().st_size
        arms = sum(1 for c in fake.commands if c.startswith('oem read:'))
        check('--resume keeps the first piece and arms only the rest',
              code == 0 and size == 3 * UPLOAD_READ and arms == 2
              and out.read_bytes()[:UPLOAD_READ] == b'\xa5' * UPLOAD_READ,
              'code=%s size=%d arms=%d' % (code, size, arms))

        # 11. download stage + flash: the bytes go out only after DATA, and the
        #     zone receives exactly what the file held
        payload = b'\x9c' * SECTOR
        out.write_bytes(payload)
        fake = FakePayload()
        with contextlib.redirect_stdout(io.StringIO()):
            code = flash_zone(_FakeDev(fake), 'nflasha16_s0', out)
        check('download then flash stages the file',
              code == 0 and bytes(fake.downloaded) == payload
              and fake.commands[:2] == ['download:200', 'flash:nflasha16_s0'],
              'code=%s staged=%d cmds=%r'
              % (code, len(fake.downloaded), fake.commands[:2]))

        # 12. a partial sector dies on the host -- the device never sees it
        out.write_bytes(b'\x00' * (SECTOR - 1))
        fake = FakePayload()
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                flash_zone(_FakeDev(fake), 'nflasha16_s0', out)
            refused = False
        except SystemExit:
            refused = True
        check('partial sector refused before the device',
              refused and not fake.commands,
              'refused=%s cmds=%r' % (refused, fake.commands))

        # 13. flash-part: one download + one offset-anchored flash per piece,
        #     in order, and the last piece is what the file ends with
        payload = bytes(range(256)) * (3 * DOWNLOAD_MAX // 256)
        out.write_bytes(payload)
        fake = FakePayload()
        with contextlib.redirect_stdout(io.StringIO()):
            code = flash_part(_FakeDev(fake), 'nflasha23', out, chunk=DOWNLOAD_MAX)
        want = []
        for off in (0, DOWNLOAD_MAX, 2 * DOWNLOAD_MAX):
            want += ['download:%x' % DOWNLOAD_MAX, 'flash:nflasha23@%x' % off]
        check('flash-part: 3 ordered download+flash@off pairs',
              code == 0 and fake.commands == want
              and bytes(fake.downloaded) == payload[2 * DOWNLOAD_MAX:],
              'code=%s cmds=%r' % (code, fake.commands))

        # 14. a size the zone cannot hold, and a zone the tool does not know,
        #     are both refused before the camera is opened
        for zone, size in (('nflasha99', SECTOR), ('nflasha16_s0', 2 * SECTOR)):
            out.write_bytes(b'\x00' * size)
            fake = FakePayload()
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    flash_part(_FakeDev(fake), zone, out)
                refused = False
            except SystemExit:
                refused = True
            check('flash-part refuses %s (%d B) on the host'
                  % (zone, size), refused and not fake.commands,
                  'refused=%s cmds=%r' % (refused, fake.commands))
    finally:
        if out.exists():
            out.unlink()

    failed = 0
    for name, ok, detail in checks:
        print('%s %s%s' % ('PASS' if ok else 'FAIL', name,
                           '' if ok else '  <%s>' % detail))
        failed += 0 if ok else 1
    print('%d/%d checks passed' % (len(checks) - failed, len(checks)))
    return 0 if not failed else 1


class _Sink:
    def __init__(self):
        self.data = bytearray()

    def write(self, data):
        self.data += data
        return len(data)


class _FakeDev:
    """Just enough of Device for stream()/stream_slices() over FakePayload."""

    def __init__(self, ep):
        self.ep = ep

    def command(self, cmd):
        return send_command(self.ep, self.ep, cmd, 1000,
                            on_info=lambda s: None)

    def upload(self, fh):
        return receive_upload(self.ep, self.ep, fh, 1000,
                              on_info=lambda s: None)

    def piece(self, spec, fh):
        return receive_upload(self.ep, self.ep, fh, 1000,
                              on_info=lambda s: None, cmd=spec)

    def download(self, data):
        return send_download(self.ep, self.ep, data, 1000,
                             on_info=lambda s: None)


class _FlakyDev:
    """Fails its first `fails` pieces, then behaves like _FakeDev.

    Mirrors the camera after a broken transfer: the payload itself is unharmed,
    so the exporter's re-arm on the same offset lands on a healthy device.
    """

    def __init__(self, fails):
        self.fails = fails
        self.attempts = 0
        self.fake = FakePayload()

    def command(self, cmd):
        return send_command(self.fake, self.fake, cmd, 1000,
                            on_info=lambda s: None)

    def piece(self, spec, fh):
        self.attempts += 1
        if self.attempts <= self.fails:
            return 'SHORT', '0 bytes (fake detach)', 0, 0
        return receive_upload(self.fake, self.fake, fh, 1000,
                              on_info=lambda s: None, cmd=spec)


class _Stall:
    """An EP_OUT that answers EPIPE once, then works -- selftest for write_out."""

    def __init__(self, payload):
        self.payload = payload
        self.device = self              # clear_halt() looks for this
        self.bEndpointAddress = 0x01
        self.halts = 0
        self.writes = 0

    def clear_halt(self, ep):
        self.halts += 1

    def write(self, data, timeout=None):
        self.writes += 1
        if self.writes == 1:
            raise usb.core.USBError('Pipe error', errno=32)
        return self.payload.write(data, timeout)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='verb', required=True)
    sub.add_parser('devices', help='list 18d1:4ee0 without opening it')
    sub.add_parser('info', help='version / product / serialno / max-download-size')
    p_getvar = sub.add_parser('getvar', help='one fastboot variable')
    p_getvar.add_argument('name')
    p_peek = sub.add_parser('peek', help='read-only register/memory read')
    p_peek.add_argument('ranges', nargs='+', metavar='addr[:len]')
    sub.add_parser('storage', help='read-only eMMC identification (oem storage)')
    sub.add_parser('partition', help='read-only SDM2 table dump (oem partition)')
    p_dump = sub.add_parser('dump', help='arm a partition slice and stream it out')
    p_dump.add_argument('name', help='slot name, e.g. nflasha23')
    p_dump.add_argument('off', type=lambda s: int(s, 0), nargs='?', default=0,
                        help='byte offset, 512-aligned (default 0)')
    p_dump.add_argument('length', type=lambda s: int(s, 0), nargs='?', default=None,
                        help='byte length, 512-aligned (default: rest of slot)')
    p_dump.add_argument('-o', '--output', help='target file under %s' % ROOT)
    p_dump.add_argument('--slice', type=lambda s: int(s, 0), default=UPLOAD_READ,
                        help='bytes per arm+upload, a 512 multiple (default %d = one '
                             'bulk transfer; the payload sends a piece in %d-byte '
                             'chunks and 24 MHz has never survived more than one '
                             'chunk per command, so >%d is known broken)'
                             % (UPLOAD_READ, UPLOAD_READ, UPLOAD_READ))
    p_dump.add_argument('--single', action='store_true',
                        help='one arm + one upload (the pre-round-88 path)')
    p_dump.add_argument('--retries', type=int, default=3,
                        help='re-arm the same offset this many times after a '
                             'piece dies (default 3; 0 = stop at the first one)')
    p_dump.add_argument('--retry-wait', type=float, default=3.0,
                        help='seconds to wait for the camera to re-enumerate '
                             'before each retry (default 3.0)')
    p_dump.add_argument('--resume', action='store_true',
                        help='continue from the length of an existing -o file '
                             'instead of truncating it')
    p_upload = sub.add_parser('upload', help='stream what the payload is armed with')
    p_upload.add_argument('outfile')
    p_flash = sub.add_parser(
        'flash', help='download one file, then `flash:<zone>` (only the -Write '
                      'test image carries a zone; every other image refuses)')
    p_flash.add_argument('zone', help='whitelisted zone name, e.g. nflasha16_s0')
    p_flash.add_argument('-f', '--file', required=True,
                         help='source file under %s, whole sectors, <= 16 KiB'
                              % ROOT)
    p_flashpart = sub.add_parser(
        'flash-part', help='write a whole zone back in 16 KiB pieces '
                           '(`flash:<zone>@<off>`; only the -Write test image '
                           'carries zones)')
    p_flashpart.add_argument('zone', help='whitelisted zone name, e.g. nflasha23')
    p_flashpart.add_argument('-f', '--file', required=True,
                             help='source file under %s, whole sectors, no '
                                  'larger than the zone' % ROOT)
    sub.add_parser('armed', help='where did the last upload stop? (oem armed)')
    sub.add_parser('reboot', help='send `reboot`: OKAY, then the payload drops '
                                  'D+ (no reset) and parks; reports what follows')
    p_darwin = sub.add_parser(
        'darwin', help='one read-only SIO exchange with the power MCU (oem darwin)')
    p_darwin.add_argument('args', nargs='?', default='',
                          metavar='peek:addr[:len]',
                          help='pass through to `oem darwin peek:...`')
    sub.add_parser('selftest', help='framing tests against a fake device')
    parser.add_argument('--timeout', type=int, default=None)
    args = parser.parse_args()

    if args.verb == 'devices':
        return 0 if list_devices() else 1
    if args.verb == 'selftest':
        return selftest()

    # Validate every range before touching the device: a rejected address must
    # never reach the camera's MMIO bus.
    # The payload's hex_parse() reads every numeric field as hex, so a decimal
    # ":16" would arrive as 0x16 and trip the 16-byte cap.
    if args.verb == 'peek':
        commands = ['oem peek:0x%08x:0x%x' % parse_peek(spec) for spec in args.ranges]
    elif args.verb == 'info':
        commands = ['getvar:' + name
                    for name in ('version', 'product', 'serialno', 'max-download-size')]
    elif args.verb == 'storage':
        commands = ['oem storage']
    elif args.verb == 'partition':
        commands = ['oem partition']
    elif args.verb == 'armed':
        commands = ['oem armed']
    elif args.verb == 'reboot':
        commands = ['reboot']
    elif args.verb == 'darwin':
        commands = ['oem darwin' + (' ' + args.args if args.args else '')]
    elif args.verb == 'getvar':
        commands = ['getvar:' + args.name]
    else:
        commands = []

    # Identification is bounded by the payload's own timeouts (CMD1 polls for
    # up to 1 s, each command waits up to 500 ms), so allow more than the
    # 5 s the plain reads need.  `partition` adds 4 CMD17 block reads on top,
    # and each upload chunk is one card read plus one bulk transfer.
    if args.verb in ('upload', 'dump', 'flash', 'flash-part'):
        timeout = args.timeout or 30000
    elif args.verb in ('storage', 'partition'):
        timeout = args.timeout or 15000
    elif args.verb == 'darwin':
        # The payload spends up to 4 s per exchange waiting for the SIO block
        # to drop busy, and a VWDT state read is two exchanges of 8 attempts:
        # 64 s worst case. A 5 s cap would report a live payload as gone.
        timeout = args.timeout or 70000
    else:
        timeout = args.timeout or 5000

    # A sliced dump owns its session: it must be able to drop a dead device and
    # open a fresh one, so it is never handed an already-open Device.
    if args.verb == 'dump' and args.length is not None and not args.single:
        out = args.output or (ROOT / 'out' / ('%s.bin' % args.name))
        return stream_slices(args.name, args.off, args.length, out, args.slice,
                             timeout=timeout, retries=args.retries,
                             retry_wait=args.retry_wait, resume=args.resume)

    dev = Device(timeout)
    try:
        if args.verb == 'flash':
            return flash_zone(dev, args.zone, args.file, timeout)
        if args.verb == 'flash-part':
            return flash_part(dev, args.zone, args.file, timeout)
        if args.verb == 'dump':
            out = args.output or (ROOT / 'out' / ('%s.bin' % args.name))
            if args.length is None:
                print('note: no length given -> one arm, no slicing')
                return stream(dev, parse_slice(args.name, args.off, None), out)
            return stream(dev, parse_slice(args.name, args.off, args.length), out)
        if args.verb == 'upload':
            return stream(dev, None, args.outfile)

        ok = True
        for cmd in commands:
            ok &= run(dev, cmd)
        if args.verb == 'reboot' and ok:
            return observe_reboot()
        return 0 if ok else 1
    finally:
        dev.close()


if __name__ == '__main__':
    sys.exit(main())
