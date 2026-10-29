# Build the A7C payload with the Arduino15 ARM toolchain.
#
#   .\build.ps1            compile, link, objcopy, wrap into out\payload.a3
#   .\build.ps1 -Clean     remove out\ first
#   .\build.ps1 -Selftest  A3-verify-only image -> out\selftest.a3
#   .\build.ps1 -Probe     read-only four-point probe -> out\probe.a3
#   .\build.ps1 -Probe2    probe + clkrst3 opening writes -> out\probe2.a3
#   .\build.ps1 -Probe3    probe + DWC3 FIFO size registers -> out\probe3.a3
#   .\build.ps1 -Enum      enum diagnostic: report every SETUP packet -> out\enum.a3
#   .\build.ps1 -Hold      same reports, but against a lamp held ON -> out\hold.a3
#   .\build.ps1 -Trace     functional image + one-shot progress marks -> out\trace.a3
#   .\build.ps1 -Beat      connect, then NOP-timed heartbeat only -> out\beat.a3
#   .\build.ps1 -BeatT     same heartbeat, but timed by the on-chip timer -> out\beatt.a3
#   .\build.ps1 -Write     functional image + CMD24 write path -> out\payload_w.a3
#
# -Write is the only build that can program the card, and only the zones named
# in src\fastboot.c's whitelist (nflasha16_s0 from M-7, nflasha23 from M-8).
# It is never the default and never payload.a3.
#
# Everything is written under a7c_payload\out.

param(
    [switch]$Clean,
    [switch]$Selftest,
    [switch]$Probe,
    [switch]$Probe2,
    [switch]$Probe3,
    [switch]$Enum,
    [switch]$Hold,
    [switch]$Trace,
    [switch]$Beat,
    [switch]$BeatT,
    [switch]$Write
)

$ErrorActionPreference = 'Stop'

$root = $PSScriptRoot
$out  = Join-Path $root 'out'
$tc   = 'C:\Users\skywa\AppData\Local\Arduino15\packages\rp2040\tools\pqt-gcc\5.0.0-9576866\bin'

$gcc     = Join-Path $tc 'arm-none-eabi-gcc.exe'
$objcopy = Join-Path $tc 'arm-none-eabi-objcopy.exe'
$size    = Join-Path $tc 'arm-none-eabi-size.exe'
$objdump = Join-Path $tc 'arm-none-eabi-objdump.exe'

foreach ($tool in @($gcc, $objcopy, $size)) {
    if (-not (Test-Path $tool)) { throw "missing tool: $tool" }
}

if ($Clean -and (Test-Path $out)) {
    Remove-Item -Recurse -Force $out
}
New-Item -ItemType Directory -Force -Path $out | Out-Null

$modes = @($Selftest, $Probe, $Probe2, $Probe3, $Enum, $Hold, $Trace, $Beat, $BeatT, $Write) | Where-Object { $_ }
if ($modes.Count -gt 1) {
    throw "-Selftest, -Probe, -Probe2, -Probe3, -Enum, -Hold, -Trace, -Beat, -BeatT and -Write are mutually exclusive"
}

# The probes never enter the USB code, so those objects are left out and the
# image stays small enough to read the disassembly by hand.
if ($Probe -or $Probe2 -or $Probe3) {
    $names = @('src\start.S', 'src\timer.c', 'src\led.c', 'src\a3.c', 'src\libc.c', 'src\main.c')
} else {
    $names = @(
        'src\start.S',
        'src\timer.c',
        'src\led.c',
        'src\a3.c',
        'src\libc.c',
        'src\usb_phy.c',
        'src\dwc3.c',
        'src\usb.c',
        'src\fastboot.c',
        'src\storage.c',
        'src\part.c',
        'src\darwin.c',
        'src\main.c'
    )
}
$srcs = $names | ForEach-Object { Join-Path $root $_ }

$flags = @(
    '-march=armv7-a',
    '-mthumb',
    '-mfloat-abi=soft',
    '-O2',
    '-g',
    '-ffreestanding',
    '-fno-builtin',
    '-fno-common',
    '-fno-stack-protector',
    # GCC otherwise turns the BSS/clear loops into memset() calls.
    '-fno-tree-loop-distribute-patterns',
    '-Wall',
    '-Wextra',
    '-Wno-unused-parameter',
    '-Iinclude',
    '-nostdlib',
    '-nostartfiles',
    '-T', 'payload.ld',
    '-Wl,--build-id=none'
)

if ($Selftest) {
    $flags += '-DSELFTEST'
}
if ($Probe) {
    $flags += '-DPROBE'
}
if ($Probe2) {
    $flags += '-DPROBE2'
}
if ($Probe3) {
    $flags += '-DPROBE3'
}
if ($Enum) {
    $flags += '-DENUM'
}
if ($Hold) {
    # -Hold reuses the whole -Enum diagnostic path; only the lamp convention
    # differs (reports are dark gaps against a lamp that stays on).
    $flags += '-DENUM', '-DHOLD'
}
if ($Trace) {
    # Functional image with a handful of one-shot progress marks: the image
    # still has to enumerate, so the marks are short and fire once each.
    $flags += '-DTRACE'
}
if ($Beat) {
    # Control build: connect, then a NOP-timed heartbeat with no USB access.
    $flags += '-DBEAT'
}
if ($BeatT) {
    # Same heartbeat, but clocked by timer_ticks()/mdelay instead of NOPs.
    $flags += '-DBEAT', '-DBEAT_TIMER'
}
if ($Write) {
    # M-7: the card-side CMD24 path plus the fastboot-side policy gate. Both
    # macros must be on together; fastboot.c #errors if only one is set.
    $flags += '-DSTORAGE_ALLOW_WRITE=1', '-DFB_ALLOW_WRITE=1'
}
$name = if ($Write) { 'payload_w' } elseif ($BeatT) { 'beatt' } elseif ($Beat) { 'beat' } elseif ($Trace) { 'trace' } elseif ($Hold) { 'hold' } elseif ($Probe3) { 'probe3' } elseif ($Probe2) { 'probe2' } elseif ($Probe) { 'probe' } elseif ($Enum) { 'enum' } elseif ($Selftest) { 'selftest' } else { 'payload' }
$flags += "-Wl,-Map=$out\$name.map"

$elf = Join-Path $out "$name.elf"
$bin = Join-Path $out "$name.bin"
$ap3 = Join-Path $out "$name.a3"

Push-Location $root
try {
    Write-Host "[build] compiling $($srcs.Count) sources"
    & $gcc @flags @srcs -o $elf
    if ($LASTEXITCODE -ne 0) { throw "compile/link failed ($LASTEXITCODE)" }

    & $objcopy -O binary $elf $bin
    if ($LASTEXITCODE -ne 0) { throw "objcopy failed ($LASTEXITCODE)" }

    & $size $elf

    $binLen = (Get-Item $bin).Length
    Write-Host ("[build] flat image {0} B, headroom {1} B" -f $binLen, (65536 - $binLen))
    if ($binLen -gt 65536) { throw "image overflows the 64K ESRAM window" }

    Write-Host "[build] wrapping A3 container"
    python (Join-Path $root 'tools\a3image.py') $bin -o $ap3 --verify
    if ($LASTEXITCODE -ne 0) { throw "a3image failed ($LASTEXITCODE)" }

    & $objdump -d $elf | Out-File -Encoding ascii (Join-Path $out "$name.dis")
} finally {
    Pop-Location
}

Write-Host "[build] done: $ap3"
