# SPDX-License-Identifier: GPL-2.0-or-later
# Local OEMSK experiment. Every guest disk write goes to a temporary snapshot.
param(
    [string]$Qemu,
    [string]$Firmware,
    [string]$Variables,
    [string]$OsDisk,
    [string]$BoardSsdt,
    [string]$RunDirectory,
    [ValidateRange(1, 8)][int]$Cpus = 4,
    [ValidateRange(1, 65535)][int]$QmpPort = 5855,
    [ValidateRange(1, 65535)][int]$GdbPort = 5856,
    [switch]$Headless,
    [switch]$WaitForDebugger
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$root = Split-Path -Parent $repo
$lab = Join-Path $root 'winrt-run\oemsk-lun0-20261112'
if (!$Qemu) { $Qemu = Join-Path $repo 'build-oemsk-arm64\qemu-system-aarch64.exe' }
if (!$Firmware) { $Firmware = Join-Path $lab 'edk2-aarch64-1344x1892.fd' }
if (!$Variables) { $Variables = Join-Path $lab 'edk2-aarch64-vars.fd' }
if (!$OsDisk) { $OsDisk = Join-Path $lab 'boot-usb-gic2.before-2880x2560.qcow2' }
if (!$BoardSsdt) {
    $BoardSsdt = Join-Path $root 'winrt-run\oemsk-lun4-20261113\qcdx-oemsk-native-engines.aml'
}
if (!$RunDirectory) {
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
    $RunDirectory = Join-Path $root "winrt-run\oemsk-model-$stamp"
}
$Qemu = (Resolve-Path -LiteralPath $Qemu).Path
$Firmware = (Resolve-Path -LiteralPath $Firmware).Path
$Variables = (Resolve-Path -LiteralPath $Variables).Path
$OsDisk = (Resolve-Path -LiteralPath $OsDisk).Path
$BoardSsdt = (Resolve-Path -LiteralPath $BoardSsdt).Path
if ($QmpPort -eq $GdbPort) { throw 'QMP and GDB need distinct listener ports' }
if (Get-NetTCPConnection -LocalPort @($QmpPort, $GdbPort) -State Listen -ErrorAction SilentlyContinue) {
    throw 'An OEMSK test port is already in use'
}
New-Item -ItemType Directory -Force -Path $RunDirectory | Out-Null
$vars = Join-Path $RunDirectory 'vars-16m.fd'
if (Test-Path -LiteralPath $vars) { throw 'Choose a new run directory: vars-16m.fd already exists' }
$reader = [IO.BinaryReader]::new([IO.File]::OpenRead($Variables))
try {
    $bytes = $reader.ReadBytes(16MB)
    if ($bytes.Length -ne 16MB) { throw 'The variable-flash template must contain at least 16 MiB' }
    [IO.File]::WriteAllBytes($vars, $bytes)
} finally {
    $reader.Dispose()
}
$env:PATH = 'C:\DOWN\msys\ucrt64\bin;C:\DOWN\msys\usr\bin;' + $env:PATH
$display = if ($Headless) { 'none' } else { 'gtk,zoom-to-fit=on,show-cursor=on' }
$vmArgs = @(
    '-name', 'OEMSK independent board',
    '-machine', ('oemsk,board-ssdt=' + $BoardSsdt.Replace(',', ',,')),
    '-cpu', 'max,pauth-impdef=on', '-accel', 'tcg,thread=multi',
    '-smp', $Cpus, '-m', '4G', '-nodefaults', '-snapshot',
    '-drive', ('if=pflash,unit=0,format=raw,readonly=on,file=' + $Firmware.Replace(',', ',,')),
    '-drive', ('if=pflash,unit=1,format=raw,file=' + $vars.Replace(',', ',,')),
    '-drive', ('if=none,id=os,format=qcow2,file=' + $OsDisk.Replace(',', ',,')),
    '-device', 'ramfb,id=bootfb', '-device', 'qemu-xhci,id=usb',
    '-device', 'usb-kbd,bus=usb.0,port=1', '-device', 'usb-tablet,bus=usb.0,port=2',
    '-device', 'usb-storage,bus=usb.0,port=3,drive=os,logical_block_size=4096,physical_block_size=4096,bootindex=1',
    '-rtc', 'base=2017-07-01T12:00:00,clock=vm', '-display', $display, '-nic', 'none',
    '-serial', ('file:' + (Join-Path $RunDirectory 'serial.log')),
    '-qmp', "tcp:127.0.0.1:${QmpPort},server=on,wait=off",
    '-gdb', "tcp:127.0.0.1:${GdbPort}",
    '-action', 'reboot=shutdown,shutdown=pause'
)
if ($WaitForDebugger) { $vmArgs += '-S' }
Write-Host "OEMSK snapshot run: $RunDirectory; QMP $QmpPort; GDB $GdbPort"
& $Qemu @vmArgs
exit $LASTEXITCODE
