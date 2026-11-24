param(
    [string]$OsDisk = 'C:\DOWN\900170480965_10.0.17047.1034_FactoryOS_3BW3.2.PRIBG.lun0.vhdx',
    [string]$Qemu = 'C:\DOWN\tempgitrepo\qemu\build-oemsk-arm64\qemu-system-aarch64.exe',
    [string]$PcBios = 'C:\DOWN\tempgitrepo\qemu\build-oemsk-arm64\pc-bios',
    [string]$Firmware = 'C:\DOWN\tempgitrepo\edk2-winrt\Build\ArmVirtFakeOemsk850-AARCH64\DEBUG_GCC5\FV\FAKEOEMSK850_CODE.fd',
    [string]$BoardSsdt = '',
    [string]$CmdDb = '',
    [string]$VarsTemplate = 'C:\DOWN\tempgitrepo\edk2-winrt\Build\ArmVirtFakeOemsk850-AARCH64\DEBUG_GCC5\FV\FAKEOEMSK850_VARS.fd',
    [string]$RpmbTemplate = 'C:\DOWN\tempgitrepo\winrt-run\oemsk-lun0-20261112\qcom8998-ccu-clear\rpmb-emmc.qcow2',
    [string]$RunDirectory,
    [ValidateRange(1, 8)][int]$Cpus = 4,
    [string]$Memory = '4G',
    [ValidateRange(1, 65535)][int]$QmpPort = 5970,
    [ValidateRange(1, 65535)][int]$GdbPort = 5971,
    [ValidateRange(1, 65535)][int]$TelnetPort = 5925,
    [switch]$Headless,
    [switch]$WaitForDebugger,
    [switch]$TraceDisplay,
    [ValidateSet('vhdx', 'qcow2')][string]$OsFormat = 'vhdx'
)

$ErrorActionPreference = 'Stop'

$OsDisk = (Resolve-Path -LiteralPath $OsDisk).Path
$Qemu = (Resolve-Path -LiteralPath $Qemu).Path
$PcBios = (Resolve-Path -LiteralPath $PcBios).Path
$Firmware = (Resolve-Path -LiteralPath $Firmware).Path
if ($BoardSsdt) { $BoardSsdt = (Resolve-Path -LiteralPath $BoardSsdt).Path }
if ($CmdDb) { $CmdDb = (Resolve-Path -LiteralPath $CmdDb).Path }

$varsTemplate = (Resolve-Path -LiteralPath $VarsTemplate).Path
$rpmbTemplate = (Resolve-Path -LiteralPath $RpmbTemplate).Path

$ports = @($QmpPort, $GdbPort, $TelnetPort)
if (($ports | Select-Object -Unique).Count -ne $ports.Count) {
    throw 'QmpPort, GdbPort, and TelnetPort must be distinct.'
}
$inUse = Get-NetTCPConnection -LocalPort $ports -State Listen -ErrorAction SilentlyContinue
if ($inUse) {
    $busy = ($inUse.LocalPort | Sort-Object -Unique) -join ', '
    throw "Port(s) already in use: $busy"
}

if (-not $RunDirectory) {
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $RunDirectory = Join-Path (Join-Path $PSScriptRoot '..\..\winrt-run\oemsk850-17047') "run-$stamp"
}
New-Item -ItemType Directory -Force -Path $RunDirectory | Out-Null

$varsPath = Join-Path $RunDirectory 'vars-16m.fd'
$rpmbPath = Join-Path $RunDirectory 'rpmb-emmc.qcow2'
$serialLog = Join-Path $RunDirectory 'serial.log'
$deviceLog = Join-Path $RunDirectory 'device.log'

Copy-Item -LiteralPath $varsTemplate -Destination $varsPath -Force
Copy-Item -LiteralPath $rpmbTemplate -Destination $rpmbPath -Force

function Escape-QemuOpt([string]$Value) {
    return $Value.Replace(',', ',,')
}

$osDrive = "if=none,id=os,format=$OsFormat,snapshot=on,file=$(Escape-QemuOpt $OsDisk)"

$display = if ($Headless) {
    'none'
} else {
    'gtk,zoom-to-fit=on,show-cursor=on,show-tabs=on'
}

$env:PATH = 'C:\DOWN\msys\ucrt64\bin;C:\DOWN\msys\usr\bin;' + $env:PATH

$machine = 'oemsk850'
if ($BoardSsdt) { $machine += ",board-ssdt=$(Escape-QemuOpt $BoardSsdt)" }
if ($CmdDb) { $machine += ",cmd-db=$(Escape-QemuOpt $CmdDb)" }

$vmArgs = @(
    '-L', $PcBios,
    '-name', 'OEMSK850 SDM845 17047',
    '-M', $machine,
    '-cpu', 'max,pauth-impdef=on',
    '-accel', 'tcg,thread=multi,tb-size=512',
    '-smp', $Cpus,
    '-m', $Memory,
    '-nodefaults',
    '-drive', ("if=pflash,unit=0,format=raw,readonly=on,file=$(Escape-QemuOpt $Firmware)"),
    '-drive', ("if=pflash,unit=1,format=raw,file=$(Escape-QemuOpt $varsPath)"),
    '-drive', $osDrive,
    '-device', 'ramfb,id=bootfb',
    '-device', 'qemu-xhci,id=usb',
    '-device', 'usb-kbd,bus=usb.0,port=1',
    '-device', 'usb-tablet,bus=usb.0,port=2',
    '-device', 'usb-storage,id=osdisk,bus=usb.0,port=3,drive=os,logical_block_size=4096,physical_block_size=4096,bootindex=1',
    '-drive', ("if=none,id=rpmb,format=qcow2,file=$(Escape-QemuOpt $rpmbPath)"),
    '-device', 'sdhci-pci,id=rpmb-host',
    '-device', 'emmc,drive=rpmb,rpmb-partition-size=131072',
    '-netdev', "user,id=telnetnet,restrict=on,hostfwd=tcp:127.0.0.1:${TelnetPort}-:23",
    '-device', 'usb-ax88772,id=telnetnic,bus=usb.0,port=4,netdev=telnetnet',
    '-rtc', 'base=2017-07-01T12:00:00,clock=vm',
    '-display', $display,
    '-serial', "file:$serialLog",
    '-D', $deviceLog,
    '-d', 'invalid_mem',
    '-trace', 'enable=qcom845_scm_call',
    '-trace', 'enable=qcom845_qsee_message',
    '-trace', 'enable=qcom845_spmi_access',
    '-qmp', "tcp:127.0.0.1:${QmpPort},server=on,wait=off",
    '-gdb', "tcp:127.0.0.1:${GdbPort}",
    '-action', 'reboot=shutdown,shutdown=pause'
)

if ($TraceDisplay) {
    $vmArgs += @('-trace', 'enable=qcom845_display_*',
                 '-trace', 'enable=qcom845_cp_*',
                 '-trace', 'enable=qcom845_scanout',
                 '-trace', 'enable=qcom845_smmu_fault')
}

if ($WaitForDebugger) {
    $vmArgs += '-S'
}

$mode = 'snapshot (read-only base)'
Write-Host "OS Disk : $OsDisk ($mode)"
Write-Host "Run Dir : $RunDirectory"
Write-Host "Ports   : QMP=$QmpPort, GDB=$GdbPort, Telnet=127.0.0.1:$TelnetPort -> guest:23"

& $Qemu @vmArgs
exit $LASTEXITCODE
