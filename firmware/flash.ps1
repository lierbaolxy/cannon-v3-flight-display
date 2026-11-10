# 烧录脚本：STM32 内置 USB DFU + STM32CubeProgrammer CLI
# 用法： .\flash.ps1                 # 烧录 flight.bin
#        .\flash.ps1 sensor_lcd.bin  # 烧录其他文件
#
# 烧录前必须：BOOT0 接 3.3V，拔插 USB 让板子进入 DFU
# 烧录后必须：BOOT0 接回 GND，按 RESET 运行新固件
param(
    [string]$Bin  = 'flight.bin',
    [string]$Addr = '0x08000000',
    [string]$Cli  = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
)

$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

if (-not (Test-Path $Cli))  { throw "未找到 STM32_Programmer_CLI：$Cli" }
if (-not (Test-Path $Bin))  { throw "未找到固件：$Bin" }

# STM32 DFU 的 VID 固定为 0483
$dfu = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
       Where-Object { $_.InstanceId -match 'VID_0483' }

if (-not $dfu) {
    Write-Host 'NO_DFU：请确认 BOOT0 已接 3.3V，并拔插一次 USB' -ForegroundColor Red
    exit 1
}

& $Cli -c port=USB1 -e all -w $Bin $Addr -v
if ($LASTEXITCODE -ne 0) { throw "烧录失败：若出现 DevID=0x0000 / mass erase 失败，拔插 USB 后重试" }
Write-Host '烧录完成：把 BOOT0 接回 GND，按 RESET 运行' -ForegroundColor Green
