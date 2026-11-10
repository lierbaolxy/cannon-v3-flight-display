# 编译脚本：Arm GNU Toolchain (arm-none-eabi-gcc) 裸机编译
# 用法： .\build.ps1            # 默认编译 flight.c
#        .\build.ps1 sensor_lcd # 编译其他源文件
param(
    [string]$Target = 'flight',
    [string]$Toolchain = 'C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin'
)

$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

$gcc     = Join-Path $Toolchain 'arm-none-eabi-gcc.exe'
$objcopy = Join-Path $Toolchain 'arm-none-eabi-objcopy.exe'
$size    = Join-Path $Toolchain 'arm-none-eabi-size.exe'

if (-not (Test-Path $gcc)) {
    throw "未找到 $gcc，请修改 -Toolchain 参数指向你的 arm-none-eabi 工具链 bin 目录"
}

# -mfpu=fpv4-sp-d16 -mfloat-abi=hard：启用 FPU 硬件浮点
# -nostdlib -nostartfiles：不用标准库，入口由 startup.c 提供
# -fno-builtin：避免编译器把自实现的 fsin/fcos/fsqrt 替换成库调用
& $gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard `
    -Os -fno-builtin -Wall -Wextra `
    -ffunction-sections -fdata-sections -nostdlib -nostartfiles `
    -T stm32f401re.ld startup.c "$Target.c" `
    -o "$Target.elf" '-Wl,--gc-sections'

if ($LASTEXITCODE -ne 0) { throw "编译失败" }

& $objcopy -O binary "$Target.elf" "$Target.bin"
& $size "$Target.elf"
Get-Item "$Target.bin" | Select-Object Name, Length
