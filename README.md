# cannon-v3-flight-display

基于 STM32F401RET6（juma.io Cannon V3 / ML14.04）的**裸机姿态仪表盘**。

LSM6DS3 六轴数据实时驱动 1.44" ST7735 屏上的 3D 线框飞机，同屏显示海拔与合加速度；
板子静止 20 秒自动切换成 RTC 时钟表盘。

- 无 HAL、无 CMSIS、无标准库，全部寄存器级驱动
- 84MHz 主频，32KB 帧缓冲整帧推送，约 35fps
- 3D 线框渲染：旋转矩阵 + 透视投影 + Wu 抗锯齿
- 串口对时：`T + HHMMSS`

---

## 1. 硬件

| 项目 | 型号 / 说明 |
|---|---|
| MCU | STM32F401RET6，Cortex-M4F，84MHz，512KB Flash，96KB RAM |
| 开发板 | juma.io Cannon V3（ML14.04） |
| 屏幕 | ST7735S 1.44"，128×128，RGB565 |
| IMU | LSM6DS3（0x6B）加速度 + 陀螺 |
| 其他传感器 | LSM303AGR 加速度 0x19 / 磁力 0x1E、LPS25H 气压 0x5D、HTS221 温湿度 0x5F |
| USB-TTL | 3.3V 电平，115200 8N1 |

板级资料：[juma_cannon_v3_schematic.html](juma_cannon_v3_schematic.html)（功能级原理图，实测重建）、
[juma_cannon_v3_pinout.csv](juma_cannon_v3_pinout.csv)（引脚网络表，含电平与实测状态）。

## 2. 接线

### 屏幕（SPI2）

| ST7735 模块 | 接到 | 说明 |
|---|---|---|
| VCC | 3.3V | |
| GND | GND | |
| BL | 3.3V | 背光，必须接，否则全黑 |
| RES | 3.3V | 复位，必须接 |
| CS | PB12 | 片选 |
| DC | PB14 | 数据/命令 |
| SDA（丝印） | PB15 | 实际是 SPI MOSI |
| SCL（丝印） | PB13 | 实际是 SPI SCK |

### 串口（USART2）

| USB-TTL | 接到 |
|---|---|
| TX | PA3（MCU RX） |
| RX | PA2（MCU TX） |
| GND | GND |

> 只做单向接收日志时，可以不接 USB-TTL 的 TX；但对时需要 PC → MCU 方向，必须接 PA3。

### 传感器（板载，I2C1）

PB6 = SCL，PB7 = SDA，400kHz，无需外接。

### 烧录

用 STM32 内置 USB DFU，**不需要 DAPLink / ST-Link**：

1. BOOT0 接 3.3V
2. 拔插 USB（此时板子枚举为 `STM32 BOOTLOADER`）
3. `.\flash.ps1`
4. BOOT0 接回 GND，按 RESET

> 若要改用 SWD 调试，DAPLink 接线见 [firmware/WIRING.md](firmware/WIRING.md)。
> **禁止**把 DAPLink 的 5V/3.3V 与板子的 3.3V 对接，会倒灌烧毁稳压器。

## 3. 工具链

| 工具 | 版本 | 用途 |
|---|---|---|
| [Arm GNU Toolchain (arm-none-eabi)](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) | 14.2.rel1 | 编译 |
| [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) | 2.23.0 | USB DFU 烧录 |
| PowerShell | 5.1+ | 执行脚本 |

## 4. 编译与烧录

```powershell
cd firmware
.\build.ps1              # 生成 flight.elf / flight.bin
.\flash.ps1              # 需先 BOOT0=3.3V 并拔插 USB
```

编译参数（`build.ps1` 内）：

```
-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
-Os -fno-builtin -Wall -Wextra
-ffunction-sections -fdata-sections -nostdlib -nostartfiles
-T stm32f401re.ld startup.c flight.c -Wl,--gc-sections
```

> `-mfpu` / `-mfloat-abi` 只是让编译器生成硬件浮点指令，
> **还必须在 `startup.c` 的 `Reset_Handler` 里写 `CPACR |= (0xF << 20)` 打开 FPU**，
> 否则第一条浮点指令就 HardFault（表现为白屏 + 串口全无）。

## 5. 运行效果

| 状态 | 显示 |
|---|---|
| 有动作 | 3D 线框飞机随姿态转动 + 左上 `ALT` 海拔 + 左下 `ACC` 合加速度 |
| 静止 20 秒 | 模拟时钟表盘（时/分/秒针） |

渲染细节：

- 相机固定俯角 30°、侧偏 22° 的 3/4 追尾视角（正后方视角机翼会投影成一条线）
- 姿态角放大 2.8 倍显示，限幅 ±83°
- 屏幕中间有固定地平线参考线
- 翼尖航行灯：**左红右绿**，用于判断左右倾斜

## 6. 串口协议

上电后每 20 帧打印一次：

```
t=1180 pitch=6 roll=0 yaw=-7 clock=0
```

### 对时

发 `T` + `HHMMSS` + 回车，例 `T142530\r\n`，成功回 `TIME OK`。

> **必须逐字节发送，间隔 ≥50ms**。USART2 只有 1 字节接收寄存器，
> 一次整串灌入会溢出丢字节。

## 7. 目录结构

```
.
├── README.md
├── CAUTIONS.md             # 注意事项：电气安全 / 烧录 / 串口 / 改代码的坑
├── DEBUG_NOTES.md          # 从 0 到 1 的调试记录（问题 / 原因 / 解决）
├── .gitignore
├── aircraft_lcd_preview.html  # 128×128 像素级效果预览（浏览器打开）
├── juma_cannon_v3_schematic.html  # 板级功能原理图（实测重建，浏览器打开）
├── juma_cannon_v3_pinout.csv      # 引脚网络表（电平 / 实测状态 / 备注）
├── docs
│   └── resume_template.md         # 嵌入式简历模板（含本项目示例条目）
└── firmware
    ├── flight.c            # 主程序：姿态 / 3D 渲染 / 时钟 / 串口
    ├── sensor_lcd.c        # 五路传感器 + LCD 显示（LCD 初始化序列来源）
    ├── startup.c           # 向量表 + FPU 使能 + .data/.bss 初始化
    ├── stm32f401re.ld      # 链接脚本
    ├── build.ps1           # 编译脚本
    ├── flash.ps1           # DFU 烧录脚本
    └── WIRING.md           # SWD / DAPLink 接线备忘
```

## 8. 已知限制

- RTC 无后备电池，**完全断电后时间丢失**，回到 12:00:00，需要重新对时
- 若 RTC 时钟源为内部 LSI（±5%），长时间走时有分钟级误差
- 偏航靠陀螺 Z 轴积分（无磁力计融合），静止时靠回中机制抑制漂移
- 板载 BlueNRG 蓝牙协议层无法打通，未使用
- 未实现 SD 卡 / 视频播放（需额外硬件确认）

调试过程与踩坑记录见 [DEBUG_NOTES.md](DEBUG_NOTES.md)，
动手改硬件或改代码前请先看 [CAUTIONS.md](CAUTIONS.md)。
