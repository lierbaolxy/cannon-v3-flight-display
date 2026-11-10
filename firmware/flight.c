/* Cannon V3 (STM32F401RET6) 3D 姿态飞机 + 实时时钟
 *
 * 硬件：
 *   I2C1  PB6=SCL / PB7=SDA 400kHz
 *     LSM6DS3   0x6B : 六轴（加速度 + 陀螺）
 *     LSM303AGR 0x19 : 加速度   0x1E : 磁力
 *     LPS25H    0x5D : 气压
 *     HTS221    0x5F : 温湿度
 *   SPI2  PB13=SCK / PB15=MOSI / PB12=CS / PB14=DC，10.5MHz
 *   ST7735S 128x128
 *
 * 时钟：HSI 16MHz -> PLL 84MHz
 * 显示：32KB 帧缓冲，整帧推送，约 30fps
 *   运动   -> 3D 线框飞机（抗锯齿）+ 海拔/合加速度
 *   静止5s -> 实时时钟表盘
 */

#include <stdint.h>

#define REG(a)          (*(volatile uint32_t *)(a))

/* ---------------- RCC / PWR / FLASH ---------------- */
#define RCC_CR          REG(0x40023800UL)
#define RCC_PLLCFGR     REG(0x40023804UL)
#define RCC_CFGR        REG(0x40023808UL)
#define RCC_AHB1ENR     REG(0x40023830UL)
#define RCC_APB1ENR     REG(0x40023840UL)
#define RCC_APB2ENR     REG(0x40023844UL)
#define RCC_BDCR        REG(0x40023870UL)
#define RCC_CSR         REG(0x40023874UL)
#define PWR_CR          REG(0x40007000UL)
#define FLASH_ACR       REG(0x40023C00UL)

/* ---------------- GPIO ---------------- */
#define GPIOA_MODER     REG(0x40020000UL)
#define GPIOA_AFRL      REG(0x40020020UL)
#define GPIOB_MODER     REG(0x40020400UL)
#define GPIOB_OTYPER    REG(0x40020404UL)
#define GPIOB_OSPEEDR   REG(0x40020408UL)
#define GPIOB_PUPDR     REG(0x4002040CUL)
#define GPIOB_BSRR      REG(0x40020418UL)
#define GPIOB_AFRL      REG(0x40020420UL)
#define GPIOB_AFRH      REG(0x40020424UL)
#define GPIOB_IDR       REG(0x40020410UL)

/* ---------------- USART2（调试输出） ---------------- */
#define USART2_SR       REG(0x40004400UL)
#define USART2_DR       REG(0x40004404UL)
#define USART2_BRR      REG(0x40004408UL)
#define USART2_CR1      REG(0x4000440CUL)

/* ---------------- I2C1 ---------------- */
#define I2C1_CR1        REG(0x40005400UL)
#define I2C1_CR2        REG(0x40005404UL)
#define I2C1_DR         REG(0x40005410UL)
#define I2C1_SR1        REG(0x40005414UL)
#define I2C1_SR2        REG(0x40005418UL)
#define I2C1_CCR        REG(0x4000541CUL)
#define I2C1_TRISE      REG(0x40005420UL)

/* ---------------- SPI2 ---------------- */
#define SPI2_CR1        REG(0x40003800UL)
#define SPI2_SR         REG(0x40003808UL)
#define SPI2_DR         REG(0x4000380CUL)

/* ---------------- RTC ---------------- */
#define RTC_TR          REG(0x40002800UL)
#define RTC_DR          REG(0x40002804UL)
#define RTC_ISR         REG(0x4000280CUL)
#define RTC_PRER        REG(0x40002810UL)
#define RTC_WPR         REG(0x40002824UL)
#define RTC_BKP0R       REG(0x40002850UL)

/* ---------------- DWT 周期计数器（软件计时用） ---------------- */
#define DEMCR           REG(0xE000EDFCUL)
#define DWT_CTRL        REG(0xE0001000UL)
#define DWT_CYCCNT      REG(0xE0001004UL)

/* ---------------- LCD 控制脚 ---------------- */
#define LCD_CS_LOW()    (GPIOB_BSRR = (1UL << (12 + 16)))
#define LCD_CS_HIGH()   (GPIOB_BSRR = (1UL << 12))
#define LCD_DC_LOW()    (GPIOB_BSRR = (1UL << (14 + 16)))
#define LCD_DC_HIGH()   (GPIOB_BSRR = (1UL << 14))

#define SCR_W           128
#define SCR_H           128
#define FB_SIZE         (SCR_W * SCR_H)

#define C_BLACK         0x0000
#define C_WHITE         0xFFFF

static void delay(volatile uint32_t n)
{
    while (n--) {
        __asm volatile("nop");
    }
}

/* ================= 时钟：84MHz（失败回退 16MHz） ================= */
static uint32_t sysclk_mhz = 16;                /* 实际主频，供串口分频用 */

static void clock_init(void)
{
    /* 打开 PWR 时钟，VOS = Scale 2（支持 84MHz） */
    RCC_APB1ENR |= (1UL << 28);
    PWR_CR |= (1UL << 14);                      /* VOS = 1: Scale 2，支持 84MHz */

    /* Flash 等待 2WS + 预取 + 指令/数据缓存 */
    FLASH_ACR = (1UL << 10) | (1UL << 9) | (1UL << 8) | 2UL;

    /* PLL: HSI16 / M16 * N336 / P4 = 84MHz, Q7 = 48MHz
       PLLSRC=0 选择 HSI（不能用 HSE，板上 HSE 状态未知） */
    RCC_PLLCFGR = (7UL << 24) | (1UL << 16) | (336UL << 6) | 16UL;
    RCC_CR |= (1UL << 24);

    /* 等 PLL 锁定，带超时：失败则保持 HSI 16MHz，不锁死 */
    {
        uint32_t t = 0;
        while ((RCC_CR & (1UL << 25)) == 0 && t < 50000) {
            t++;
        }
        if (t >= 50000) {
            return;                             /* PLL 没锁，继续用 HSI */
        }
    }

    /* AHB /1 = 84MHz, APB1 /2 = 42MHz, APB2 /1 = 84MHz, SW = PLL */
    RCC_CFGR = (4UL << 10) | 2UL;
    {
        uint32_t t = 0;
        while (((RCC_CFGR >> 2) & 3UL) != 2UL && t < 50000) {
            t++;
        }
        if (((RCC_CFGR >> 2) & 3UL) == 2UL) {
            sysclk_mhz = 84;
        }
    }
}

/* 精确毫秒延时，基于 DWT 周期计数器，与主频无关 */
static void delay_ms(uint32_t ms)
{
    uint32_t start = DWT_CYCCNT;
    uint32_t ticks = ms * (sysclk_mhz * 1000UL);
    uint32_t spin  = ms * (sysclk_mhz * 400UL);   /* DWT 异常时的兜底次数 */

    while ((uint32_t)(DWT_CYCCNT - start) < ticks) {
        if (--spin == 0) {
            break;                                 /* DWT 没在计数，不再死等 */
        }
    }
}

/* ================= USART2 ================= */
static void uart2_init(void)
{
    RCC_AHB1ENR |= (1UL << 0);
    GPIOA_MODER &= ~((3UL << 4) | (3UL << 6));
    GPIOA_MODER |=  ((2UL << 4) | (2UL << 6));
    GPIOA_AFRL  &= ~((0xFUL << 8) | (0xFUL << 12));
    GPIOA_AFRL  |=  ((7UL << 8) | (7UL << 12));
    RCC_APB1ENR |= (1UL << 17);
    if (sysclk_mhz == 84) {
        USART2_BRR = 365;                   /* APB1 42MHz / 115200 */
    } else {
        USART2_BRR = 139;                   /* APB1 16MHz / 115200 */
    }
    USART2_CR1 = (1UL << 13) | (1UL << 3) | (1UL << 2);
}

static void uart2_putc(char c)
{
    while ((USART2_SR & (1UL << 7)) == 0) {
    }
    USART2_DR = (uint32_t)c;
}

static void uart2_puts(const char *s)
{
    while (*s) {
        uart2_putc(*s++);
    }
}

static void uart2_int(int32_t v)
{
    char buf[12];
    int i = 0;

    if (v < 0) {
        uart2_putc('-');
        v = -v;
    }
    if (v == 0) {
        uart2_putc('0');
        return;
    }
    while (v > 0 && i < 11) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i > 0) {
        uart2_putc(buf[--i]);
    }
}

/* ================= I2C1 400kHz ================= */
static void i2c1_init(void)
{
    RCC_APB1ENR |= (1UL << 21);

    GPIOB_MODER   &= ~((3UL << 12) | (3UL << 14));
    GPIOB_MODER   |=  ((2UL << 12) | (2UL << 14));
    GPIOB_OTYPER  |=  (1UL << 6) | (1UL << 7);
    GPIOB_OSPEEDR |=  (3UL << 12) | (3UL << 14);
    GPIOB_PUPDR   &= ~((3UL << 12) | (3UL << 14));
    GPIOB_PUPDR   |=  ((1UL << 12) | (1UL << 14));
    GPIOB_AFRL    &= ~((0xFUL << 24) | (0xFUL << 28));
    GPIOB_AFRL    |=  ((4UL << 24) | (4UL << 28));

    I2C1_CR1 &= ~(1UL << 0);
    I2C1_CR1 |=  (1UL << 15);
    I2C1_CR1 &= ~(1UL << 15);
    I2C1_CR2   = 42;                        /* APB1 42MHz */
    I2C1_CCR   = (1UL << 15) | 35UL;        /* 快速模式 400kHz */
    I2C1_TRISE = 14;
    I2C1_CR1  |= (1UL << 0) | (1UL << 10);
}

static int i2c1_wait_idle(void)
{
    volatile uint32_t t = 0;
    while ((I2C1_SR2 & (1UL << 1)) && t < 20000) {
        t++;
    }
    return (t < 20000);
}

static void i2c1_stop(void)
{
    I2C1_CR1 |= (1UL << 9);
    delay(200);
}

static int i2c1_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    volatile uint32_t t;

    if (!i2c1_wait_idle()) {
        return 0;
    }
    I2C1_CR1 |= (1UL << 8);
    t = 0;
    while ((I2C1_SR1 & (1UL << 0)) == 0 && t < 20000) {
        t++;
    }
    if (t >= 20000) {
        i2c1_stop();
        return 0;
    }
    I2C1_DR = (uint32_t)(addr << 1);
    t = 0;
    while (t < 20000) {
        uint32_t sr1 = I2C1_SR1;
        if (sr1 & (1UL << 1)) {
            break;
        }
        if (sr1 & (1UL << 10)) {
            I2C1_SR1 &= ~(1UL << 10);
            i2c1_stop();
            return 0;
        }
        t++;
    }
    if (t >= 20000) {
        i2c1_stop();
        return 0;
    }
    (void)I2C1_SR1;
    (void)I2C1_SR2;

    t = 0;
    while ((I2C1_SR1 & (1UL << 7)) == 0 && t < 20000) {
        t++;
    }
    if (t >= 20000) {
        i2c1_stop();
        return 0;
    }
    I2C1_DR = reg;

    t = 0;
    while ((I2C1_SR1 & (1UL << 7)) == 0 && t < 20000) {
        t++;
    }
    if (t >= 20000) {
        i2c1_stop();
        return 0;
    }
    I2C1_DR = val;

    t = 0;
    while ((I2C1_SR1 & (1UL << 2)) == 0 && t < 20000) {
        t++;
    }
    i2c1_stop();
    return (t < 20000);
}

static int i2c1_read_reg(uint8_t addr, uint8_t reg, uint8_t *val)
{
    volatile uint32_t t;

    if (!i2c1_wait_idle()) {
        return 0;
    }
    I2C1_CR1 |= (1UL << 8);
    t = 0;
    while ((I2C1_SR1 & (1UL << 0)) == 0 && t < 20000) {
        t++;
    }
    if (t >= 20000) {
        i2c1_stop();
        return 0;
    }
    I2C1_DR = (uint32_t)(addr << 1);
    t = 0;
    while (t < 20000) {
        uint32_t sr1 = I2C1_SR1;
        if (sr1 & (1UL << 1)) {
            break;
        }
        if (sr1 & (1UL << 10)) {
            I2C1_SR1 &= ~(1UL << 10);
            i2c1_stop();
            return 0;
        }
        t++;
    }
    if (t >= 20000) {
        i2c1_stop();
        return 0;
    }
    (void)I2C1_SR1;
    (void)I2C1_SR2;

    t = 0;
    while ((I2C1_SR1 & (1UL << 7)) == 0 && t < 20000) {
        t++;
    }
    if (t >= 20000) {
        i2c1_stop();
        return 0;
    }
    I2C1_DR = reg;

    t = 0;
    while ((I2C1_SR1 & (1UL << 2)) == 0 && t < 20000) {
        t++;
    }
    if (t >= 20000) {
        i2c1_stop();
        return 0;
    }

    I2C1_CR1 |= (1UL << 8);
    t = 0;
    while ((I2C1_SR1 & (1UL << 0)) == 0 && t < 20000) {
        t++;
    }
    if (t >= 20000) {
        i2c1_stop();
        return 0;
    }
    I2C1_DR = (uint32_t)((addr << 1) | 1);
    t = 0;
    while (t < 20000) {
        uint32_t sr1 = I2C1_SR1;
        if (sr1 & (1UL << 1)) {
            break;
        }
        if (sr1 & (1UL << 10)) {
            I2C1_SR1 &= ~(1UL << 10);
            i2c1_stop();
            return 0;
        }
        t++;
    }
    if (t >= 20000) {
        i2c1_stop();
        return 0;
    }

    I2C1_CR1 &= ~(1UL << 10);
    (void)I2C1_SR1;
    (void)I2C1_SR2;

    t = 0;
    while ((I2C1_SR1 & (1UL << 6)) == 0 && t < 20000) {
        t++;
    }
    if (t >= 20000) {
        I2C1_CR1 |= (1UL << 10);
        i2c1_stop();
        return 0;
    }

    *val = (uint8_t)I2C1_DR;
    I2C1_CR1 |= (1UL << 10);
    i2c1_stop();
    return 1;
}

static int16_t make_s16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

/* ================= 传感器 ================= */
static int32_t acc_mg[3];
static int32_t gyro_dps10[3];
static int32_t acc2_mg[3];
static int32_t mag_ut10[3];
static int32_t press_hpa10;
static int32_t hts_temp_c10;
static int32_t hts_hum_p10;
static uint8_t sensors_ok;

static void sensors_init(void)
{
    uint8_t id;
    int ok = 0;

    id = 0xFF;
    if (i2c1_read_reg(0x6B, 0x0F, &id) && id == 0x69) {
        i2c1_write_reg(0x6B, 0x10, 0x40);
        i2c1_write_reg(0x6B, 0x11, 0x40);
        i2c1_write_reg(0x6B, 0x12, 0x44);
        ok++;
    }
    id = 0xFF;
    if (i2c1_read_reg(0x19, 0x0F, &id) && id == 0x33) {
        i2c1_write_reg(0x19, 0x20, 0x57);
        i2c1_write_reg(0x19, 0x23, 0x88);
        ok++;
    }
    id = 0xFF;
    if (i2c1_read_reg(0x1E, 0x4F, &id) && id == 0x40) {
        i2c1_write_reg(0x1E, 0x60, 0x40);
        delay_ms(10);                   /* 磁力计 reboot 需 5ms */
        i2c1_write_reg(0x1E, 0x61, 0x00);
        i2c1_write_reg(0x1E, 0x62, 0x10);
        i2c1_write_reg(0x1E, 0x60, 0x00);
        ok++;
    }
    id = 0xFF;
    if (i2c1_read_reg(0x5D, 0x0F, &id) && id == 0xBD) {
        i2c1_write_reg(0x5D, 0x20, 0xC4);
        ok++;
    }
    id = 0xFF;
    if (i2c1_read_reg(0x5F, 0x0F, &id) && id == 0xBC) {
        i2c1_write_reg(0x5F, 0x20, 0x81);
        i2c1_write_reg(0x5F, 0x21, 0x00);
        ok++;
    }
    sensors_ok = (uint8_t)ok;
}

static void read_lsm6ds3(void)
{
    uint8_t lo, hi;
    int i;

    for (i = 0; i < 3; i++) {
        if (i2c1_read_reg(0x6B, (uint8_t)(0x28 + i * 2), &lo) &&
            i2c1_read_reg(0x6B, (uint8_t)(0x29 + i * 2), &hi)) {
            acc_mg[i] = (int32_t)make_s16(lo, hi) * 61 / 1000;
        }
    }
    for (i = 0; i < 3; i++) {
        if (i2c1_read_reg(0x6B, (uint8_t)(0x22 + i * 2), &lo) &&
            i2c1_read_reg(0x6B, (uint8_t)(0x23 + i * 2), &hi)) {
            gyro_dps10[i] = (int32_t)make_s16(lo, hi) * 875 / 10000;
        }
    }
}

static void read_lsm303a(void)
{
    uint8_t lo, hi;
    int i;

    for (i = 0; i < 3; i++) {
        if (i2c1_read_reg(0x19, (uint8_t)(0x28 + i * 2), &lo) &&
            i2c1_read_reg(0x19, (uint8_t)(0x29 + i * 2), &hi)) {
            acc2_mg[i] = (int32_t)make_s16(lo, hi) >> 4;
        }
    }
}

static void read_lsm303m(void)
{
    uint8_t lo, hi;
    int i;

    for (i = 0; i < 3; i++) {
        if (i2c1_read_reg(0x1E, (uint8_t)(0x28 + i * 2), &lo) &&
            i2c1_read_reg(0x1E, (uint8_t)(0x29 + i * 2), &hi)) {
            mag_ut10[i] = (int32_t)make_s16(lo, hi) * 15 / 10;
        }
    }
}

static void read_lps25h(void)
{
    uint8_t xl, xh, yl, yh, zl, zh;
    int32_t p, t;

    if (i2c1_read_reg(0x5D, 0x28, &xl) && i2c1_read_reg(0x5D, 0x29, &xh) &&
        i2c1_read_reg(0x5D, 0x2A, &yl) && i2c1_read_reg(0x5D, 0x2B, &yh) &&
        i2c1_read_reg(0x5D, 0x2C, &zl) && i2c1_read_reg(0x5D, 0x2D, &zh)) {
        p = (int32_t)((uint32_t)xl | ((uint32_t)xh << 8) |
                      ((uint32_t)yl << 16) | ((uint32_t)yh << 24));
        t = (int32_t)make_s16(zl, zh);
        press_hpa10 = (p * 10) / 4096;
        (void)t;
    }
}

/* HTS221 简化：只用温度（原始值 / 8 即摄氏度） */
static void hts_simple(void)
{
    uint8_t t0, t1;
    int32_t t8;

    if (i2c1_read_reg(0x5F, 0x2C, &t0) && i2c1_read_reg(0x5F, 0x2D, &t1)) {
        t8 = ((int32_t)make_s16(t0, t1)) >> 2;
        hts_temp_c10 = t8 * 10 / 8;
        if (hts_temp_c10 < -400) hts_temp_c10 = -400;
        if (hts_temp_c10 > 1200) hts_temp_c10 = 1200;
    }
    hts_hum_p10 = 0;
}

/* ================= SPI2 + ST7735 ================= */
static void spi2_init(void)
{
    RCC_AHB1ENR |= (1UL << 1);
    RCC_APB1ENR |= (1UL << 14);

    /* PB13 SCK, PB15 MOSI 复用 AF5 */
    GPIOB_MODER &= ~((3UL << 26) | (3UL << 30));
    GPIOB_MODER |=  ((2UL << 26) | (2UL << 30));
    GPIOB_OSPEEDR |= (3UL << 26) | (3UL << 30);
    GPIOB_AFRH  &= ~((0xFUL << 20) | (0xFUL << 28));
    GPIOB_AFRH  |=  ((5UL << 20) | (5UL << 28));

    /* PB12 CS, PB14 DC 推挽输出 */
    GPIOB_MODER &= ~((3UL << 24) | (3UL << 28));
    GPIOB_MODER |=  ((1UL << 24) | (1UL << 28));
    GPIOB_OSPEEDR |= (3UL << 24) | (3UL << 28);
    LCD_CS_HIGH();
    LCD_DC_HIGH();

    /* MSTR | SSM | SSI | BR=/4(10.5MHz) | SPE */
    SPI2_CR1 = (1UL << 2) | (1UL << 9) | (1UL << 8) | (1UL << 3);
    SPI2_CR1 |= (1UL << 6);
}

static void spi2_write(uint8_t d)
{
    while ((SPI2_SR & (1UL << 1)) == 0) {
    }
    SPI2_DR = (uint32_t)d;
}

static void spi2_flush(void)
{
    while ((SPI2_SR & (1UL << 1)) == 0) {
    }
    while (SPI2_SR & (1UL << 7)) {
    }
}

static void lcd_cmd(uint8_t c)
{
    LCD_DC_LOW();
    LCD_CS_LOW();
    spi2_write(c);
    spi2_flush();
    LCD_CS_HIGH();
}

static void lcd_dat(uint8_t d)
{
    LCD_DC_HIGH();
    LCD_CS_LOW();
    spi2_write(d);
    spi2_flush();
    LCD_CS_HIGH();
}

static void lcd_init(void)
{
    LCD_CS_HIGH();
    LCD_DC_HIGH();
    delay_ms(150);

    lcd_cmd(0x01);                  /* SWRESET */
    delay_ms(150);
    lcd_cmd(0x11);                  /* SLPOUT */
    delay_ms(150);

    lcd_cmd(0x3A); lcd_dat(0x05);   /* RGB565 */
    lcd_cmd(0x36); lcd_dat(0x00);   /* MADCTL：实测唯一正确方向 */

    lcd_cmd(0xB2);                  /* PORCTRL */
    lcd_dat(0x0C); lcd_dat(0x0C); lcd_dat(0x00);
    lcd_dat(0x33); lcd_dat(0x33);

    lcd_cmd(0xB7); lcd_dat(0x35);   /* GCTRL */
    lcd_cmd(0xBB); lcd_dat(0x19);   /* VCOMS */
    lcd_cmd(0xC0); lcd_dat(0x2C);   /* LCMCTRL */
    lcd_cmd(0xC2); lcd_dat(0x01);   /* VDVVRHEN */
    lcd_cmd(0xC3); lcd_dat(0x12);   /* VRHS */
    lcd_cmd(0xC4); lcd_dat(0x20);   /* VDVS */
    lcd_cmd(0xC6); lcd_dat(0x0F);   /* FRCTRL2 */
    lcd_cmd(0xD0); lcd_dat(0xA4); lcd_dat(0xA1);   /* PWCTRL1 */

    lcd_cmd(0xE0);                  /* 正极性 Gamma */
    lcd_dat(0xD0); lcd_dat(0x08); lcd_dat(0x11); lcd_dat(0x08);
    lcd_dat(0x0C); lcd_dat(0x15); lcd_dat(0x39); lcd_dat(0x33);
    lcd_dat(0x50); lcd_dat(0x36); lcd_dat(0x13); lcd_dat(0x14);
    lcd_dat(0x29); lcd_dat(0x2D);

    lcd_cmd(0xE1);                  /* 负极性 Gamma */
    lcd_dat(0xD0); lcd_dat(0x08); lcd_dat(0x10); lcd_dat(0x08);
    lcd_dat(0x06); lcd_dat(0x06); lcd_dat(0x39); lcd_dat(0x44);
    lcd_dat(0x51); lcd_dat(0x0B); lcd_dat(0x16); lcd_dat(0x14);
    lcd_dat(0x2F); lcd_dat(0x31);

    lcd_cmd(0x20);                  /* 关闭反显 */
    lcd_cmd(0x13);                  /* NORON */
    lcd_cmd(0x29);                  /* DISPON */
    delay_ms(150);
}

static void lcd_set_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    lcd_cmd(0x2A);
    lcd_dat(0x00); lcd_dat((uint8_t)(x0 + 2));
    lcd_dat(0x00); lcd_dat((uint8_t)(x1 + 2));

    lcd_cmd(0x2B);
    lcd_dat(0x00); lcd_dat((uint8_t)(y0 + 1));
    lcd_dat(0x00); lcd_dat((uint8_t)(y1 + 1));

    lcd_cmd(0x2C);
}

/* ================= 帧缓冲 ================= */
static uint16_t fb[FB_SIZE];

static void fb_clear(void)
{
    uint32_t i;
    for (i = 0; i < FB_SIZE; i++) {
        fb[i] = C_BLACK;
    }
}

static void lcd_push(void)
{
    uint32_t i;

    lcd_set_window(0, 0, SCR_W - 1, SCR_H - 1);
    LCD_DC_HIGH();
    LCD_CS_LOW();
    for (i = 0; i < FB_SIZE; i++) {
        uint16_t c = fb[i];
        spi2_write((uint8_t)(c >> 8));
        spi2_write((uint8_t)(c & 0xFF));
    }
    spi2_flush();
    LCD_CS_HIGH();
}

/* ================= 绘制原语 ================= */
static uint16_t blend565(uint16_t bg, uint16_t fg, uint8_t a)
{
    int32_t r, g, b;

    if (a >= 250) return fg;
    if (a == 0) return bg;

    r = (((fg >> 11) & 0x1F) * a + ((bg >> 11) & 0x1F) * (255 - a)) / 255;
    g = (((fg >> 5) & 0x3F) * a + ((bg >> 5) & 0x3F) * (255 - a)) / 255;
    b = ((fg & 0x1F) * a + (bg & 0x1F) * (255 - a)) / 255;
    return (uint16_t)((r << 11) | (g << 5) | b);
}

static void px(int x, int y, uint16_t c, uint8_t a)
{
    if (x < 0 || y < 0 || x >= SCR_W || y >= SCR_H) return;
    if (a == 0) return;
    fb[y * SCR_W + x] = blend565(fb[y * SCR_W + x], c, a);
}

static void px_solid(int x, int y, uint16_t c)
{
    if (x < 0 || y < 0 || x >= SCR_W || y >= SCR_H) return;
    fb[y * SCR_W + x] = c;
}

/* Wu 抗锯齿线段 */
static void wu_line(float x0, float y0, float x1, float y1, uint16_t c)
{
    int steep;
    float t;
    float dx, dy, grad, y;
    int xs, xe, x, yi;
    float f;

    steep = (y1 > y0 ? y1 - y0 : y0 - y1) > (x1 > x0 ? x1 - x0 : x0 - x1);
    if (steep) {
        t = x0; x0 = y0; y0 = t;
        t = x1; x1 = y1; y1 = t;
    }
    if (x0 > x1) {
        t = x0; x0 = x1; x1 = t;
        t = y0; y0 = y1; y1 = t;
    }

    dx = x1 - x0;
    dy = y1 - y0;
    grad = (dx == 0.0f) ? 1.0f : dy / dx;

    y = y0 + grad * ((float)((int)(x0 + 0.5f)) - x0);
    xs = (int)(x0 + 0.5f);
    xe = (int)(x1 + 0.5f);

    for (x = xs; x <= xe; x++) {
        yi = (int)y;
        f = y - (float)yi;
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        if (steep) {
            px(yi, x, c, (uint8_t)(255 - (int32_t)(f * 255.0f)));
            px(yi + 1, x, c, (uint8_t)((int32_t)(f * 255.0f)));
            px(yi + 2, x, c, 116);              /* 加粗到约 2 像素 */
        } else {
            px(x, yi, c, (uint8_t)(255 - (int32_t)(f * 255.0f)));
            px(x, yi + 1, c, (uint8_t)((int32_t)(f * 255.0f)));
            px(x, yi + 2, c, 116);              /* 加粗到约 2 像素 */
        }
        y += grad;
    }
}

/* ================= 5x7 字库 ================= */
static const uint8_t font5x7[59][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14}, {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, {0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02}, {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00}, {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06}, {0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
};

static void draw_char(int x, int y, char ch, uint16_t col)
{
    int i, b;
    const uint8_t *g;

    if (ch < ' ' || ch > 'z') return;
    g = font5x7[ch - ' '];
    for (i = 0; i < 5; i++) {
        uint8_t v = g[i];
        for (b = 0; b < 7; b++) {
            if (v & (1u << b)) {
                px_solid(x + i, y + b, col);
            }
        }
    }
}

static void draw_str(int x, int y, const char *s, uint16_t col)
{
    while (*s) {
        draw_char(x, y, *s++, col);
        x += 6;
    }
}

static int str_w(const char *s)
{
    int n = 0;
    while (*s++) n++;
    return n * 6 - 1;
}

/* 格式化整数到 buf，返回长度 */
static int fmt_int(char *p, int32_t v)
{
    char t[12];
    int n = 0, i = 0;

    if (v < 0) {
        *p++ = '-';
        v = -v;
        n++;
    }
    if (v == 0) {
        *p++ = '0';
        return n + 1;
    }
    while (v > 0) {
        t[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i > 0) {
        *p++ = t[--i];
        n++;
    }
    return n;
}

/* ================= 3D 数学 ================= */
static float fsin(float a)
{
    const float PI = 3.14159265f;
    const float TWO_PI = 6.2831853f;
    float x2, r;

    while (a < 0.0f) a += TWO_PI;
    while (a >= TWO_PI) a -= TWO_PI;
    /* 折到 -pi/2 .. pi/2，泰勒精度最佳 */
    if (a > PI * 1.5f) a -= TWO_PI;
    if (a > PI * 0.5f) a = PI - a;
    if (a < -PI * 0.5f) a = -PI - a;

    x2 = a * a;
    r = a * (1.0f - x2 / 6.0f * (1.0f - x2 / 20.0f * (1.0f - x2 / 42.0f * (1.0f - x2 / 72.0f))));
    return r;
}

static float fcos(float a)
{
    return fsin(a + 1.57079633f);
}

static float fsqrt(float x)
{
    float r;
    int i;

    if (x <= 0.0f) return 0.0f;
    r = x > 1.0f ? x : 1.0f;
    for (i = 0; i < 16; i++) {
        r = 0.5f * (r + x / r);
    }
    return r;
}

/* atan，输入 0..1，输出 0..pi/4 */
static float fatan_pos(float s)
{
    float s2 = s * s;
    return s * (0.9998660f - s2 * (0.3302995f - s2 * (0.1801410f
             - s2 * (0.0851330f - s2 * 0.0208351f))));
}

static float fatan2(float y, float x)
{
    const float PI = 3.14159265f;
    float ax = x < 0 ? -x : x;
    float ay = y < 0 ? -y : y;
    float a;

    if (ax == 0.0f && ay == 0.0f) return 0.0f;

    if (ax >= ay) a = fatan_pos(ay / ax);
    else          a = PI * 0.5f - fatan_pos(ax / ay);

    if (x < 0.0f) a = PI - a;
    if (y < 0.0f) a = -a;
    return a;
}

/* ================= 姿态 ================= */
static float pitch_rad;
static float roll_rad;
static float yaw_rad;
static uint8_t att_inited;

static void attitude_update(float dt)
{
    float ax = (float)acc_mg[0];
    float ay = (float)acc_mg[1];
    float az = (float)acc_mg[2];
    float ap, ar;
    float gx = (float)gyro_dps10[0] / 10.0f;
    float gy = (float)gyro_dps10[1] / 10.0f;
    float gz = (float)gyro_dps10[2] / 10.0f;

    ap = fatan2(-ax, fsqrt(ay * ay + az * az));
    ar = fatan2(ay, az);

    if (!att_inited) {
        pitch_rad = ap;
        roll_rad = ar;
        att_inited = 1;
        return;
    }

    /* 互补滤波：陀螺积分 + 加速度校正（权重 0.18，约 0.2 秒跟平） */
    pitch_rad = pitch_rad + gx * dt * 0.0174533f + (ap - pitch_rad) * 0.18f;
    roll_rad  = roll_rad  + gy * dt * 0.0174533f + (ar - roll_rad)  * 0.18f;

    /* 偏航：无磁力计时用陀螺积分，缓慢回中抑制漂移 */
    yaw_rad = (yaw_rad + gz * dt * 0.0174533f) * 0.99f;

    if (pitch_rad > 1.4f) pitch_rad = 1.4f;
    if (pitch_rad < -1.4f) pitch_rad = -1.4f;
    if (roll_rad > 1.4f) roll_rad = 1.4f;
    if (roll_rad < -1.4f) roll_rad = -1.4f;
    if (yaw_rad > 1.4f) yaw_rad = 1.4f;
    if (yaw_rad < -1.4f) yaw_rad = -1.4f;
}

/* ================= 飞机模型 ================= */
static const float VX[19] = {
      0,   0,   0, -3.6f, 3.6f,   0, -3.6f, 3.6f,
    -27,  27, -3.6f, 3.6f,   0,   0,   0, -11,
     11, -3.6f, 3.6f
};
static const float VY[19] = {
      0, 3.6f, -3.6f, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 3.6f, 15, 0.5f, 0,
      0, 0, 0
};
static const float VZ[19] = {
     24,  6,  6,  6,  6, -22,  4,  4,
    -12, -12, -9, -9, -11, -20, -22, -19,
    -19, -14, -14
};

static const uint8_t EDGE[25][2] = {
    {0,1},{0,2},{0,3},{0,4},
    {1,5},{2,5},{3,5},{4,5},
    {1,3},{3,2},{2,4},{4,1},
    {6,8},{8,10},{10,6},
    {7,9},{9,11},{11,7},
    {12,13},{13,14},{14,12},
    {15,16},{15,17},{16,18},{17,18}
};

static const float FOCAL = 58.0f;
static const float CXF = 64.0f;
static const float CYF = 63.0f;

/* 标准大气压高度表：550~1100 hPa，步长 2 hPa，单位米 */
static const int16_t alt_tab[276] = {
    4866, 4839, 4812, 4784, 4757, 4730, 4704, 4677, 4650, 4623, 4597, 4570,
    4544, 4518, 4491, 4465, 4439, 4413, 4387, 4361, 4335, 4309, 4284, 4258,
    4233, 4207, 4182, 4156, 4131, 4106, 4081, 4056, 4031, 4006, 3981, 3956,
    3931, 3907, 3882, 3857, 3833, 3808, 3784, 3760, 3735, 3711, 3687, 3663,
    3639, 3615, 3591, 3567, 3544, 3520, 3496, 3473, 3449, 3426, 3402, 3379,
    3356, 3332, 3309, 3286, 3263, 3240, 3217, 3194, 3171, 3148, 3126, 3103,
    3080, 3058, 3035, 3013, 2990, 2968, 2946, 2923, 2901, 2879, 2857, 2835,
    2813, 2791, 2769, 2747, 2725, 2703, 2681, 2660, 2638, 2616, 2595, 2573,
    2552, 2531, 2509, 2488, 2467, 2445, 2424, 2403, 2382, 2361, 2340, 2319,
    2298, 2277, 2256, 2236, 2215, 2194, 2174, 2153, 2132, 2112, 2091, 2071,
    2051, 2030, 2010, 1990, 1970, 1949, 1929, 1909, 1889, 1869, 1849, 1829,
    1809, 1789, 1769, 1750, 1730, 1710, 1691, 1671, 1651, 1632, 1612, 1593,
    1573, 1554, 1535, 1515, 1496, 1477, 1458, 1438, 1419, 1400, 1381, 1362,
    1343, 1324, 1305, 1286, 1267, 1249, 1230, 1211, 1192, 1174, 1155, 1136,
    1118, 1099, 1081, 1062, 1044, 1025, 1007,  989,  970,  952,  934,  916,
     897,  879,  861,  843,  825,  807,  789,  771,  753,  735,  717,  700,
     682,  664,  646,  629,  611,  593,  576,  558,  540,  523,  505,  488,
     470,  453,  436,  418,  401,  384,  366,  349,  332,  315,  298,  281,
     263,  246,  229,  212,  195,  178,  162,  145,  128,  111,   94,   77,
      61,   44,   27,   10,   -6,  -23,  -39,  -56,  -73,  -89, -106, -122,
    -139, -155, -171, -188, -204, -220, -237, -253, -269, -285, -302, -318,
    -334, -350, -366, -382, -398, -414, -430, -446, -462, -478, -494, -510,
    -526, -541, -557, -573, -589, -605, -620, -636, -652, -667, -683, -698
};

static int32_t altitude_of(int32_t hpa10)
{
    int32_t idx, frac;

    if (hpa10 < 5500) hpa10 = 5500;
    if (hpa10 > 11000) hpa10 = 11000;
    idx = (hpa10 - 5500) / 20;
    if (idx >= 275) idx = 274;
    frac = (hpa10 - 5500) % 20;
    return (int32_t)alt_tab[idx] + ((int32_t)(alt_tab[idx + 1] - alt_tab[idx]) * frac) / 20;
}

/* 相机固定观察角：俯角 30° + 侧偏 22°。
   正后方水平视角下机翼会投影成一条线，看不出是飞机 */
static const float CAM_ELEV = 0.52f;    /* 俯角 30° */
static const float CAM_YAW  = 0.38f;    /* 侧偏 22° */
static const float GAIN     = 1.45f;    /* 整体放大，占满屏幕 */
static const float VIEW_GAIN = 2.8f;    /* 姿态放大：小幅晃动也能看清 */

static void render_plane(void)
{
    float sx[19], sy[19];
    float p, r;
    float cx, sa, cc, sc, cyb, syb;
    float ce, se, cyw, syw;
    int i;

    fb_clear();

    /* 固定地平线参考线，倾斜方向靠它对比出来 */
    for (i = 0; i < SCR_W; i += 4) {
        px_solid(i, 63, 0x2945);
        px_solid(i + 1, 63, 0x2945);
    }

    /* 姿态放大后限幅，避免翻出屏幕 */
    p = pitch_rad * VIEW_GAIN;
    r = roll_rad  * VIEW_GAIN;
    if (p >  1.45f) p =  1.45f;
    if (p < -1.45f) p = -1.45f;
    if (r >  1.45f) r =  1.45f;
    if (r < -1.45f) r = -1.45f;

    cx = fcos(p);  sa = fsin(p);
    cc = fcos(r);  sc = fsin(r);
    cyb = fcos(yaw_rad); syb = fsin(yaw_rad);
    ce = fcos(CAM_ELEV);  se = fsin(CAM_ELEV);
    cyw = fcos(CAM_YAW);  syw = fsin(CAM_YAW);

    for (i = 0; i < 19; i++) {
        float x = VX[i], y = VY[i], z = VZ[i];
        float y1, z1, x3, y3, x5, z5, y4, z3, x4, z4, s;

        /* 机体姿态：俯仰 -> 横滚 -> 偏航 */
        y1 = y * cx - z * sa;
        z1 = y * sa + z * cx;
        x3 = x * cc - y1 * sc;
        y3 = x * sc + y1 * cc;
        x5 = x3 * cyb - z1 * syb;
        z5 = x3 * syb + z1 * cyb;

        /* 相机：固定俯角 + 侧偏角，形成 3/4 追尾视角 */
        y4 =  y3 * ce + z5 * se;
        z3 = -y3 * se + z5 * ce;
        x4 =  x5 * cyw + z3 * syw;
        z4 = -x5 * syw + z3 * cyw;

        s = (FOCAL / (FOCAL + z4)) * GAIN;
        sx[i] = CXF + x4 * s;
        sy[i] = CYF - y4 * s;
    }

    for (i = 0; i < 25; i++) {
        int a = EDGE[i][0];
        int b = EDGE[i][1];
        float zm = (VZ[a] + VZ[b]) * 0.5f;
        float t = (zm + 22.0f) / 46.0f;
        int32_t rr, gg, bb;
        uint16_t col;

        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        rr = (int32_t)(48.0f + (1.0f - t) * 90.0f);
        gg = (int32_t)(140.0f + (1.0f - t) * 105.0f);
        bb = (int32_t)(160.0f + (1.0f - t) * 90.0f);
        col = (uint16_t)(((rr >> 3) << 11) | ((gg >> 2) << 5) | (bb >> 3));

        wu_line(sx[a], sy[a], sx[b], sy[b], col);
    }

    /* 翼尖航行灯：左红右绿，用来判断往哪边侧 */
    {
        int lx = (int)(sx[8] + 0.5f), ly = (int)(sy[8] + 0.5f);
        int rx = (int)(sx[9] + 0.5f), ry = (int)(sy[9] + 0.5f);

        px_solid(lx - 1, ly - 1, 0xF800); px_solid(lx, ly - 1, 0xF800);
        px_solid(lx - 1, ly,     0xF800); px_solid(lx, ly,     0xF800);
        px_solid(rx - 1, ry - 1, 0x07E0); px_solid(rx, ry - 1, 0x07E0);
        px_solid(rx - 1, ry,     0x07E0); px_solid(rx, ry,     0x07E0);
    }

    /* 中心准星 */
    px_solid(63, 62, 0x04D1);
    px_solid(64, 62, 0x04D1);

    /* 数据 */
    {
        char b[12];
        int n;
        int32_t alt = altitude_of(press_hpa10);
        int32_t am = (int32_t)fsqrt((float)(acc_mg[0]*acc_mg[0] + acc_mg[1]*acc_mg[1] + acc_mg[2]*acc_mg[2]));

        draw_str(3, 3, "ALT", 0x2C94);
        n = fmt_int(b, alt);
        b[n++] = 'M';
        b[n] = '\0';
        draw_str(124 - str_w(b), 3, b, 0x7FF7);

        draw_str(3, 118, "ACC", 0x2C94);
        n = fmt_int(b, am);
        b[n++] = 'M';
        b[n++] = 'G';
        b[n] = '\0';
        draw_str(124 - str_w(b), 118, b, 0x7FF7);
    }
}

/* ================= 时钟 ================= */
static uint8_t bcd2bin(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static uint8_t bin2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

static uint8_t rtc_ok;
static uint32_t soft_sec;                       /* RTC 不可用时软件计时 */

static void rtc_init(void)
{
    volatile uint32_t t;

    RCC_APB1ENR |= (1UL << 28);
    PWR_CR |= (1UL << 8);                   /* DBP */

    /* 先试 LSE */
    RCC_BDCR |= (1UL << 0);
    t = 0;
    while ((RCC_BDCR & (1UL << 1)) == 0 && t < 4000000UL) {
        t++;
    }

    if (RCC_BDCR & (1UL << 1)) {
        /* LSE 起振，32768Hz */
        RCC_BDCR = (RCC_BDCR & ~(3UL << 8)) | (1UL << 8);
        rtc_ok = 1;
    } else {
        /* 退回 LSI */
        RCC_CSR |= (1UL << 0);
        t = 0;
        while ((RCC_CSR & (1UL << 1)) == 0 && t < 4000000UL) {
            t++;
        }
        if ((RCC_CSR & (1UL << 1)) == 0) {
            rtc_ok = 0;                     /* 两种时钟源都不可用 */
            return;
        }
        RCC_BDCR = (RCC_BDCR & ~(3UL << 8)) | (2UL << 8);
        rtc_ok = 2;
    }

    RCC_BDCR |= (1UL << 15);                /* RTCEN */
    delay_ms(10);

    RTC_WPR = 0xCA;
    RTC_WPR = 0x53;

    RTC_ISR = (1UL << 7);                   /* INIT，直接写避免读改写副作用 */
    t = 0;
    while ((RTC_ISR & (1UL << 6)) == 0 && t < 2000000UL) {
        t++;
    }
    if (t >= 2000000UL) {
        rtc_ok = 0;                         /* RTC 无响应，转软件计时 */
        RTC_WPR = 0xFF;
        return;
    }

    if (rtc_ok == 1) {
        RTC_PRER = (127UL << 16) | 255UL;   /* LSE 32768 */
    } else {
        RTC_PRER = (127UL << 16) | 249UL;   /* LSI ~32000 */
    }

    /* 只有从未对过时（备份寄存器无标记）才写默认 12:00:00，
       否则每次复位都会把时间冲回 12 点 */
    if (RTC_BKP0R != 0x32F2UL) {
        RTC_TR = (bin2bcd(12) << 16) | (bin2bcd(0) << 8) | bin2bcd(0);
        RTC_DR = (bin2bcd(1) << 16) | (bin2bcd(1) << 8) | bin2bcd(1);
    }

    RTC_ISR &= ~(1UL << 7);
    t = 0;
    while ((RTC_ISR & (1UL << 6)) && t < 200000UL) {
        t++;
    }
    RTC_WPR = 0xFF;
    RTC_BKP0R = 0x32F2UL;                   /* 标记：已完成初始化 */
}

/* 串口对时：写入 RTC，同时同步软件秒计数 */
static void rtc_set_time(uint8_t hh, uint8_t mm, uint8_t ss)
{
    volatile uint32_t t;
    uint32_t tr;

    soft_sec = (uint32_t)hh * 3600UL + (uint32_t)mm * 60UL + (uint32_t)ss;
    if (rtc_ok == 0) {
        return;
    }

    RTC_WPR = 0xCA;
    RTC_WPR = 0x53;
    RTC_ISR = (1UL << 7);
    t = 0;
    while ((RTC_ISR & (1UL << 6)) == 0 && t < 2000000UL) {
        t++;
    }
    if (t >= 2000000UL) {
        RTC_WPR = 0xFF;
        return;
    }

    if (rtc_ok == 1) {
        RTC_PRER = (127UL << 16) | 255UL;
    } else {
        RTC_PRER = (127UL << 16) | 249UL;
    }

    tr = ((uint32_t)bin2bcd(hh) << 16) | ((uint32_t)bin2bcd(mm) << 8) | bin2bcd(ss);
    RTC_TR = tr;

    RTC_ISR &= ~(1UL << 7);
    t = 0;
    while ((RTC_ISR & (1UL << 6)) && t < 200000UL) {
        t++;
    }
    RTC_WPR = 0xFF;
}

/* 协议：T + HHMMSS + 回车换行，例 "T142530\r\n" */
static void uart_time_poll(void)
{
    static char buf[8];
    static uint8_t n;

    while ((USART2_SR & (1UL << 5)) != 0) {         /* RXNE */
        char c = (char)(USART2_DR & 0xFF);

        if (n == 0) {
            if (c == 'T') buf[n++] = c;
        } else if (c >= '0' && c <= '9' && n < 7) {
            buf[n++] = c;
        } else if ((c == '\r' || c == '\n') && n == 7) {
            uint8_t hh = (uint8_t)((buf[1]-'0')*10 + (buf[2]-'0'));
            uint8_t mm = (uint8_t)((buf[3]-'0')*10 + (buf[4]-'0'));
            uint8_t ss = (uint8_t)((buf[5]-'0')*10 + (buf[6]-'0'));

            if (hh < 24 && mm < 60 && ss < 60) {
                rtc_set_time(hh, mm, ss);
                uart2_puts("TIME OK\r\n");
            }
            n = 0;
        } else {
            n = 0;                                   /* 非法字符，丢弃重来 */
        }
    }
}

static void rtc_read(uint8_t *h, uint8_t *m, uint8_t *s)
{
    uint32_t v;
    volatile uint32_t t = 0;

    if (rtc_ok == 0) {                      /* RTC 不可用，用软件计时 */
        *h = (uint8_t)((soft_sec / 3600UL) % 24UL);
        *m = (uint8_t)((soft_sec / 60UL) % 60UL);
        *s = (uint8_t)(soft_sec % 60UL);
        return;
    }

    while ((RTC_ISR & (1UL << 5)) == 0 && t < 200000UL) {   /* RSF，带超时 */
        t++;
    }
    v = RTC_TR;
    *h = bcd2bin((uint8_t)((v >> 16) & 0x3F));
    *m = bcd2bin((uint8_t)((v >> 8) & 0x7F));
    *s = bcd2bin((uint8_t)(v & 0x7F));
}

static void draw_clock_hand(float ang, float len, uint16_t col, int thick)
{
    float ex = 64.0f + fcos(ang) * len;
    float ey = 61.0f + fsin(ang) * len;

    wu_line(64.0f, 61.0f, ex, ey, col);
    if (thick) {
        float nx = -fsin(ang) * 1.2f;
        float ny = fcos(ang) * 1.2f;
        wu_line(64.0f + nx, 61.0f + ny, ex + nx, ey + ny, col);
    }
}

static void render_clock(void)
{
    const float PI = 3.14159265f;
    float px0, py0, a;
    int i;
    uint8_t hh, mm, ss;
    char b[10];
    int n;

    fb_clear();

    /* 外圆 */
    px0 = 64.0f + 50.0f;
    py0 = 61.0f;
    for (i = 1; i <= 120; i++) {
        float x, y;
        a = (float)i / 120.0f * 2.0f * PI;
        x = 64.0f + fcos(a) * 50.0f;
        y = 61.0f + fsin(a) * 50.0f;
        wu_line(px0, py0, x, y, 0x1FEA);
        px0 = x; py0 = y;
    }

    /* 刻度 */
    for (i = 0; i < 12; i++) {
        float ang = (float)i / 12.0f * 2.0f * PI - 1.5707963f;
        int major = (i % 3 == 0);
        float len = major ? 8.0f : 4.0f;
        uint16_t c = major ? 0xDFFF : 0x2D93;
        wu_line(64.0f + fcos(ang) * 46.0f, 61.0f + fsin(ang) * 46.0f,
                64.0f + fcos(ang) * (46.0f - len), 61.0f + fsin(ang) * (46.0f - len), c);
    }

    rtc_read(&hh, &mm, &ss);

    /* 指针 */
    draw_clock_hand((float)hh / 12.0f * 2.0f * PI - 1.5707963f, 24.0f, 0xDFFF, 1);
    draw_clock_hand((float)mm / 60.0f * 2.0f * PI - 1.5707963f, 35.0f, 0x7FF7, 1);
    draw_clock_hand((float)ss / 60.0f * 2.0f * PI - 1.5707963f, 43.0f, 0xFAA0, 0);

    /* 轴心 */
    px_solid(63, 60, 0xDFFF);
    px_solid(64, 60, 0xDFFF);
    px_solid(63, 61, 0xDFFF);
    px_solid(64, 61, 0xDFFF);

    /* 数字时间 */
    n = 0;
    b[n++] = (char)('0' + hh / 10);
    b[n++] = (char)('0' + hh % 10);
    b[n++] = ':';
    b[n++] = (char)('0' + mm / 10);
    b[n++] = (char)('0' + mm % 10);
    b[n++] = ':';
    b[n++] = (char)('0' + ss / 10);
    b[n++] = (char)('0' + ss % 10);
    b[n] = '\0';
    draw_str(64 - str_w(b) / 2, 118, b, 0x7FF7);
}

/* ================= 静止检测（按真实时间累计，与帧率无关） ================= */
static uint32_t idle_ms;
static uint32_t idle_cyc;
static uint8_t show_clock;

static void check_idle(void)
{
    int32_t am = (int32_t)fsqrt((float)(acc_mg[0]*acc_mg[0] + acc_mg[1]*acc_mg[1] + acc_mg[2]*acc_mg[2]));
    int32_t gm = (gyro_dps10[0] < 0 ? -gyro_dps10[0] : gyro_dps10[0])
               + (gyro_dps10[1] < 0 ? -gyro_dps10[1] : gyro_dps10[1])
               + (gyro_dps10[2] < 0 ? -gyro_dps10[2] : gyro_dps10[2]);
    int32_t d = am - 1000;
    uint32_t now = DWT_CYCCNT;
    uint32_t dt  = (uint32_t)(now - idle_cyc) / (sysclk_mhz * 1000UL);

    idle_cyc = now;
    if (dt > 100) dt = 100;             /* 异常跳变钳位 */

    if (d < 0) d = -d;

    if (d < 100 && gm < 150) {
        if (idle_ms < 60000UL) idle_ms += dt;
    } else {
        idle_ms = 0;
    }
    /* 静止 20 秒后切时钟表盘 */
    show_clock = (idle_ms >= 20000UL) ? 1 : 0;
}

/* ================= main ================= */
int main(void)
{
    uint32_t slow_cnt = 0;
    uint32_t start_ms = 0;
    uint32_t last_cyc = 0;

    /* 先用默认 HSI 16MHz 打开串口，保证任何阶段失败都有输出 */
    uart2_init();
    uart2_puts("\r\n=== BOOT ===\r\n");

    clock_init();
    RCC_AHB1ENR |= (1UL << 0) | (1UL << 1);

    /* 启动 DWT 周期计数器，供软件计时用 */
    DEMCR |= (1UL << 24);
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1UL;

    uart2_init();                       /* 按实际主频重设波特率 */
    uart2_puts("CLK=");
    uart2_int((int32_t)sysclk_mhz);
    uart2_puts("MHz\r\n");

    i2c1_init();
    uart2_puts("I2C OK\r\n");
    spi2_init();
    uart2_puts("SPI OK\r\n");
    lcd_init();
    uart2_puts("LCD OK\r\n");
    rtc_init();
    uart2_puts("RTC src=");
    uart2_int(rtc_ok);
    uart2_puts("\r\n");

    fb_clear();
    lcd_push();
    uart2_puts("FB OK\r\n");

    uart2_puts("\r\n[Cannon] 3D flight display\r\n");
    uart2_puts("SYSCLK: ");
    uart2_int((int32_t)sysclk_mhz);
    uart2_puts("MHz\r\nRTC src: ");
    uart2_puts(rtc_ok == 1 ? "LSE" : "LSI");
    uart2_puts("\r\n");

    sensors_init();
    uart2_puts("SENSORS OK\r\n");
    delay_ms(300);                  /* 等各传感器完成首次转换 */

    read_lsm6ds3();
    read_lsm303a();
    read_lsm303m();
    read_lps25h();
    hts_simple();
    attitude_update(0.02f);

    while (1) {
        uart_time_poll();
        read_lsm6ds3();
        attitude_update(0.02f);
        check_idle();

        if (++slow_cnt >= 25) {         /* 25 x 20ms = 500ms */
            slow_cnt = 0;
            read_lsm303a();
            read_lsm303m();
            read_lps25h();
            hts_simple();
        }

        if (show_clock) {
            render_clock();
        } else {
            render_plane();
        }
        lcd_push();

        /* 软件秒计时：RTC 不可用时时钟仍能走 */
        {
            uint32_t now = DWT_CYCCNT;
            if ((uint32_t)(now - last_cyc) >= (sysclk_mhz * 1000000UL)) {
                last_cyc += sysclk_mhz * 1000000UL;
                soft_sec++;
            }
        }

        start_ms++;
        if ((start_ms % 20) == 0) {
            uart2_puts("t=");
            uart2_int((int32_t)start_ms);
            uart2_puts(" pitch=");
            uart2_int((int32_t)(pitch_rad * 57.2958f));
            uart2_puts(" roll=");
            uart2_int((int32_t)(roll_rad * 57.2958f));
            uart2_puts(" yaw=");
            uart2_int((int32_t)(yaw_rad * 57.2958f));
            uart2_puts(" clock=");
            uart2_int(show_clock);
            uart2_puts("\r\n");
        }

        delay(50000);
    }
}
