/* Cannon V3 (STM32F401RET6) 五路传感器实时显示到 ST7735S 128x128
 *
 * 传感器（I2C1: PB6=SCL / PB7=SDA, 100kHz）
 *   0x6B LSM6DS3    六轴 IMU
 *   0x19 LSM303AGR  加速度
 *   0x1E LSM303AGR  磁力计
 *   0x5D LPS25H     气压 + 温度
 *   0x5F HTS221     温度 + 湿度
 *
 * 屏幕（SPI2: PB13=SCK / PB15=MOSI / PB12=CS / PB14=DC, 8MHz）
 *   3V3 -> VCC + BL + RES，GND -> GND
 *   MADCTL = 0x00（实测唯一正确方向），关闭反显
 *
 * 串口：USART2 PA2=TX / PA3=RX 115200 8N1
 * 屏幕和串口同时输出
 */

#include <stdint.h>

#define REG(a)          (*(volatile uint32_t *)(a))

/* ---------------- RCC ---------------- */
#define RCC_AHB1ENR     REG(0x40023830UL)
#define RCC_APB1ENR     REG(0x40023840UL)

/* ---------------- GPIOA ---------------- */
#define GPIOA_MODER     REG(0x40020000UL)
#define GPIOA_AFRL      REG(0x40020020UL)

/* ---------------- GPIOB ---------------- */
#define GPIOB_MODER     REG(0x40020400UL)
#define GPIOB_OTYPER    REG(0x40020404UL)
#define GPIOB_OSPEEDR   REG(0x40020408UL)
#define GPIOB_PUPDR     REG(0x4002040CUL)
#define GPIOB_BSRR      REG(0x40020418UL)
#define GPIOB_AFRL      REG(0x40020420UL)
#define GPIOB_AFRH      REG(0x40020424UL)

/* ---------------- USART2 ---------------- */
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

/* LCD 控制脚 */
#define CS_LOW()        (GPIOB_BSRR = (1UL << (12 + 16)))
#define CS_HIGH()       (GPIOB_BSRR = (1UL << 12))
#define DC_LOW()        (GPIOB_BSRR = (1UL << (14 + 16)))
#define DC_HIGH()       (GPIOB_BSRR = (1UL << 14))

/* 显存偏移 */
#define X_OFF           2
#define Y_OFF           1

/* 颜色 RGB565 */
#define C_BLACK         0x0000
#define C_WHITE         0xFFFF
#define C_RED           0xF800
#define C_GREEN         0x07E0
#define C_BLUE          0x001F
#define C_YELLOW        0xFFE0
#define C_CYAN          0x07FF
#define C_MAGENTA       0xF81F
#define C_GRAY          0x8410

static void delay(volatile uint32_t n)
{
    while (n--) {
        __asm volatile("nop");
    }
}

/* ================= USART2 ================= */
static void uart2_init(void)
{
    GPIOA_MODER &= ~((3UL << 4) | (3UL << 6));
    GPIOA_MODER |=  ((2UL << 4) | (2UL << 6));
    GPIOA_AFRL  &= ~((0xFUL << 8) | (0xFUL << 12));
    GPIOA_AFRL  |=  ((7UL << 8) | (7UL << 12));
    RCC_APB1ENR |= (1UL << 17);
    USART2_BRR = 139;
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

static void uart2_fixed1(int32_t v)
{
    if (v < 0) {
        uart2_putc('-');
        v = -v;
    }
    uart2_int(v / 10);
    uart2_putc('.');
    uart2_putc((char)('0' + (v % 10)));
}

/* ================= I2C1 ================= */
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
    I2C1_CR2   = 16;
    I2C1_CCR   = 80;
    I2C1_TRISE = 17;
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
    delay(300);
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
static uint8_t dev_lsm6ds3 = 0;
static uint8_t dev_lsm303a = 0;
static uint8_t dev_lsm303m = 0;
static uint8_t dev_lps25h  = 0;
static uint8_t dev_hts221  = 0;

static int16_t id_lsm6ds3 = -1;
static int16_t id_lsm303a = -1;
static int16_t id_lsm303m = -1;
static int16_t id_lps25h  = -1;
static int16_t id_hts221  = -1;

static void sensors_init(void)
{
    uint8_t id;

    /* LSM6DS3 0x6B */
    id = 0xFF;
    if (i2c1_read_reg(0x6B, 0x0F, &id)) {
        id_lsm6ds3 = id;
    }
    if (id == 0x69) {
        i2c1_write_reg(0x6B, 0x10, 0x40);
        i2c1_write_reg(0x6B, 0x11, 0x40);
        i2c1_write_reg(0x6B, 0x12, 0x44);
        dev_lsm6ds3 = 1;
    }

    /* LSM303AGR 加速度 0x19 */
    id = 0xFF;
    if (i2c1_read_reg(0x19, 0x0F, &id)) {
        id_lsm303a = id;
    }
    if (id == 0x33) {
        i2c1_write_reg(0x19, 0x20, 0x57);
        i2c1_write_reg(0x19, 0x23, 0x88);
        dev_lsm303a = 1;
    }

    /* LSM303AGR 磁力 0x1E */
    id = 0xFF;
    if (i2c1_read_reg(0x1E, 0x4F, &id)) {
        id_lsm303m = id;
    }
    if (id == 0x40) {
        i2c1_write_reg(0x1E, 0x60, 0x40);
        delay(20000);
        i2c1_write_reg(0x1E, 0x61, 0x00);
        i2c1_write_reg(0x1E, 0x62, 0x10);
        i2c1_write_reg(0x1E, 0x60, 0x00);
        dev_lsm303m = 1;
    }

    /* LPS25H 0x5D */
    id = 0xFF;
    if (i2c1_read_reg(0x5D, 0x0F, &id)) {
        id_lps25h = id;
    }
    if (id == 0xBD) {
        i2c1_write_reg(0x5D, 0x20, 0xC4);
        dev_lps25h = 1;
    }

    /* HTS221 0x5F */
    id = 0xFF;
    if (i2c1_read_reg(0x5F, 0x0F, &id)) {
        id_hts221 = id;
    }
    if (id == 0xBC) {
        i2c1_write_reg(0x5F, 0x20, 0x81);
        i2c1_write_reg(0x5F, 0x21, 0x00);
        dev_hts221 = 1;
    }
}

static int32_t acc_mg[3];
static int32_t gyro_dps10[3];
static int32_t acc2_mg[3];
static int32_t mag_ut10[3];
static int32_t press_hpa10;
static int32_t press_temp_c10;
static int32_t hts_temp_c10;
static int32_t hts_hum_p10;

static void read_lsm6ds3(void)
{
    uint8_t lo, hi;
    int i;
    static const uint8_t acc_reg[3]  = { 0x28, 0x2A, 0x2C };
    static const uint8_t gyro_reg[3] = { 0x22, 0x24, 0x26 };

    if (!dev_lsm6ds3) {
        return;
    }
    for (i = 0; i < 3; i++) {
        if (i2c1_read_reg(0x6B, acc_reg[i], &lo) &&
            i2c1_read_reg(0x6B, (uint8_t)(acc_reg[i] + 1), &hi)) {
            int16_t raw = make_s16(lo, hi);
            acc_mg[i] = ((int32_t)raw * 61) / 1000;
        }
        if (i2c1_read_reg(0x6B, gyro_reg[i], &lo) &&
            i2c1_read_reg(0x6B, (uint8_t)(gyro_reg[i] + 1), &hi)) {
            int16_t raw = make_s16(lo, hi);
            gyro_dps10[i] = ((int32_t)raw * 875) / 10000;
        }
    }
}

static void read_lsm303a(void)
{
    uint8_t lo, hi;
    int i;
    static const uint8_t reg[3] = { 0x28, 0x2A, 0x2C };

    if (!dev_lsm303a) {
        return;
    }
    for (i = 0; i < 3; i++) {
        if (i2c1_read_reg(0x19, reg[i], &lo) &&
            i2c1_read_reg(0x19, (uint8_t)(reg[i] + 1), &hi)) {
            int16_t raw = make_s16(lo, hi);
            acc2_mg[i] = (int32_t)raw >> 4;
        }
    }
}

static void read_lsm303m(void)
{
    uint8_t lo, hi;
    int i;
    static const uint8_t reg[3] = { 0x68, 0x6A, 0x6C };

    if (!dev_lsm303m) {
        return;
    }
    for (i = 0; i < 3; i++) {
        if (i2c1_read_reg(0x1E, reg[i], &lo) &&
            i2c1_read_reg(0x1E, (uint8_t)(reg[i] + 1), &hi)) {
            int16_t raw = make_s16(lo, hi);
            mag_ut10[i] = ((int32_t)raw * 15) / 10;
        }
    }
}

static void read_lps25h(void)
{
    uint8_t xl, l, h, tl, th;
    int32_t p_raw;
    int16_t t_raw;

    if (!dev_lps25h) {
        return;
    }
    if (i2c1_read_reg(0x5D, 0x28, &xl) &&
        i2c1_read_reg(0x5D, 0x29, &l) &&
        i2c1_read_reg(0x5D, 0x2A, &h)) {
        p_raw = (int32_t)((uint32_t)xl | ((uint32_t)l << 8) | ((uint32_t)h << 16));
        if (p_raw & 0x00800000) {
            p_raw |= (int32_t)0xFF000000;
        }
        press_hpa10 = (p_raw * 10) / 4096;
    }
    if (i2c1_read_reg(0x5D, 0x2B, &tl) &&
        i2c1_read_reg(0x5D, 0x2C, &th)) {
        t_raw = make_s16(tl, th);
        press_temp_c10 = (int32_t)(((t_raw * 10) / 480) + 425);
    }
}

static void read_hts221(void)
{
    uint8_t b;
    int32_t t0, t1, t0_out, t1_out, t_raw;
    int32_t h0, h1, h0_out, h1_out, h_raw;
    uint8_t t0x8, t1x8, msb;
    int16_t s;

    if (!dev_hts221) {
        return;
    }

    if (!i2c1_read_reg(0x5F, 0x32, &t0x8)) {
        return;
    }
    if (!i2c1_read_reg(0x5F, 0x33, &t1x8)) {
        return;
    }
    if (!i2c1_read_reg(0x5F, 0x35, &msb)) {
        return;
    }
    t0 = (int32_t)((uint32_t)t0x8 | (((uint32_t)msb & 0x03) << 8));
    t1 = (int32_t)((uint32_t)t1x8 | (((uint32_t)msb & 0x0C) << 6));

    {
        uint8_t lo, hi;
        if (!i2c1_read_reg(0x5F, 0x3C, &lo) || !i2c1_read_reg(0x5F, 0x3D, &hi)) {
            return;
        }
        t0_out = make_s16(lo, hi);
        if (!i2c1_read_reg(0x5F, 0x3E, &lo) || !i2c1_read_reg(0x5F, 0x3F, &hi)) {
            return;
        }
        t1_out = make_s16(lo, hi);
        if (!i2c1_read_reg(0x5F, 0x2A, &lo) || !i2c1_read_reg(0x5F, 0x2B, &hi)) {
            return;
        }
        s = make_s16(lo, hi);
        t_raw = s;
    }

    if (t1_out != t0_out) {
        int32_t t8 = t0 + ((t1 - t0) * (t_raw - t0_out)) / (t1_out - t0_out);
        hts_temp_c10 = (t8 * 5) / 4;
    } else {
        hts_temp_c10 = (t0 * 5) / 4;
    }

    if (!i2c1_read_reg(0x5F, 0x30, &b)) {
        return;
    }
    h0 = ((int32_t)b * 10) / 2;
    if (!i2c1_read_reg(0x5F, 0x31, &b)) {
        return;
    }
    h1 = ((int32_t)b * 10) / 2;
    {
        uint8_t lo, hi;
        if (!i2c1_read_reg(0x5F, 0x36, &lo) || !i2c1_read_reg(0x5F, 0x37, &hi)) {
            return;
        }
        h0_out = make_s16(lo, hi);
        if (!i2c1_read_reg(0x5F, 0x3A, &lo) || !i2c1_read_reg(0x5F, 0x3B, &hi)) {
            return;
        }
        h1_out = make_s16(lo, hi);
        if (!i2c1_read_reg(0x5F, 0x28, &lo) || !i2c1_read_reg(0x5F, 0x29, &hi)) {
            return;
        }
        h_raw = make_s16(lo, hi);
    }

    if (h1_out != h0_out) {
        hts_hum_p10 = h0 + ((h1 - h0) * (h_raw - h0_out)) / (h1_out - h0_out);
        if (hts_hum_p10 < 0) {
            hts_hum_p10 = 0;
        }
        if (hts_hum_p10 > 1000) {
            hts_hum_p10 = 1000;
        }
    } else {
        hts_hum_p10 = h0;
    }
}

/* ================= SPI2 + ST7735 ================= */
static void spi2_init(void)
{
    RCC_AHB1ENR |= (1UL << 1);
    RCC_APB1ENR |= (1UL << 14);

    GPIOB_MODER &= ~((3UL << 24) | (3UL << 28));
    GPIOB_MODER |=  ((1UL << 24) | (1UL << 28));

    GPIOB_MODER &= ~((3UL << 26) | (3UL << 30));
    GPIOB_MODER |=  ((2UL << 26) | (2UL << 30));

    GPIOB_AFRH  &= ~((0xFUL << 20) | (0xFUL << 28));
    GPIOB_AFRH  |=  ((5UL << 20) | (5UL << 28));

    GPIOB_OSPEEDR |= (3UL << 24) | (3UL << 26) | (3UL << 28) | (3UL << 30);

    CS_HIGH();
    DC_HIGH();

    SPI2_CR1 = (1UL << 2) | (1UL << 8) | (1UL << 9) | (1UL << 6);
}

static void spi_write(uint8_t d)
{
    volatile uint32_t t = 0;

    while (((SPI2_SR & (1UL << 1)) == 0) && t < 100000) {
        t++;
    }
    SPI2_DR = (uint32_t)d;
}

static void spi_flush(void)
{
    volatile uint32_t t = 0;

    while (((SPI2_SR & (1UL << 7)) != 0) && t < 100000) {
        t++;
    }
}

static void lcd_cmd(uint8_t c)
{
    DC_LOW();
    CS_LOW();
    spi_write(c);
    spi_flush();
    CS_HIGH();
}

static void lcd_dat(uint8_t d)
{
    DC_HIGH();
    CS_LOW();
    spi_write(d);
    spi_flush();
    CS_HIGH();
}

static void lcd_init(void)
{
    lcd_cmd(0x01);                  /* 软件复位 */
    delay(3000000);

    lcd_cmd(0x11);                  /* 退出睡眠 */
    delay(3000000);

    lcd_cmd(0x3A);                  /* 像素格式 */
    lcd_dat(0x05);                  /* 16bit */

    lcd_cmd(0x36);                  /* MADCTL 扫描方向 */
    lcd_dat(0x00);                  /* 实测唯一正确方向 */

    lcd_cmd(0xB2);
    lcd_dat(0x0C); lcd_dat(0x0C); lcd_dat(0x00);
    lcd_dat(0x33); lcd_dat(0x33);

    lcd_cmd(0xB7); lcd_dat(0x35);

    lcd_cmd(0xBB); lcd_dat(0x19);

    lcd_cmd(0xC0); lcd_dat(0x2C);

    lcd_cmd(0xC2); lcd_dat(0x01);

    lcd_cmd(0xC3); lcd_dat(0x12);

    lcd_cmd(0xC4); lcd_dat(0x20);

    lcd_cmd(0xC6); lcd_dat(0x0F);

    lcd_cmd(0xD0); lcd_dat(0xA4); lcd_dat(0xA1);

    lcd_cmd(0xE0);
    lcd_dat(0xD0); lcd_dat(0x08); lcd_dat(0x11); lcd_dat(0x08);
    lcd_dat(0x0C); lcd_dat(0x15); lcd_dat(0x39); lcd_dat(0x33);
    lcd_dat(0x50); lcd_dat(0x36); lcd_dat(0x13); lcd_dat(0x14);
    lcd_dat(0x29); lcd_dat(0x2D);

    lcd_cmd(0xE1);
    lcd_dat(0xD0); lcd_dat(0x08); lcd_dat(0x10); lcd_dat(0x08);
    lcd_dat(0x06); lcd_dat(0x06); lcd_dat(0x39); lcd_dat(0x44);
    lcd_dat(0x51); lcd_dat(0x0B); lcd_dat(0x16); lcd_dat(0x14);
    lcd_dat(0x2F); lcd_dat(0x31);

    lcd_cmd(0x20);                  /* 关闭反显 */
    lcd_cmd(0x13);
    lcd_cmd(0x29);
    delay(1000000);
}

static void lcd_set_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    lcd_cmd(0x2A);
    lcd_dat(0x00); lcd_dat((uint8_t)(x0 + X_OFF));
    lcd_dat(0x00); lcd_dat((uint8_t)(x1 + X_OFF));

    lcd_cmd(0x2B);
    lcd_dat(0x00); lcd_dat((uint8_t)(y0 + Y_OFF));
    lcd_dat(0x00); lcd_dat((uint8_t)(y1 + Y_OFF));

    lcd_cmd(0x2C);
}

static void lcd_fill_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color)
{
    uint32_t n = (uint32_t)(x1 - x0 + 1) * (uint32_t)(y1 - y0 + 1);

    lcd_set_window(x0, y0, x1, y1);
    DC_HIGH();
    CS_LOW();
    while (n--) {
        spi_write((uint8_t)(color >> 8));
        spi_write((uint8_t)(color & 0xFF));
    }
    spi_flush();
    CS_HIGH();
}

/* 5x7 字库，ASCII 32~90 */
static const uint8_t font5x7[59][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},   /* ' ' */
    {0x00, 0x00, 0x5F, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00},
    {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x50},
    {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00},
    {0x00, 0x41, 0x22, 0x1C, 0x00},
    {0x14, 0x08, 0x3E, 0x08, 0x14},
    {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00},
    {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02},
    {0x3E, 0x51, 0x49, 0x45, 0x3E},   /* '0' */
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},   /* '9' */
    {0x00, 0x36, 0x36, 0x00, 0x00},
    {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x00, 0x08, 0x14, 0x22, 0x41},
    {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x41, 0x22, 0x14, 0x08, 0x00},
    {0x02, 0x01, 0x51, 0x09, 0x06},
    {0x32, 0x49, 0x79, 0x41, 0x3E},
    {0x7E, 0x11, 0x11, 0x11, 0x7E},   /* 'A' */
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x41, 0x49, 0x7A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},   /* 'O' */
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43}    /* 'Z' */
};

/* 12x12 汉字点阵（宋体），索引：0温 1度 2湿 3气 4压 5加 6速 7陀 8螺 9磁 10力
 * 11俯 12仰 13横 14滚 15航 16向 17方 18海 19拔 20相 21对 22合 23运 24行 */
static const uint8_t hz12[25][24] = {
    { 0x00, 0xF8, 0x08, 0x88, 0x04, 0xF8, 0x00, 0x88, 0x00, 0xF8, 0x08, 0x00,
      0x05, 0xFC, 0x01, 0x54, 0x03, 0x54, 0x05, 0x54, 0x09, 0x54, 0x03, 0xFE },  /* 温 */
    { 0x00, 0x20, 0x07, 0xFE, 0x04, 0x88, 0x07, 0xFE, 0x04, 0x88, 0x04, 0xF8,
      0x04, 0x00, 0x05, 0xFC, 0x04, 0x84, 0x04, 0x48, 0x04, 0x30, 0x09, 0xCE },  /* 度 */
    { 0x08, 0x00, 0x05, 0xFC, 0x01, 0x04, 0x01, 0xFC, 0x09, 0x04, 0x05, 0xFC,
      0x00, 0x50, 0x02, 0x52, 0x01, 0x54, 0x04, 0x50, 0x08, 0x50, 0x03, 0xFE },  /* 湿 */
    { 0x02, 0x00, 0x03, 0xFE, 0x04, 0x00, 0x0B, 0xFC, 0x00, 0x00, 0x07, 0xF8,
      0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x0A, 0x00, 0x06, 0x00, 0x02 },  /* 气 */
    { 0x07, 0xFE, 0x04, 0x00, 0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0x05, 0xFC,
      0x04, 0x20, 0x04, 0x28, 0x04, 0x24, 0x04, 0x20, 0x0B, 0xFE, 0x00, 0x00 },  /* 压 */
    { 0x02, 0x00, 0x02, 0x00, 0x02, 0x1E, 0x0F, 0xD2, 0x02, 0x52, 0x02, 0x52,
      0x02, 0x52, 0x02, 0x52, 0x02, 0x52, 0x04, 0x52, 0x04, 0x5E, 0x09, 0x92 },  /* 加 */
    { 0x08, 0x20, 0x05, 0xFE, 0x00, 0x20, 0x01, 0xFC, 0x0D, 0x24, 0x05, 0x24,
      0x05, 0xFC, 0x04, 0x70, 0x04, 0xA8, 0x05, 0x24, 0x04, 0x20, 0x0B, 0xFE },  /* 速 */
    { 0x00, 0x10, 0x0F, 0x08, 0x09, 0x7E, 0x09, 0x42, 0x0A, 0x00, 0x0A, 0x22,
      0x09, 0x24, 0x09, 0x28, 0x09, 0x30, 0x0E, 0x22, 0x08, 0x22, 0x08, 0x1E },  /* 陀 */
    { 0x02, 0x3E, 0x02, 0x2A, 0x02, 0x3E, 0x0F, 0xAA, 0x0A, 0xBE, 0x0A, 0x94,
      0x0F, 0xB8, 0x02, 0x12, 0x02, 0xBE, 0x03, 0x88, 0x0C, 0xAC, 0x00, 0x5A },  /* 螺 */
    { 0x00, 0x44, 0x0E, 0x24, 0x04, 0x08, 0x04, 0xFE, 0x08, 0x44, 0x0E, 0x88,
      0x0A, 0xAA, 0x0A, 0xCC, 0x0A, 0x44, 0x0E, 0xAA, 0x0A, 0xEE, 0x00, 0x22 },  /* 磁 */
    { 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x0F, 0xFC, 0x00, 0x84, 0x00, 0x84,
      0x00, 0x84, 0x01, 0x04, 0x01, 0x04, 0x02, 0x04, 0x04, 0x04, 0x08, 0x38 },  /* 力 */
    { 0x22, 0x00, 0x21, 0x00, 0x5F, 0xE0, 0x50, 0x00, 0xD2, 0x40, 0x54, 0x40,
      0x5D, 0xE0, 0x54, 0x40, 0x55, 0x40, 0x54, 0xC0, 0x54, 0x40, 0x64, 0xC0 },  /* 俯 */
    { 0x12, 0x00, 0x14, 0x00, 0x29, 0xE0, 0x29, 0x20, 0x69, 0x20, 0xA9, 0x20,
      0x29, 0x20, 0x2B, 0x20, 0x2D, 0x20, 0x29, 0x60, 0x21, 0x00, 0x21, 0x00 },  /* 仰 */
    { 0x24, 0x40, 0x2F, 0xE0, 0xF4, 0x40, 0x2F, 0xE0, 0x21, 0x00, 0x7F, 0xE0,
      0x69, 0x20, 0xAF, 0xE0, 0x29, 0x20, 0x2F, 0xE0, 0x24, 0x40, 0x28, 0x20 },  /* 横 */
    { 0x01, 0x00, 0x9F, 0xE0, 0x48, 0x40, 0x12, 0x20, 0x04, 0x80, 0x88, 0x40,
      0x5F, 0xE0, 0x05, 0x20, 0x09, 0x40, 0x58, 0x80, 0xAA, 0x40, 0x0C, 0x20 },  /* 滚 */
    { 0x21, 0x00, 0x78, 0x80, 0x4F, 0xE0, 0x68, 0x00, 0x58, 0x00, 0x4B, 0x80,
      0xFA, 0x80, 0x4A, 0x80, 0x6A, 0x80, 0x5A, 0xA0, 0x4C, 0xA0, 0x98, 0x60 },  /* 航 */
    { 0x08, 0x00, 0x10, 0x00, 0xFF, 0xE0, 0x80, 0x20, 0x80, 0x20, 0x9F, 0x20,
      0x91, 0x20, 0x91, 0x20, 0x91, 0x20, 0x9F, 0x20, 0x80, 0x20, 0x80, 0xE0 },  /* 向 */
    { 0x08, 0x00, 0x04, 0x00, 0xFF, 0xE0, 0x10, 0x00, 0x10, 0x00, 0x1F, 0xC0,
      0x10, 0x40, 0x10, 0x40, 0x20, 0x40, 0x20, 0x40, 0x40, 0x40, 0x83, 0x80 },  /* 方 */
    { 0x88, 0x00, 0x4F, 0xE0, 0x10, 0x00, 0x2F, 0xC0, 0x88, 0x40, 0x4A, 0x40,
      0x3F, 0xE0, 0x10, 0x40, 0x12, 0x40, 0x5F, 0xE0, 0x80, 0x40, 0x01, 0x80 },  /* 海 */
    { 0x22, 0x80, 0x22, 0x40, 0x22, 0x00, 0xF7, 0xE0, 0x22, 0x00, 0x23, 0xE0,
      0x35, 0x20, 0xE5, 0x20, 0x25, 0x40, 0x28, 0x80, 0x29, 0x40, 0x72, 0x20 },  /* 拔 */
    { 0x20, 0x00, 0x27, 0xE0, 0x24, 0x20, 0xF4, 0x20, 0x27, 0xE0, 0x64, 0x20,
      0x74, 0x20, 0xA7, 0xE0, 0x24, 0x20, 0x24, 0x20, 0x27, 0xE0, 0x24, 0x20 },  /* 相 */
    { 0x00, 0x40, 0x00, 0x40, 0xF8, 0x40, 0x0B, 0xE0, 0x08, 0x40, 0x88, 0x40,
      0x52, 0x40, 0x51, 0x40, 0x21, 0x40, 0x50, 0x40, 0x48, 0x40, 0x89, 0xC0 },  /* 对 */
    { 0x04, 0x00, 0x0A, 0x00, 0x11, 0x00, 0x20, 0x80, 0xC0, 0x60, 0x3F, 0x80,
      0x00, 0x00, 0x7F, 0xC0, 0x40, 0x40, 0x40, 0x40, 0x7F, 0xC0, 0x40, 0x40 },  /* 合 */
    { 0x40, 0x00, 0x27, 0xC0, 0x00, 0x00, 0x00, 0x00, 0xEF, 0xE0, 0x21, 0x00,
      0x22, 0x00, 0x24, 0x40, 0x2F, 0xE0, 0x20, 0x20, 0x50, 0x00, 0x8F, 0xE0 },  /* 运 */
    { 0x10, 0x00, 0x27, 0xC0, 0x40, 0x00, 0x90, 0x00, 0x2F, 0xE0, 0x60, 0x80,
      0xA0, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x23, 0x80 }   /* 行 */
};

#define HZ_WEN  0
#define HZ_DU   1
#define HZ_SHI  2
#define HZ_QI   3
#define HZ_YA   4
#define HZ_JIA  5
#define HZ_SU   6
#define HZ_TUO  7
#define HZ_LUO  8
#define HZ_CI   9
#define HZ_LI   10
#define HZ_FU   11
#define HZ_YANG 12
#define HZ_HENG 13
#define HZ_GUN  14
#define HZ_HANG 15
#define HZ_XIANG 16
#define HZ_FANG 17
#define HZ_HAI  18
#define HZ_BA   19
#define HZ_XIANG2 20
#define HZ_DUI  21
#define HZ_HE   22
#define HZ_YUN  23
#define HZ_XING 24

/* 单个 12x12 汉字 */
static void lcd_hz(uint8_t x, uint8_t y, uint8_t idx, uint16_t fg, uint16_t bg)
{
    uint8_t row, col;
    const uint8_t *fp = hz12[idx];

    lcd_set_window(x, y, (uint8_t)(x + 11), (uint8_t)(y + 11));
    DC_HIGH();
    CS_LOW();
    for (row = 0; row < 12; row++) {
        uint16_t bits = ((uint16_t)fp[row * 2] << 8) | (uint16_t)fp[row * 2 + 1];
        for (col = 0; col < 12; col++) {
            uint16_t color = (bits & (1U << (11 - col))) ? fg : bg;
            spi_write((uint8_t)(color >> 8));
            spi_write((uint8_t)(color & 0xFF));
        }
    }
    spi_flush();
    CS_HIGH();
}

/* 两个汉字并排 */
static void lcd_hz2(uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint16_t fg, uint16_t bg)
{
    lcd_hz(x, y, a, fg, bg);
    lcd_hz((uint8_t)(x + 12), y, b, fg, bg);
}

static void lcd_char(uint8_t x, uint8_t y, char ch, uint16_t fg, uint16_t bg)
{
    uint8_t col, row;
    const uint8_t *fp;

    if (ch >= 'a' && ch <= 'z') {
        ch = (char)(ch - 'a' + 'A');
    }
    if (ch < 32 || ch > 90) {
        ch = '?';
    }
    fp = font5x7[(uint8_t)ch - 32];

    lcd_set_window(x, y, (uint8_t)(x + 5), (uint8_t)(y + 7));
    DC_HIGH();
    CS_LOW();
    for (row = 0; row < 8; row++) {
        for (col = 0; col < 6; col++) {
            uint16_t color = bg;
            if (col < 5 && (fp[col] & (1U << row)) != 0) {
                color = fg;
            }
            spi_write((uint8_t)(color >> 8));
            spi_write((uint8_t)(color & 0xFF));
        }
    }
    spi_flush();
    CS_HIGH();
}

static void lcd_str(uint8_t x, uint8_t y, const char *s, uint16_t fg, uint16_t bg)
{
    while (*s) {
        lcd_char(x, y, *s, fg, bg);
        x = (uint8_t)(x + 6);
        s++;
    }
}

/* ================= 数值格式化 ================= */
static char *fmt_int(char *p, int32_t v)
{
    char tmp[12];
    int i = 0;

    if (v < 0) {
        *p++ = '-';
        v = -v;
    }
    if (v == 0) {
        tmp[i++] = '0';
    }
    while (v > 0) {
        tmp[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i > 0) {
        *p++ = tmp[--i];
    }
    *p = '\0';
    return p;
}

static char *fmt_fixed1(char *p, int32_t v)
{
    if (v < 0) {
        *p++ = '-';
        v = -v;
    }
    p = fmt_int(p, v / 10);
    *p++ = '.';
    *p++ = (char)('0' + (v % 10));
    *p = '\0';
    return p;
}

/* ================= 屏幕显示 ================= */
static void lcd_boot_screen(void)
{
    char b[20];

    lcd_fill_rect(0, 0, 127, 127, C_BLACK);
    lcd_str(16, 4, "CANNON V3", C_CYAN, C_BLACK);
    lcd_str(10, 20, "SENSOR ID", C_GRAY, C_BLACK);

    lcd_str(4, 38, "6DS3", C_WHITE, C_BLACK);
    lcd_str(40, 38, (id_lsm6ds3 == 0x69) ? "69 OK" : "ERR", 
            (id_lsm6ds3 == 0x69) ? C_GREEN : C_RED, C_BLACK);

    lcd_str(4, 52, "303A", C_WHITE, C_BLACK);
    lcd_str(40, 52, (id_lsm303a == 0x33) ? "33 OK" : "ERR",
            (id_lsm303a == 0x33) ? C_GREEN : C_RED, C_BLACK);

    lcd_str(4, 66, "303M", C_WHITE, C_BLACK);
    lcd_str(40, 66, (id_lsm303m == 0x40) ? "40 OK" : "ERR",
            (id_lsm303m == 0x40) ? C_GREEN : C_RED, C_BLACK);

    lcd_str(4, 80, "25H", C_WHITE, C_BLACK);
    lcd_str(40, 80, (id_lps25h == 0xBD) ? "BD OK" : "ERR",
            (id_lps25h == 0xBD) ? C_GREEN : C_RED, C_BLACK);

    lcd_str(4, 94, "221", C_WHITE, C_BLACK);
    lcd_str(40, 94, (id_hts221 == 0xBC) ? "BC OK" : "ERR",
            (id_hts221 == 0xBC) ? C_GREEN : C_RED, C_BLACK);

    /* 串口同步打印一遍 */
    uart2_puts("\r\n[Cannon V3] sensor + lcd\r\n");
    uart2_puts("6DS3=0x"); uart2_int(id_lsm6ds3);
    uart2_puts(" 303A=0x"); uart2_int(id_lsm303a);
    uart2_puts(" 303M=0x"); uart2_int(id_lsm303m);
    uart2_puts(" 25H=0x"); uart2_int(id_lps25h);
    uart2_puts(" 221=0x"); uart2_int(id_hts221);
    uart2_puts("\r\n");

    (void)b;
    delay(8000000);
}

/* ---------------- 姿态 / 导航解算（全定点，不依赖数学库） ---------------- */

/* tan(0..45 度) * 10000，用于 atan 查表 */
static const int32_t tan_tab[46] = {
       0,  175,  349,  524,  699,  875, 1051, 1228, 1405, 1584,
    1763, 1944, 2126, 2309, 2493, 2679, 2867, 3057, 3249, 3443,
    3640, 3839, 4040, 4245, 4452, 4663, 4877, 5095, 5317, 5543,
    5774, 6009, 6249, 6494, 6745, 7002, 7265, 7536, 7813, 8098,
    8391, 8693, 9004, 9325, 9657, 10000
};

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

static uint32_t isqrt32(uint32_t v)
{
    uint32_t r = 0, b = 1UL << 30;

    while (b > v) {
        b >>= 2;
    }
    while (b) {
        if (v >= r + b) {
            v -= r + b;
            r = (r >> 1) + b;
        } else {
            r >>= 1;
        }
        b >>= 2;
    }
    return r;
}

/* atan(num/den)，要求 num<=den，返回 0.1 度（0~450） */
static int32_t atan_ratio(int32_t num, int32_t den)
{
    int32_t r, i, lo, hi;

    if (den == 0) {
        return 450;
    }
    if (num <= 0) {
        return 0;
    }
    r = (num * 10000) / den;
    if (r >= 10000) {
        return 450;
    }
    for (i = 0; i < 45; i++) {
        if (r < tan_tab[i + 1]) {
            break;
        }
    }
    lo = tan_tab[i];
    hi = tan_tab[i + 1];
    if (hi == lo) {
        return i * 10;
    }
    return i * 10 + ((r - lo) * 10) / (hi - lo);
}

/* atan2(y,x)，返回 0.1 度，范围 -1800~1800 */
static int32_t atan2_deg10(int32_t y, int32_t x)
{
    int32_t ax = (x < 0) ? -x : x;
    int32_t ay = (y < 0) ? -y : y;
    int32_t ang;

    if (ax == 0 && ay == 0) {
        return 0;
    }
    if (ax >= ay) {
        ang = atan_ratio(ay, ax);
    } else {
        ang = 900 - atan_ratio(ax, ay);
    }
    if (x < 0) {
        ang = 1800 - ang;
    }
    if (y < 0) {
        ang = -ang;
    }
    return ang;
}

/* 气压（0.1 hPa）转海拔（米），查表 + 线性插值 */
static int32_t altitude_m(int32_t hpa10)
{
    int32_t idx, frac;

    if (hpa10 < 5500) {
        hpa10 = 5500;
    }
    if (hpa10 > 11000) {
        hpa10 = 11000;
    }
    idx = (hpa10 - 5500) / 20;
    if (idx >= 275) {
        idx = 274;
    }
    frac = (hpa10 - 5500) % 20;
    return (int32_t)alt_tab[idx] + ((int32_t)(alt_tab[idx + 1] - alt_tab[idx]) * frac) / 20;
}

static int32_t att_pitch10;
static int32_t att_roll10;
static int32_t att_heading10;
static int32_t att_alt_m;
static int32_t att_alt_rel_m;
static int32_t att_alt_base_m;
static int32_t att_acc_mag_mg;
static int32_t att_mag_mag_ut10;
static uint32_t run_tick10;

/* 六轴互补滤波：陀螺积分 + 加速度长期校正 */
static int32_t pitch_filt10;
static int32_t roll_filt10;
static uint8_t att_inited;

static void update_attitude_fast(void)
{
    int32_t acc_pitch, acc_roll;
    int32_t s;

    /* 加速度参考角 */
    s = isqrt32((uint32_t)(acc_mg[1] * acc_mg[1] + acc_mg[2] * acc_mg[2]));
    acc_pitch = atan2_deg10(-acc_mg[0], s);
    acc_roll  = atan2_deg10(acc_mg[1], acc_mg[2]);

    if (!att_inited) {
        pitch_filt10 = acc_pitch;
        roll_filt10  = acc_roll;
        att_inited = 1;
        return;
    }

    /* 陀螺积分：gyro_dps10(0.1dps) x 20ms => 0.1度增量 = gyro/50
     * 再加 2% 的加速度误差修正（时间常数约 1 秒）
     */
    pitch_filt10 += gyro_dps10[0] / 50 + (acc_pitch - pitch_filt10) * 2 / 100;
    roll_filt10  += gyro_dps10[1] / 50 + (acc_roll  - roll_filt10)  * 2 / 100;

    if (pitch_filt10 > 900)  pitch_filt10 = 900;
    if (pitch_filt10 < -900) pitch_filt10 = -900;
    if (roll_filt10 > 900)   roll_filt10 = 900;
    if (roll_filt10 < -900)  roll_filt10 = -900;
}

static void calc_attitude(void)
{
    int32_t s;

    att_pitch10 = pitch_filt10;
    att_roll10  = roll_filt10;

    s = atan2_deg10(-mag_ut10[1], mag_ut10[0]);
    if (s < 0) {
        s += 3600;
    }
    if (s >= 3600) {
        s -= 3600;
    }
    att_heading10 = s;

    att_alt_m = altitude_m(press_hpa10);
    att_alt_rel_m = att_alt_m - att_alt_base_m;

    att_acc_mag_mg = (int32_t)isqrt32((uint32_t)(acc_mg[0] * acc_mg[0]
                                               + acc_mg[1] * acc_mg[1]
                                               + acc_mg[2] * acc_mg[2]));
    s = mag_ut10[0] * mag_ut10[0] + mag_ut10[1] * mag_ut10[1] + mag_ut10[2] * mag_ut10[2];
    att_mag_mag_ut10 = (int32_t)isqrt32((uint32_t)s);
}

static const char *dir_str(int32_t h10)
{
    if (h10 < 225 || h10 >= 3375) {
        return "N";
    }
    if (h10 < 675) {
        return "NE";
    }
    if (h10 < 1125) {
        return "E";
    }
    if (h10 < 1575) {
        return "SE";
    }
    if (h10 < 2025) {
        return "S";
    }
    if (h10 < 2475) {
        return "SW";
    }
    if (h10 < 2925) {
        return "W";
    }
    return "NW";
}

static void lcd_show_page2(void)
{
    char b[24];
    char *p;

    /* 俯仰 */
    lcd_hz2(2, 2, HZ_FU, HZ_YANG, C_WHITE, C_BLACK);
    p = b; p = fmt_fixed1(p, att_pitch10); *p = '\0';
    lcd_str(36, 4, b, C_WHITE, C_BLACK);

    /* 横滚 */
    lcd_hz2(2, 17, HZ_HENG, HZ_GUN, C_CYAN, C_BLACK);
    p = b; p = fmt_fixed1(p, att_roll10); *p = '\0';
    lcd_str(36, 19, b, C_CYAN, C_BLACK);

    /* 航向 + 方位 */
    lcd_hz2(2, 32, HZ_HANG, HZ_XIANG, C_YELLOW, C_BLACK);
    p = b; p = fmt_fixed1(p, att_heading10); *p++ = ' '; *p = '\0';
    lcd_str(36, 34, b, C_YELLOW, C_BLACK);
    lcd_str(92, 34, dir_str(att_heading10), C_YELLOW, C_BLACK);

    /* 海拔 */
    lcd_hz2(2, 47, HZ_HAI, HZ_BA, C_GREEN, C_BLACK);
    p = b; p = fmt_int(p, att_alt_m); *p++ = 'M'; *p = '\0';
    lcd_str(36, 49, b, C_GREEN, C_BLACK);

    /* 相对高度 */
    lcd_hz2(2, 62, HZ_XIANG2, HZ_DUI, C_GREEN, C_BLACK);
    p = b; p = fmt_int(p, att_alt_rel_m); *p++ = 'M'; *p = '\0';
    lcd_str(36, 64, b, C_GREEN, C_BLACK);

    /* 合加速度 */
    lcd_hz2(2, 77, HZ_HE, HZ_JIA, C_MAGENTA, C_BLACK);
    p = b; p = fmt_int(p, att_acc_mag_mg); *p++ = 'M'; *p++ = 'G'; *p = '\0';
    lcd_str(36, 79, b, C_MAGENTA, C_BLACK);

    /* 合磁场 */
    lcd_hz2(2, 92, HZ_HE, HZ_CI, C_BLUE, C_BLACK);
    p = b; p = fmt_fixed1(p, att_mag_mag_ut10); *p++ = 'U'; *p++ = 'T'; *p = '\0';
    lcd_str(36, 94, b, C_BLUE, C_BLACK);

    /* 运行时间 mm:ss */
    lcd_hz2(2, 107, HZ_YUN, HZ_XING, C_GRAY, C_BLACK);
    {
        uint32_t total_s = run_tick10 / 10;
        uint32_t mm = total_s / 60;
        uint32_t ss = total_s % 60;
        p = b;
        if (mm < 10) {
            *p++ = '0';
        }
        p = fmt_int(p, (int32_t)mm);
        *p++ = ':';
        if (ss < 10) {
            *p++ = '0';
        }
        p = fmt_int(p, (int32_t)ss);
        *p = '\0';
        lcd_str(36, 109, b, C_GRAY, C_BLACK);
    }
}

static void lcd_show(void)
{
    char b[24];
    char *p;

    /* 温度 */
    lcd_hz2(2, 2, HZ_WEN, HZ_DU, C_WHITE, C_BLACK);
    p = b; p = fmt_fixed1(p, hts_temp_c10); *p++ = 'C'; *p = '\0';
    lcd_str(30, 4, b, C_WHITE, C_BLACK);

    /* 湿度 */
    lcd_hz2(2, 17, HZ_SHI, HZ_DU, C_CYAN, C_BLACK);
    p = b; p = fmt_fixed1(p, hts_hum_p10); *p++ = '%'; *p = '\0';
    lcd_str(30, 19, b, C_CYAN, C_BLACK);

    /* 气压 */
    lcd_hz2(2, 32, HZ_QI, HZ_YA, C_YELLOW, C_BLACK);
    p = b; p = fmt_fixed1(p, press_hpa10); *p++ = 'H'; *p++ = 'P'; *p++ = 'A'; *p = '\0';
    lcd_str(30, 34, b, C_YELLOW, C_BLACK);

    /* 气压温度 */
    lcd_hz2(2, 47, HZ_YA, HZ_WEN, C_YELLOW, C_BLACK);
    p = b; p = fmt_fixed1(p, press_temp_c10); *p++ = 'C'; *p = '\0';
    lcd_str(30, 49, b, C_YELLOW, C_BLACK);

    /* 加速度 LSM6DS3 */
    lcd_hz2(2, 62, HZ_JIA, HZ_SU, C_GREEN, C_BLACK);
    p = b;
    p = fmt_int(p, acc_mg[0]); *p++ = ' ';
    p = fmt_int(p, acc_mg[1]); *p++ = ' ';
    p = fmt_int(p, acc_mg[2]); *p = '\0';
    lcd_str(30, 64, b, C_GREEN, C_BLACK);

    /* 陀螺仪 */
    lcd_hz2(2, 77, HZ_TUO, HZ_LUO, C_MAGENTA, C_BLACK);
    p = b;
    p = fmt_fixed1(p, gyro_dps10[0]); *p++ = ' ';
    p = fmt_fixed1(p, gyro_dps10[1]); *p++ = ' ';
    p = fmt_fixed1(p, gyro_dps10[2]); *p = '\0';
    lcd_str(30, 79, b, C_MAGENTA, C_BLACK);

    /* 磁力计 */
    lcd_hz2(2, 92, HZ_CI, HZ_LI, C_BLUE, C_BLACK);
    p = b;
    p = fmt_fixed1(p, mag_ut10[0]); *p++ = ' ';
    p = fmt_fixed1(p, mag_ut10[1]); *p++ = ' ';
    p = fmt_fixed1(p, mag_ut10[2]); *p = '\0';
    lcd_str(30, 94, b, C_BLUE, C_BLACK);

    /* 加速度 LSM303AGR（第二个） */
    lcd_hz2(2, 107, HZ_JIA, HZ_SU, C_GRAY, C_BLACK);
    lcd_str(26, 109, "2", C_GRAY, C_BLACK);
    p = b;
    p = fmt_int(p, acc2_mg[0]); *p++ = ' ';
    p = fmt_int(p, acc2_mg[1]); *p++ = ' ';
    p = fmt_int(p, acc2_mg[2]); *p = '\0';
    lcd_str(34, 109, b, C_GRAY, C_BLACK);
}

static void print_line(uint32_t n)
{
    uart2_puts("#");
    uart2_int((int32_t)n);
    uart2_puts(" T=");
    uart2_fixed1(hts_temp_c10);
    uart2_puts("C H=");
    uart2_fixed1(hts_hum_p10);
    uart2_puts("% P=");
    uart2_fixed1(press_hpa10);
    uart2_puts("hPa Pt=");
    uart2_fixed1(press_temp_c10);
    uart2_puts("C\r\n");

    uart2_puts("   ACC=");
    uart2_int(acc_mg[0]); uart2_putc(',');
    uart2_int(acc_mg[1]); uart2_putc(',');
    uart2_int(acc_mg[2]);
    uart2_puts("mg GYRO=");
    uart2_fixed1(gyro_dps10[0]); uart2_putc(',');
    uart2_fixed1(gyro_dps10[1]); uart2_putc(',');
    uart2_fixed1(gyro_dps10[2]);
    uart2_puts("dps\r\n");

    uart2_puts("   A2=");
    uart2_int(acc2_mg[0]); uart2_putc(',');
    uart2_int(acc2_mg[1]); uart2_putc(',');
    uart2_int(acc2_mg[2]);
    uart2_puts("mg MAG=");
    uart2_fixed1(mag_ut10[0]); uart2_putc(',');
    uart2_fixed1(mag_ut10[1]); uart2_putc(',');
    uart2_fixed1(mag_ut10[2]);
    uart2_puts("uT\r\n");
}

int main(void)
{
    uint32_t cnt = 0;
    uint8_t page = 0;
    uint8_t page_cnt = 0;
    uint8_t fast_cnt = 0;

    RCC_AHB1ENR |= (1UL << 0) | (1UL << 1);

    uart2_init();
    i2c1_init();
    spi2_init();

    lcd_init();
    lcd_fill_rect(0, 0, 127, 127, C_BLACK);

    sensors_init();
    delay(2500000);             /* 等传感器完成首次转换 */

    lcd_boot_screen();
    lcd_fill_rect(0, 0, 127, 127, C_BLACK);   /* 清掉自检页，避免与实时数据重叠 */

    /* 先采一轮，记录海拔基准（相对高度的 0 点） */
    read_lsm6ds3();
    read_lsm303a();
    read_lsm303m();
    read_lps25h();
    read_hts221();
    att_alt_base_m = altitude_m(press_hpa10);
    update_attitude_fast();     /* 用加速度初始化融合角 */

    while (1) {
        /* 快环 20ms：只跑六轴，做陀螺积分 + 加速度校正 */
        read_lsm6ds3();
        update_attitude_fast();

        if (++fast_cnt >= 25) {  /* 25 x 20ms = 500ms */
            fast_cnt = 0;

            read_lsm303a();
            read_lsm303m();
            read_lps25h();
            read_hts221();

            calc_attitude();

            if (++page_cnt >= 40) {      /* 40 x 500ms = 20 秒翻页 */
                page_cnt = 0;
                page ^= 1;
                lcd_fill_rect(0, 0, 127, 127, C_BLACK);
            }

            if (page == 0) {
                lcd_show();
            } else {
                lcd_show_page2();
            }

            print_line(cnt++);

            run_tick10 += 5;             /* 每慢环 0.5 秒 */
        }

        delay(120000);           /* 约 20ms */
    }
}
