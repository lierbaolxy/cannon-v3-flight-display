/* Cannon V3 (STM32F401RET6) 最小启动文件
 * 只实现复位向量，不启用任何中断
 */

#include <stdint.h>

extern uint32_t _sidata;   /* .data 在 Flash 中的起始地址 */
extern uint32_t _sdata;    /* .data 在 RAM 中的起始地址 */
extern uint32_t _edata;    /* .data 在 RAM 中的结束地址 */
extern uint32_t _sbss;     /* .bss 起始地址 */
extern uint32_t _ebss;     /* .bss 结束地址 */
extern uint32_t _estack;   /* 栈顶地址（RAM 末尾） */

void Reset_Handler(void);
void Default_Handler(void);
int  main(void);

/* 中断向量表，放在 Flash 最前面 0x08000000 */
__attribute__((section(".isr_vector"), used))
const uint32_t g_pfnVectors[16] = {
    (uint32_t)&_estack,          /* 0x00 初始栈顶 */
    (uint32_t)Reset_Handler,     /* 0x04 复位 */
    (uint32_t)Default_Handler,   /* 0x08 NMI */
    (uint32_t)Default_Handler,   /* 0x0C HardFault */
    (uint32_t)Default_Handler,   /* 0x10 MemManage */
    (uint32_t)Default_Handler,   /* 0x14 BusFault */
    (uint32_t)Default_Handler,   /* 0x18 UsageFault */
    0, 0, 0, 0,                  /* 保留 */
    (uint32_t)Default_Handler,   /* 0x2C SVCall */
    (uint32_t)Default_Handler,   /* 0x30 DebugMonitor */
    0,                           /* 保留 */
    (uint32_t)Default_Handler,   /* 0x38 PendSV */
    (uint32_t)Default_Handler    /* 0x3C SysTick */
};

void Reset_Handler(void)
{
    /* 使能 FPU：CPACR 的 CP10/CP11 置 11（全访问）
       必须在任何浮点指令之前执行，否则第一条 VFP 指令就触发 HardFault */
    *(volatile uint32_t *)0xE000ED88UL |= (0xFUL << 20);
    __asm volatile("dsb");
    __asm volatile("isb");

    /* 拷贝已初始化数据到 RAM */
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }
    /* 清零 .bss */
    for (uint32_t *p = &_sbss; p < &_ebss; ) {
        *p++ = 0;
    }

    main();

    while (1) {
    }
}

/* 未使用的中断统一落在这里 */
void Default_Handler(void)
{
    while (1) {
    }
}
