/*
 * dio.c - DIO 全局变量 + 原子读写实现
 *
 * 端口寄存器地址 (GPIO 基地址 + 偏移):
 *   ODR = 基地址+0, IDR = 基地址+1
 *   PB = 0x5005  (DI7~DI0)   -> IDR 0x5006
 *   PE = 0x5014  (DI15~DI8)  -> IDR 0x5015
 *   PG = 0x501E  (DO7~DO0)   -> ODR 0x501E
 *   PI = 0x5028  (DO15~DO8)  -> ODR 0x5028
 *
 * 原子性: 所有读/写都直接进/出自全局变量, 且用 __critical{...}
 *         包裹(进入 sim 关中断, 退出 rim 恢复), 避免中断打断。
 *         ASM 中按 SDCC 命名引用全局: C 全局在汇编里加下划线前缀。
 */
#include "dio.h"
#include <stm8s.h>

/* ==================== 全局变量 (默认 small 模型 -> Page0) ==================== */

/* 数字量: DO 初始 0xFF = 高电平 = 输出关闭 */
volatile uint8_t PORT_DI_H = 0;
volatile uint8_t PORT_DI_L = 0;
volatile uint8_t PORT_DO_H = 0xFF;
volatile uint8_t PORT_DO_L = 0xFF;

/* 模拟量: 初始 0 */
volatile uint8_t PORT_AI0_H = 0, PORT_AI0_L = 0;
volatile uint8_t PORT_AI1_H = 0, PORT_AI1_L = 0;
volatile uint8_t PORT_AI2_H = 0, PORT_AI2_L = 0;
volatile uint8_t PORT_AI3_H = 0, PORT_AI3_L = 0;
volatile uint8_t PORT_DO0_H = 0, PORT_DO0_L = 0;
volatile uint8_t PORT_DO1_H = 0, PORT_DO1_L = 0;
volatile uint8_t PORT_DO2_H = 0, PORT_DO2_L = 0;
volatile uint8_t PORT_DO3_H = 0, PORT_DO3_L = 0;

/* ---- 初始化 ----------------------------------------------------- */
void dio_init(void)
{
    /* DI7~DI0 = PB: 浮空输入 (DDR=0, CR1=0, CR2=0) */
    GPIOB->DDR = 0x00;
    GPIOB->CR1 = 0x00;
    GPIOB->CR2 = 0x00;

    /* DI15~DI8 = PE: 浮空输入 */
    GPIOE->DDR = 0x00;
    GPIOE->CR1 = 0x00;
    GPIOE->CR2 = 0x00;

    /* DO7~DO0 = PG: 推挽输出, 初始全高(关闭) */
    GPIOG->ODR = 0xFF;
    GPIOG->DDR = 0xFF;
    GPIOG->CR1 = 0xFF;
    GPIOG->CR2 = 0x00;

    /* DO15~DO8 = PI: 推挽输出, 初始全高(关闭) */
    GPIOI->ODR = 0xFF;
    GPIOI->DDR = 0xFF;
    GPIOI->CR1 = 0xFF;
    GPIOI->CR2 = 0x00;
}

/* ==================== 原子 ASM 原语: 读/写直接进/出自全局 ==================== */

/* PB IDR -> PORT_DI_L */
static void dio_read_di_low_asm(void) __naked
{
    __asm
        ld  a, 0x5006
        ld  _PORT_DI_L, a
        ret
    __endasm;
}

/* PE IDR -> PORT_DI_H */
static void dio_read_di_high_asm(void) __naked
{
    __asm
        ld  a, 0x5015
        ld  _PORT_DI_H, a
        ret
    __endasm;
}

/* PORT_DO_L -> PG ODR */
static void dio_write_do_low_asm(void) __naked
{
    __asm
        ld  a, _PORT_DO_L
        ld  0x501E, a
        ret
    __endasm;
}

/* PORT_DO_H -> PI ODR */
static void dio_write_do_high_asm(void) __naked
{
    __asm
        ld  a, _PORT_DO_H
        ld  0x5028, a
        ret
    __endasm;
}

/* ==================== 公开原子接口 ==================== */

void dio_read_di_low(void)
{
    __critical {
        dio_read_di_low_asm();
    }
}

void dio_read_di_high(void)
{
    __critical {
        dio_read_di_high_asm();
    }
}

void dio_write_do_low(void)
{
    __critical {
        dio_write_do_low_asm();
    }
}

void dio_write_do_high(void)
{
    __critical {
        dio_write_do_high_asm();
    }
}

/* 批量读 DI: PE/PB -> PORT_DI_H/L, 一次关中断完成 */
void dio_read_di(void)
{
    __critical {
        dio_read_di_high_asm();
        dio_read_di_low_asm();
    }
}

/* 批量写 DO: PORT_DO_H/L -> PI/PG ODR, 一次关中断完成 */
void dio_write_do(void)
{
    __critical {
        dio_write_do_low_asm();
        dio_write_do_high_asm();
    }
}
