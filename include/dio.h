/*
 * dio.h - 数字/模拟 IO 全局变量 + 原子读写
 * DI : DI7~DI0 = PB, DI15~DI8 = PE  (浮空输入)
 * DO : DO7~DO0 = PG, DO15~DO8 = PI  (推挽输出, 低有效, 初始高=关闭)
 * AI : AI0~AI3 = PF4~PF7 (ADC2 10bit)
 * AO : AO0~AO3 = AD5314 DAC (SPI, 10bit)
 *
 * 约定: 全局变量一律大写, 加 PORT_ 前缀, 全部放 Page0
 *       (SDCC stm8 默认 small 模型即把全局放 Page0 0x0000-0x00FF,
 *        只要 DATA 段总大小 < 256B 就保证在 Page0)
 */
#ifndef __DIO_H
#define __DIO_H

#include <stdint.h>

/* ==================== DIO 全局变量 (Page0) ==================== */

/* 数字量 (每组一个字节) */
extern volatile uint8_t PORT_DI_H;   /* DI15~DI8 (PE) 输入 */
extern volatile uint8_t PORT_DI_L;   /* DI7~DI0  (PB) 输入 */
extern volatile uint8_t PORT_DO_H;   /* DO15~DO8 (PI) 输出 */
extern volatile uint8_t PORT_DO_L;   /* DO7~DO0  (PG) 输出 */

/* 模拟量拆高低字约定 (10bit 值, 高位对齐, 与 adc.h/dac.h 一致):
 *   _H = 高8位(值>>2), _L = 低2位左移到高位, 低6位=0 (值<<6 & 0xFF)
 *   恢复10bit = (_H<<2)|(_L>>6)
 *   帧8bit通路直接发 _H; 帧10bit通路用 (_H<<2)|(_L>>6)
 */
/* 模拟输入 (10bit ADC2) */
extern volatile uint8_t PORT_AI0_H, PORT_AI0_L;
extern volatile uint8_t PORT_AI1_H, PORT_AI1_L;
extern volatile uint8_t PORT_AI2_H, PORT_AI2_L;
extern volatile uint8_t PORT_AI3_H, PORT_AI3_L;

/* 模拟输出 (AD5314 DAC 10bit, 4 通道) */
extern volatile uint8_t PORT_DO0_H, PORT_DO0_L;
extern volatile uint8_t PORT_DO1_H, PORT_DO1_L;
extern volatile uint8_t PORT_DO2_H, PORT_DO2_L;
extern volatile uint8_t PORT_DO3_H, PORT_DO3_L;

/* ==================== 接口 ==================== */

/* 初始化: DI 浮空输入, DO 推挽输出高 */
void dio_init(void);

/* ---- 数字量原子读/写 (__critical 关中断, 直接读/写进全局) ---- */
void dio_read_di_low(void);    /* 原子: PB IDR  -> PORT_DI_L */
void dio_read_di_high(void);   /* 原子: PE IDR  -> PORT_DI_H */
void dio_write_do_low(void);   /* 原子: PORT_DO_L -> PG ODR */
void dio_write_do_high(void);  /* 原子: PORT_DO_H -> PI ODR */
void dio_read_di(void);        /* 原子批量: PE/PB -> PORT_DI_H/L */
void dio_write_do(void);       /* 原子批量: PORT_DO_H/L -> PI/PG */

#endif /* __DIO_H */
