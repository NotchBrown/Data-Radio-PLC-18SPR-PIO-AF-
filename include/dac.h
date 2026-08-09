/*
 * dac.h - AD5314 DAC SPI 接口
 * AO0~AO3 通过 AD5314 (SPI) 输出
 *
 * 通信协议:
 *   16bit命令: [A1 A0 | C1 C0 | D9 D8 D7 D6 D5 D4 D3 D2 D1 D0]
 *   A1:A0 = 通道 (00=AO0, 01=AO1, 10=AO2, 11=AO3)
 *   C1:C0 = 01 (Write to DAC)
 *   D[9:0] = 10bit 数据
 *
 * 数据格式: 与ADC一致
 *   PORT_DO*_H = (value >> 2)       [高8位]
 *   PORT_DO*_L = (value << 6) & 0xFF [低2位左移到高位，低6位=0]
 */
#ifndef __DAC_H
#define __DAC_H

#include <stdint.h>

/* DAC 初始化 (SPI已在main.c初始化) */
void dac_init(void);

/* 写入所有通道 (4个), 原子操作, 用PORT_DO*_H/L的值 */
void write_dac_all(void);

/* 写入单个通道, 原子操作 */
void write_dac_ao0(void);
void write_dac_ao1(void);
void write_dac_ao2(void);
void write_dac_ao3(void);

#endif /* __DAC_H */
