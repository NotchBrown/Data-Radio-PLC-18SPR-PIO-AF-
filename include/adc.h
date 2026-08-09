/*
 * adc.h - ADC2 10bit 读取接口
 * AI0~AI3 = PF4~PF7 (ADC2)
 *
 * 数据格式: 10bit右对齐到高8位，低8位中的低6位补0
 *   PORT_AI*_H = (adc_value >> 2)       [高8位]
 *   PORT_AI*_L = (adc_value << 6) & 0xFF [低2位左移到高位，低6位=0]
 *   恢复10bit = (PORT_AI*_H << 2) | (PORT_AI*_L >> 6)
 */
#ifndef __ADC_H
#define __ADC_H

#include <stdint.h>

/* ADC 初始化: ADC2, 10bit, AI0~AI3 = PF4~PF7 浮空输入 */
void adc_init(void);

/* 读取所有通道 (4个), 原子操作, 更新PORT_AI*_H/L全局变量 */
void read_adc_all(void);

/* 读取单个通道, 原子操作, 更新对应PORT_AI*_H/L */
void read_adc_ai0(void);
void read_adc_ai1(void);
void read_adc_ai2(void);
void read_adc_ai3(void);

#endif /* __ADC_H */
