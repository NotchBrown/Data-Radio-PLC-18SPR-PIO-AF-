/*
 * adc.c - ADC2 10bit 读取实现
 * AI0~AI3 = PF4~PF7 (ADC通道4~7)
 *
 * 数据拆分:
 *   PORT_AI*_H = (adc_value >> 2)       [高8位]
 *   PORT_AI*_L = (adc_value << 6) & 0xFF [低2位左移到高位，低6位=0]
 */
#include "adc.h"
#include "dio.h"
#include <stm8s.h>

/* 内部辅助: 读取单个ADC通道并拆分为 _H/_L */
static void adc_read_channel(uint8_t ch, volatile uint8_t *p_h, volatile uint8_t *p_l)
{
    uint16_t val;

    /* 选择通道 (AI0~AI3 = ADC通道4~7) */
    ADC2->CSR = (ADC2->CSR & 0xF0) | (ch + 4);

    /* 启动单次转换: ADON 先清 0 再置 1 (0->1 边沿才触发新转换) */
    ADC2->CR1 &= (uint8_t)(~0x01);  /* ADON = 0 */
    ADC2->CR1 |= 0x01;              /* ADON = 1, 开始转换 */

    /* 等待转换完成 (EOC flag 在 CSR bit 7) */
    while (!(ADC2->CSR & 0x80));  /* bit 7 = EOC */

    /* 读取10bit结果 (存储在DRL:DRH低10位) */
    val = ((uint16_t)ADC2->DRH << 8) | ADC2->DRL;
    val &= 0x3FF;  /* 只保留低10位 */

    /* 拆分: 高8位和低2位(左移到高位) */
    __critical {
        *p_h = (uint8_t)(val >> 2);
        *p_l = (uint8_t)((val << 6) & 0xFF);
    }
}

/* ==================== 初始化 ==================== */
void adc_init(void)
{
    /* AI0~AI3 = PF4~PF7 浮空输入 */
    GPIOF->DDR &= ~0xF0;   /* bit 7~4 输入 */
    GPIOF->CR1 &= ~0xF0;   /* 浮空输入 */
    GPIOF->CR2 &= ~0xF0;

    /* ADC2 配置 */
    /* 时钟分频: SPSEL 位在 CR1 bit6:4, 0x40 = fcpu/8
     * fADC = 16MHz/8 = 2MHz (≤ 4MHz 转换时钟上限) */
    ADC2->CR1 &= (uint8_t)(~0x70);   /* 清 SPSEL[2:0] */
    ADC2->CR1 |= 0x40;               /* SPSEL = 110: fADC2 = fcpu/8 = 3MHz */

    /* 数据格式: 右对齐, 10bit */
    ADC2->CR2 = 0x00;
    ADC2->CR2 |= 0x04;  /* ALIGN = 1 (右对齐) */
    ADC2->CSR = 0x00;   /* 通道选择, 初始为0 */

    /* 上电 ADC (首次置位 ADON 启动一次转换) */
    ADC2->CR1 |= 0x01;  /* ADON = 1 */
}

/* ==================== 读取接口 ==================== */

void read_adc_all(void)
{
    adc_read_channel(0, &PORT_AI0_H, &PORT_AI0_L);
    adc_read_channel(1, &PORT_AI1_H, &PORT_AI1_L);
    adc_read_channel(2, &PORT_AI2_H, &PORT_AI2_L);
    adc_read_channel(3, &PORT_AI3_H, &PORT_AI3_L);
}

void read_adc_ai0(void)
{
    adc_read_channel(0, &PORT_AI0_H, &PORT_AI0_L);
}

void read_adc_ai1(void)
{
    adc_read_channel(1, &PORT_AI1_H, &PORT_AI1_L);
}

void read_adc_ai2(void)
{
    adc_read_channel(2, &PORT_AI2_H, &PORT_AI2_L);
}

void read_adc_ai3(void)
{
    adc_read_channel(3, &PORT_AI3_H, &PORT_AI3_L);
}
