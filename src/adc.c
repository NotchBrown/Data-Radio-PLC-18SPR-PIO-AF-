/*
 * adc.c - ADC2 10bit 读取实现
 * AI0~AI3 = PF4~PF7 (ADC通道12~15, 见 STM8S208MBT6FANOUT.csv)
 *
 * 数据拆分:
 *   PORT_AI*_H = (adc_value >> 2)       [高8位]
 *   PORT_AI*_L = (adc_value << 6) & 0xFF [低2位左移到高位，低6位=0]
 */
#include "adc.h"
#include "dio.h"
#include <stm8s.h>

/* EOC 超时保护: 转换未完成最多等这么多轮, 防止死机退不出模式 */
#define ADC2_EOC_TIMEOUT  1000

/* 内部辅助: 读取单个ADC通道并拆分为 _H/_L */
static void adc_read_channel(uint8_t ch, volatile uint8_t *p_h, volatile uint8_t *p_l)
{
    uint16_t guard = 0;

    /* 选择通道: AI0~AI3 = AIN12~AIN15 (PF4~PF7, 见引脚网表), CH 在 CSR bit3:0
     * (STM8S ADC2 布局, 与 sduino stm8s.h 的 ADC2_CSR_CH=0x0F 一致) */
    ADC2->CSR = (uint8_t)((ADC2->CSR & 0xF0) | (ch + 12));

    /* 启动转换: ADC 已在 adc_init 上电, 写 ADON=1 即启动转换
     * 注意: 不能先清 ADON=0 再置 1, 那样只会重新上电而不启动转换
     * (RM0016: 首次写 ADON=1 上电, 之后每次写 1 启动一次转换) */
    ADC2->CR1 |= 0x01;              /* ADON = 1, 开始转换 */

    /* 等待转换完成 (EOC 在 CSR bit7, ADC2 布局) + 超时保护防死机 */
    while (!(ADC2->CSR & 0x80)) {
        if (++guard > ADC2_EOC_TIMEOUT) break;   /* 超时放弃, 不死循环 */
    }

    /* 清 EOC 标志 (下次转换需重新等待) */
    ADC2->CSR &= (uint8_t)~0x80;

    /* 读取结果 (左对齐): DRH=val>>2 即 _H, DRL=(val<<6)&0xFF 即 _L,
     * 天然就是项目的高位对齐格式, 直接取用无需再转换 */
    __critical {
        *p_h = ADC2->DRH;
        *p_l = ADC2->DRL;
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
     * fADC = 24MHz/8 = 3MHz (≤ 4MHz 上限; 16M 时为 2MHz), 寄存器值不变 */
    ADC2->CR1 &= (uint8_t)(~0x70);   /* 清 SPSEL[2:0] */
    ADC2->CR1 |= 0x40;               /* SPSEL = 100: fADC2 = fcpu/8 = 3MHz */

    /* 数据格式: 左对齐 (ALIGN=0, 复位默认, CR2 清零即可)
     * 左对齐时 DRH=val>>2, DRL=(val<<6)&0xFF, 天然就是高位对齐的 _H/_L */
    ADC2->CR2 = 0x00;   /* 左对齐 (默认) */
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
