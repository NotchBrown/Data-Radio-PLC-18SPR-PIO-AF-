/*
 * mode4.c - 模式4: DEBUG=1, RUN=1 本机直通测试
 *   所有 DI -> DO, 所有 AI -> AO, 不允许串口
 *   DI/DO 均为低有效: 直接拷贝保持"导通=导通"一致
 */
#include "mode4.h"
#include "uart3.h"
#include "dio.h"
#include "adc.h"
#include "dac.h"

void mode4_run(void)
{
    uart3_enable(0);      /* 不允许串口 */

    /* DI -> DO (低有效直接拷贝) */
    dio_read_di();                 /* 采样 DI (PE/PB -> PORT_DI_H/L) */
    PORT_DO_L = PORT_DI_L;         /* DI7~0   -> DO7~0   */
    PORT_DO_H = PORT_DI_H;         /* DI15~8  -> DO15~8  */
    dio_write_do();                /* 同步到硬件 (PI/PG) */

    /* AI -> AO (10bit 拷贝后写 DAC) */
    read_adc_all();                /* 采样 AI0~3 -> PORT_AI*_H/L */
    PORT_DO0_H = PORT_AI0_H; PORT_DO0_L = PORT_AI0_L;
    PORT_DO1_H = PORT_AI1_H; PORT_DO1_L = PORT_AI1_L;
    PORT_DO2_H = PORT_AI2_H; PORT_DO2_L = PORT_AI2_L;
    PORT_DO3_H = PORT_AI3_H; PORT_DO3_L = PORT_AI3_L;
    write_dac_all();               /* 写 AD5314 (AO0~3) */
}
