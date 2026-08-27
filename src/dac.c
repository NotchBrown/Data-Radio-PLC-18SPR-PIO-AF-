/*
 * dac.c - AD5314 DAC SPI 写入实现
 *
 * 通信协议 (AD5314 数据手册, 16bit 命令):
 *   [A1 A0 | PD | LDAC | D9 D8 D7 D6 D5 D4 D3 D2 D1 D0 | X X]
 *   A1:A0 = 通道 (00=AO0, 01=AO1, 10=AO2, 11=AO3)
 *   PD    = 1 正常操作 (0=掉电, 输出高阻)
 *   LDAC  = 0 写入后立即更新所有 DAC 输出 (本芯片无外部 LDAC 引脚)
 *   D[9:0] = 10bit 数据 (AD5314 只用 10bit, 低 2 位忽略)
 * 数据在 SCLK 下降沿被锁存 -> 由 spi 模块配置 Mode2 (见 spi.h)
 *
 * 数据拆分恢复(高位对齐 _L): (PORT_DO*_H << 2) | (PORT_DO*_L >> 6)
 */
#include "dac.h"
#include "dio.h"
#include "spi.h"
#include "rf.h"

/* 内部辅助: 通过SPI写入单个通道 (一个 SPI 事务) */
static void dac_write_channel(uint8_t ch, uint8_t val_h, uint8_t val_l)
{
    uint16_t cmd;
    uint16_t val;

    /* 恢复10bit值: (val_h << 2) | (val_l >> 6) */
    val = ((uint16_t)val_h << 2) | (val_l >> 6);

    /* 构建16bit命令:
     * [A1 A0 | PD=1 | LDAC=0 | D9..D0 | XX]
     * ch<<14 | 0x2000(PD=1,LDAC=0) | (val & 0x3FF)<<2
     */
    cmd = ((uint16_t)ch << 14) | 0x2000 | ((val & 0x3FF) << 2);

    /* 选从机(配Mode2 + 拉低SYNC) -> 发16bit -> 释放(拉高SYNC)
     * 置 RF_SPI_BUSY: TIM4 RF 轮询让路, 保证 SPI 时序不乱 + 不丢 RF 数据 */
    RF_SPI_BUSY = 1;
    spi_begin(SPI_SLAVE_DAC);
    spi_transfer((cmd >> 8) & 0xFF);
    spi_transfer(cmd & 0xFF);
    spi_end(SPI_SLAVE_DAC);
    RF_SPI_BUSY = 0;
}

/* ==================== 初始化 ==================== */
void dac_init(void)
{
    /* SPI 已由 spi_init() 初始化; DAC 的模式/速率由 spi_begin(SPI_SLAVE_DAC) 配置 */

    /* 初始化全局变量为 0 (复位值) */
    PORT_DO0_H = 0;
    PORT_DO0_L = 0;
    PORT_DO1_H = 0;
    PORT_DO1_L = 0;
    PORT_DO2_H = 0;
    PORT_DO2_L = 0;
    PORT_DO3_H = 0;
    PORT_DO3_L = 0;

    /* 把 0V 实际写入全部 4 路 DAC 输出 */
    write_dac_all();
}

/* ==================== 写入接口 ==================== */

void write_dac_all(void)
{
    dac_write_channel(0, PORT_DO0_H, PORT_DO0_L);
    dac_write_channel(1, PORT_DO1_H, PORT_DO1_L);
    dac_write_channel(2, PORT_DO2_H, PORT_DO2_L);
    dac_write_channel(3, PORT_DO3_H, PORT_DO3_L);
}

void write_dac_ao0(void)
{
    dac_write_channel(0, PORT_DO0_H, PORT_DO0_L);
}

void write_dac_ao1(void)
{
    dac_write_channel(1, PORT_DO1_H, PORT_DO1_L);
}

void write_dac_ao2(void)
{
    dac_write_channel(2, PORT_DO2_H, PORT_DO2_L);
}

void write_dac_ao3(void)
{
    dac_write_channel(3, PORT_DO3_H, PORT_DO3_L);
}
