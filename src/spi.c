/*
 * spi.c - SPI 主控统一管理实现
 *
 * 从机模式/速率 (24MHz 主频, 见 spi.h):
 *   FLASH: Mode0, BR=/4 -> 6MHz
 *   RF   : Mode0, BR=/4 -> 6MHz
 *   DAC  : Mode2, BR=/2 -> 12MHz
 *
 * STM8S 无总线分频, SPI 外设时钟 = fMASTER = 24MHz;
 * SCK = fMASTER / BR, BR 仅 2 的幂分频 (/2~/256)
 */
#include "spi.h"
#include <Arduino.h>
#include <stm8s.h>

/* SPI->CR1 需重新配置的位: BR(bit5:3=0x38) | CPOL(bit1=0x02) | CPHA(bit0=0x01) */
#define SPI_CFG_MASK  0x3B

/* 各从机 CR1 配置值 (BR + CPOL + CPHA, 不含 MSTR/SPE/LSBFIRST) */
#define SPI_CFG_FLASH  0x08   /* Mode0, BR=/4 -> 6MHz   */
#define SPI_CFG_RF     0x08   /* Mode0, BR=/4 -> 6MHz   */
#define SPI_CFG_DAC    0x02   /* Mode2, BR=/2 -> 12MHz  */

/* ==================== 初始化 ==================== */
void spi_init(void)
{
    /* 软件从机管理: 硬件 NSS 引脚(PE5/DI13)不再影响 SPI */
    SPI->CR2 |= SPI_CR2_SSM | SPI_CR2_SSI;
    SPI->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;   /* 主机并使能 */

    pinMode(SCK,  OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);

    pinMode(PF0, OUTPUT); digitalWrite(PF0, HIGH);  /* GD25Q64 CS */
    pinMode(PH6, OUTPUT); digitalWrite(PH6, HIGH);  /* RA-01 NSS  */
    pinMode(PH7, OUTPUT); digitalWrite(PH7, HIGH);  /* AD5314 SYNC*/
}

/* ==================== 从机选择 ==================== */
void spi_begin(spi_slave_t slave)
{
    uint8_t cfg;

    switch (slave) {
    case SPI_SLAVE_DAC:   cfg = SPI_CFG_DAC;   break;
    case SPI_SLAVE_RF:    cfg = SPI_CFG_RF;    break;
    case SPI_SLAVE_FLASH:
    default:              cfg = SPI_CFG_FLASH; break;
    }

    __critical {
        /* 配 Mode/速率 (保留 MSTR/SPE/LSBFIRST) */
        SPI->CR1 = (SPI->CR1 & ~SPI_CFG_MASK) | cfg;

        /* 拉低对应 CS */
        switch (slave) {
        case SPI_SLAVE_DAC:   GPIOH->ODR &= ~0x80; break;  /* PH7 */
        case SPI_SLAVE_RF:    GPIOH->ODR &= ~0x40; break;  /* PH6 */
        case SPI_SLAVE_FLASH:
        default:              GPIOF->ODR &= ~0x01; break;  /* PF0 */
        }
    }
}

void spi_end(spi_slave_t slave)
{
    switch (slave) {
    case SPI_SLAVE_DAC:   GPIOH->ODR |= 0x80; break;  /* PH7 */
    case SPI_SLAVE_RF:    GPIOH->ODR |= 0x40; break;  /* PH6 */
    case SPI_SLAVE_FLASH:
    default:              GPIOF->ODR |= 0x01; break;  /* PF0 */
    }
}

/* ==================== 收发 ==================== */
uint8_t spi_transfer(uint8_t byte)
{
    while (!(SPI->SR & 0x02));   /* 等 TXE=1 可写 */
    SPI->DR = byte;
    while (!(SPI->SR & 0x01));   /* 等 RXNE=1 有数据 */
    return SPI->DR;
}

void spi_transfer_n(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    uint16_t i;
    for (i = 0; i < n; i++) {
        uint8_t out = tx ? tx[i] : 0xFF;
        while (!(SPI->SR & 0x02));
        SPI->DR = out;
        while (!(SPI->SR & 0x01));
        if (rx) rx[i] = SPI->DR;
    }
}
