/*
 * spi.h - SPI 主控统一管理 (硬件 SPI + 软件 NSS)
 * 复制自主工程 include/spi.h (RF 调试用)
 *
 * 三个从机共用一条 SPI 总线 (SCK=PC5 / MOSI=PC6 / MISO=PC7),
 * 片选由 GPIO 手动控制:
 *   GD25Q64 flash : CS=PF0, Mode0, BR=/4 = 6MHz @24MHz
 *   RA-01  SX1278 : NSS=PH6, Mode0, BR=/4 = 6MHz @24MHz
 *   AD5314 DAC    : SYNC=PH7, Mode2, BR=/4 = 6MHz @24MHz
 */
#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>

/* 从机枚举 */
typedef enum {
    SPI_SLAVE_FLASH = 0,   /* GD25Q64, CS = PF0 */
    SPI_SLAVE_RF,          /* RA-01,   NSS = PH6 */
    SPI_SLAVE_DAC          /* AD5314,  SYNC = PH7 */
} spi_slave_t;

/* SPI 初始化: 硬件SPI(软件NSS) + SCK/MOSI/MISO + 三个CS引脚(初始高) */
void spi_init(void);

/* 选中从机: 配置对应 Mode/速率 + 拉低 CS (原子, 关中断) */
void spi_begin(spi_slave_t slave);

/* 释放从机: 拉高 CS */
void spi_end(spi_slave_t slave);

/* 收发一字节 (全双工) */
uint8_t spi_transfer(uint8_t byte);

/* 收发 n 字节; tx/rx 可传 NULL (对应侧忽略) */
void spi_transfer_n(const uint8_t *tx, uint8_t *rx, uint16_t n);

#endif /* __SPI_H */
