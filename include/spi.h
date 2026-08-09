/*
 * spi.h - SPI 主控统一管理 (硬件 SPI + 软件 NSS)
 *
 * 三个从机共用一条 SPI 总线 (SCK=PC5 / MOSI=PC6 / MISO=PC7),
 * 片选由 GPIO 手动控制, 各从机模式/速率不同:
 *   GD25Q64 flash : CS=PF0, Mode0 (上升沿), BR=/4 = 4MHz
 *   RA-01  SX1278 : NSS=PH6, Mode0 (上升沿), BR=/4 = 4MHz
 *   AD5314 DAC    : SYNC=PH7, Mode2 (下降沿), BR=/2 = 8MHz
 *
 * 约定:
 *   - 每次访问从机前必须 spi_begin(slave) 配好模式/速率并拉低 CS,
 *     事务结束 spi_end(slave) 拉高 CS (见各驱动)
 *   - 不同从机采样沿相反, 禁止在 ISR 内切换从机
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
