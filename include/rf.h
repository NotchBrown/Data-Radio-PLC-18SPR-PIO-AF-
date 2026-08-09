/*
 * rf.h - RA-01 (SX1278) LoRa 驱动接口
 *
 * 硬件: NSS=PH6 (spi 模块管理), RST=PH4, DIO0=PH5 (RF_IRQ)
 *      SPI Mode0 / 6MHz (见 spi.h)
 * 调制参数 (频率/SF/BW/CR/功率) 见 rf.c 顶部宏, 待用户确认
 */
#ifndef __RF_H
#define __RF_H

#include <stdint.h>

/* 初始化: 复位 + 进入 LoRa 模式 + 配置频率/调制参数 */
void rf_init(void);

/* 寄存器读写 (供上层/调试使用) */
uint8_t rf_read_reg(uint8_t addr);
void    rf_write_reg(uint8_t addr, uint8_t val);

/* 读芯片版本号 (0x12 = SX1276/78) */
uint8_t rf_read_version(void);

/* 发送: 阻塞直到发送完成 */
void rf_send(const uint8_t *buf, uint8_t len);

/* 接收: 阻塞等一包或超时; 返回 1=收到 (*len 为长度), 0=超时 */
uint8_t rf_receive(uint8_t *buf, uint8_t *len, uint16_t timeout_ms);

#endif /* __RF_H */
