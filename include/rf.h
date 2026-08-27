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

/* ---- 非阻塞收发 (TIM4 ISR 轮询驱动) ---- */
/* 接收缓冲最大长度 */
#define RF_RX_MAX  64

/* 非阻塞发送: 提交一帧; 返回 1=忙(拒绝, 上层亮 SYSTEM_LED), 0=已提交 */
uint8_t rf_tx_start(const uint8_t *buf, uint8_t len);

/* TIM4 ISR 轮询 (timer.c 调用): 处理 RxDone/TxDone/接收缓冲 */
void rf_poll(void);

/* 收发状态 (ISR/主循环共享, 原子读) */
extern volatile uint8_t RF_TX_BUSY;  /* 发送忙 (rf_tx_start 置位, TxDone 清除) */
extern volatile uint8_t RF_TX_DONE;  /* 发送完成标志 (TxDone 置位) */
extern volatile uint8_t RF_RX_FLAG;  /* 收到一包标志 (读走缓冲后清) */
extern volatile uint8_t RF_RX_LEN;   /* 收到的长度 */
extern volatile uint8_t RF_RX_BUF[RF_RX_MAX]; /* 接收缓冲 */

#endif /* __RF_H */
