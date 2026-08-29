/*
 * uart1.h - UART1 RS-485 透传驱动
 *
 * 硬件: ISO3082DWR (半双工 RS-485 隔离收发)
 *   PA5 = UART1_TX (D), PA4 = UART1_RX (R), PA3 = UART1_CTRL (DE+RE#)
 *   CTRL 高 = DE 使能(发送), 低 = RE# 使能(接收)
 *
 * 配置 (doc/upperpc.md 0x1A~0x1D, 存 EEPROM):
 *   UART1_BAUD 波特率 / UART1_BUF_MAX 缓冲上限 / UART1_TIMEOUT 组帧超时 / UART1_EN 使能
 */
#ifndef __UART1_H
#define __UART1_H

#include <stdint.h>

/* 485 接收缓冲最大 (上限可配置 1~127, 数组固定最大) */
#define UART1_BUF_SIZE 127

/* 初始化: PA3=CTRL(推挽,低=接收), PA4=RX(浮空), PA5=TX(推挽)
 * 波特率取 UART1_BAUD; 开 RXNE 中断; 初始禁用(由运行模式启用) */
void uart1_init(void);

/* 使能/禁用 UART1 (485 透传: 运行模式3 且 UART1_EN=1 时启用) */
void uart1_enable(uint8_t on);

/* 是否有 485 待发帧 (缓冲有数据且满足组帧: 满 或 距上次收字节超时) */
uint8_t uart1_has_frame(void);
/* 485 缓冲是否满 (三条件条件1: 满则优先发) */
uint8_t uart1_is_full(void);
/* 取走 485 待发数据 (返回长度, 0=无); 清缓冲 (组帧+RF发送由 rf_app 调度) */
uint8_t uart1_take_frame(uint8_t *buf);
/* 上次发 485 时刻 (TICK_MS; 供 rf_app 判断 485 空闲超时) */
extern volatile uint16_t UART1_LAST_TX_MS;

/* 485 下行: 收到 RF 485 帧数据, 经 UART1 发出 (CTRL 方向控制, 半双工) */
void uart1_send(const uint8_t *data, uint8_t len);

/* 当前 485 接收缓冲长度 (快照/诊断用) */
uint8_t uart1_get_len(void);

#endif /* __UART1_H */
