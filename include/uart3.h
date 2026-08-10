/*
 * uart3.h - UART3 上位机配置接口 (CH340N, 115200/8N1)
 *
 * 帧协议见 doc/upperpc.md:
 *   帧 = 定位头 + 地址 + 数据(2B小端) + CRC-8 + 定位尾
 *   定位头 0x37 写入 / 0x36 读取; 定位尾 = 定位头按位非
 *   接收用 RX 中断 + 短小状态机, 收完一帧置标志,
 *   主循环调 uart3_process() 解析执行并回对称帧
 */
#ifndef __UART3_H
#define __UART3_H

#include <stdint.h>

/* 初始化: 115200 8N1 + RX 中断 + 状态机复位 */
void uart3_init(void);

/* 上电主动上报 MCU ID: 发 3 帧 (地址 0x00~0x02, 每帧 2 字节小端) */
void uart3_send_id(void);

/* 使能/禁用 UART3 (运行模式禁用配置口) */
void uart3_enable(uint8_t on);

/* 设置是否允许写入: en=0 只读(写帧被拦截并回读当前值), en=1 读写 */
void uart3_set_write(uint8_t en);

/* 主循环调用: 处理完整帧 (解析/执行/回复), 含帧超时复位 */
void uart3_process(void);

/* 发射流程调度: 遍历任务表, ENA 且到周期的任务经 rf_send 发射
 * (发射模式下由主循环调用; 周期单位 128us, 见 doc/upperpc.md) */
void uart3_tx_run(void);

#endif /* __UART3_H */
