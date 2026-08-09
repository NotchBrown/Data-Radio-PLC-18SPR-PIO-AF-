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

/* 主循环调用: 处理完整帧 (解析/执行/回复), 含帧超时复位 */
void uart3_process(void);

#endif /* __UART3_H */
