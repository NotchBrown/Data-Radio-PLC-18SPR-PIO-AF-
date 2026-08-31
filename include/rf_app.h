/*
 * rf_app.h - RF 应用层 (帧打包/解包 + 4 模式收发状态机)
 *
 * 依赖: rf.c(TIM4 轮询收原始帧) + uart3.c(发送表/地址/模式) + dio/adc/dac + compress
 * 协议: doc/frame.md (定位头+指示字+地址+内容指示+数据)
 * 模式: doc/upperpc.md 0x18 (1同步发送 2异步接收 3异步收发 4定时发送+异步接收)
 */
#ifndef __RF_APP_H
#define __RF_APP_H

#include <stdint.h>

/* 初始化 (上电/换模式时调用; 复位发送表时刻等) */
void rf_app_init(void);

/* RF 公共维护 (mode1/2/3 都调用): 接收解析 + 发送完成收尾 + 溢出灯 + 频偏校正 */
void rf_app_poll(void);

/* 主循环 (mode3 远程发射模式调用): 公共维护 + 4 模式收发状态机 */
void rf_app_run(void);

/* ISR 解包 (rf.c 的 TIM4 轮询收到完整帧后调用):
 * 解析定位头/地址/内容指示/数据, 更新 PORT_DO_* 与 AO_* 内存, 置"输出已更新" */
void rf_app_rx_isr(const uint8_t *buf, uint8_t len);

/* 任务堆叠计数 (发送忙时又收到新任务; 非0 时 SYSTEM_LED 亮) */
extern volatile uint8_t RF_APP_OVERFLOW;

/* 收发统计 + 频偏校正请求 (控制指令 0x21/0x22/0x26 用) */
extern volatile uint16_t RF_APP_RX_CNT;     /* 收到有效帧计数 */
extern volatile uint16_t RF_APP_TX_CNT;     /* 发送帧计数 (含握手/重传) */
extern volatile uint16_t RF_APP_CRC_CNT;    /* CRC 错帧计数 (rf.c 递增) */
extern volatile uint8_t  RF_APP_FE_REQUEST; /* 频偏校正请求 (uart3 0x26 触发) */

/* 发送测试帧 (控制指令 0x20): 用发送表[0] 内容指示发一帧 */
void rf_app_test_tx(void);

/* 内部状态只读 (快照/诊断 0x2B~0x2C): 通联标志 / 主状态机 */
uint8_t rf_app_get_link(void);
uint8_t rf_app_get_state(void);

/* 复位 RUN 灯闪烁状态 (每次重新进入模式3时调用, 从"先常亮"重新开始) */
void rf_app_run_reset(void);

#endif /* __RF_APP_H */
