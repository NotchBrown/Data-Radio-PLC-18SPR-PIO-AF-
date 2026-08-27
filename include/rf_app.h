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

/* 主循环 (mode3 远程发射模式调用): 4 模式收发状态机 */
void rf_app_run(void);

/* ISR 解包 (rf.c 的 TIM4 轮询收到完整帧后调用):
 * 解析定位头/地址/内容指示/数据, 更新 PORT_DO_* 与 AO_* 内存, 置"输出已更新" */
void rf_app_rx_isr(const uint8_t *buf, uint8_t len);

/* 任务堆叠计数 (发送忙时又收到新任务; 非0 时 SYSTEM_LED 亮) */
extern volatile uint8_t RF_APP_OVERFLOW;

#endif /* __RF_APP_H */
