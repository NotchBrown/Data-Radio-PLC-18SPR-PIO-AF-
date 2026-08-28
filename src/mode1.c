/*
 * mode1.c - 模式1: DEBUG=0, RUN=0 只读配置
 *   UART3 允许处理, 但只允许读取, 不允许写入
 *   (写帧被 uart3_set_write(0) 拦截, 回读当前值)
 */
#include "mode1.h"
#include "uart3.h"
#include "rf_app.h"

void mode1_run(void)
{
    uart3_enable(1);      /* 允许 UART3 配置口  */
    uart3_set_write(0);   /* 只读: 禁止写入      */
    uart3_process();      /* 处理上位机帧        */
    rf_app_poll();        /* RF 公共维护(接收/统计) */
}
