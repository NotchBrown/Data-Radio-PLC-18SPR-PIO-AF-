/*
 * mode3.c - 模式3: DEBUG=0, RUN=1 远程发射
 *   UART3 禁用; 运行 RF 应用层主循环 (4 种收发模式, 见 rf_app.c/doc)
 */
#include "mode3.h"
#include "uart3.h"
#include "rf_app.h"
#include "uart1.h"
#include "dbg.h"

void mode3_run(void)
{
    static uint8_t m3_once = 0;
#ifdef RF_DEBUG
    uart3_enable(1);   /* Debug 构建: 保留 UART3 供 RF_DEBUG 打印诊断 */
#else
    uart3_enable(0);   /* Release: 运行模式彻底禁用 UART3 */
#endif
    if (!m3_once) {               /* 进入模式3 标志(只打一次): 确认模式切换+UART3输出 */
        m3_once = 1;
        DBG_STR("[D]M3 enter\r\n");
    }
    rf_app_run();         /* RF 收发主循环 (含 485 三条件调度) */
}
