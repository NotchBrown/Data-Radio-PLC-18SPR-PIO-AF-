/*
 * mode3.c - 模式3: DEBUG=0, RUN=1 远程发射 (待实现)
 *   预留: 执行序列中的设置, 进行远程发送任务
 *   当前仅禁止串口; 远程发送逻辑后续补充
 */
#include "mode3.h"
#include "uart3.h"

void mode3_run(void)
{
    uart3_enable(0);      /* 不允许串口          */
    /* TODO: 远程发送任务 (待实现) */
}
