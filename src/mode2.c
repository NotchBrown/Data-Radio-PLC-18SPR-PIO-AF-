/*
 * mode2.c - 模式2: DEBUG=1, RUN=0 读写配置
 *   UART3 允许读写 (默认调试/配置模式)
 */
#include "mode2.h"
#include "uart3.h"

void mode2_run(void)
{
    uart3_enable(1);      /* 允许 UART3 配置口  */
    uart3_set_write(1);   /* 允许读写           */
    uart3_process();      /* 处理上位机帧       */
}
