/*
 * mode2.h - 模式2: DEBUG=1, RUN=0 读写配置
 *   UART3 允许读写 (默认调试/配置模式)
 */
#ifndef __MODE2_H
#define __MODE2_H

/* 周期性子程序: 主循环 loop() 中 switch 调用 */
void mode2_run(void);

#endif /* __MODE2_H */
