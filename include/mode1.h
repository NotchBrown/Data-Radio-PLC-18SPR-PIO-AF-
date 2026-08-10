/*
 * mode1.h - 模式1: DEBUG=0, RUN=0 只读配置
 *   UART3 允许处理, 但只允许读取, 不允许写入
 */
#ifndef __MODE1_H
#define __MODE1_H

/* 周期性子程序: 主循环 loop() 中 switch 调用 */
void mode1_run(void);

#endif /* __MODE1_H */
