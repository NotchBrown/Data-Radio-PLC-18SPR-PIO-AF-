/*
 * mode4.h - 模式4: DEBUG=1, RUN=1 本机直通测试
 *   所有 DI -> DO, 所有 AI -> AO, 不允许串口
 */
#ifndef __MODE4_H
#define __MODE4_H

/* 周期性子程序: 主循环 loop() 中 switch 调用 */
void mode4_run(void);

#endif /* __MODE4_H */
