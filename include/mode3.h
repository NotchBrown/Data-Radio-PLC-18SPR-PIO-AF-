/*
 * mode3.h - 模式3: DEBUG=0, RUN=1 远程发射 (待实现)
 *   预留: 执行序列中的设置, 进行远程发送任务
 */
#ifndef __MODE3_H
#define __MODE3_H

/* 周期性子程序: 主循环 loop() 中 switch 调用 */
void mode3_run(void);

#endif /* __MODE3_H */
