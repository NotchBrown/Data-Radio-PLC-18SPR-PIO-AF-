/*
 * rf.h - RA-01 (SX1278) LoRa 驱动接口
 *
 * 硬件: NSS=PH6 (spi 模块管理), RST=PH4, DIO0=PH5 (RF_IRQ)
 *      SPI Mode0 / 6MHz (见 spi.h)
 * 调制参数 (频率/SF/BW/CR/功率) 见 rf.c 顶部宏, 待用户确认
 */
#ifndef __RF_H
#define __RF_H

#include <stdint.h>

/* 初始化: 复位 + 进入 LoRa 模式 + 配置频率/调制参数 */
void rf_init(void);

/* 按 UART3_RF_* (0x30~0x38) 重新配置 SX1278 (频率/SF/BW/CR/功率/前导/同步/LNA)
 * rf_init 后调用; uart3_config_restore 恢复 EEPROM 后也需调用 */
void rf_apply_config(void);

/* 寄存器读写 (供上层/调试使用) */
uint8_t rf_read_reg(uint8_t addr);
void    rf_write_reg(uint8_t addr, uint8_t val);

/* 读芯片版本号 (0x12 = SX1276/78) */
uint8_t rf_read_version(void);

/* 发送: 阻塞直到发送完成 */
void rf_send(const uint8_t *buf, uint8_t len);

/* 接收: 阻塞等一包或超时; 返回 1=收到 (*len 为长度), 0=超时 */
uint8_t rf_receive(uint8_t *buf, uint8_t *len, uint16_t timeout_ms);

/* ---- 非阻塞收发 (TIM4 ISR 轮询驱动) ---- */
/* 单帧最大长度: 容纳最大 485 帧 (地址+帧头+长度+127B 数据 = 130B) */
#define RF_RX_MAX  130
/* 环形接收缓冲帧数: ISR 快速搬入原始帧, 主循环解析 (防溢出) */
#define RF_RX_QUEUE 8

/* SPI 占用计数 (spi_begin/end + 长事务 rf_tx_start/DAC 统一管理;
 * >0 时 TIM4 rf_poll 让路, 防 ISR 打断复用 SPI 总线时序) */
extern volatile uint8_t RF_SPI_BUSY;

/* 非阻塞发送: 提交一帧; 返回 1=忙(拒绝, 上层亮 SYSTEM_LED), 0=已提交 */
uint8_t rf_tx_start(const uint8_t *buf, uint8_t len);

/* TIM4 ISR 轮询 (timer.c 调用): 检测 DIO0, 处理 TxDone/RxDone */
void rf_poll(void);

/* 主循环: 从环形接收缓冲取一帧; 返回 1=取到(*len/rssi/snr 回填), 0=空 */
uint8_t rf_rx_pop(uint8_t *buf, uint8_t *len, int8_t *rssi, int8_t *snr);

/* 频偏校正 (控制指令 0x26~0x28, 单位 Hz; 默认关闭, 仅开关打开才校正):
 * 读 FEI(0x28~0x2A) 估频偏(Hz), 返回 1=有效; rf_set_freq_offset 应用校正 */
uint8_t rf_freq_correct_measure(int32_t *out_hz);
void   rf_set_freq_offset(int16_t offset_hz);   /* 载波 = RF_FREQ_HZ + offset */

/* 发送超时兜底: TxDone 长时间未检测到(500ms)时强制回接收, 防 RF_TX_BUSY 卡死 */
void rf_abort_tx(void);
extern volatile uint16_t RF_TX_START_MS;   /* 发送开始时刻 (TICK_MS) */

/* 收发状态 (ISR/主循环共享, 原子读) */
extern volatile uint8_t RF_TX_BUSY;  /* 发送忙 (rf_tx_start 置位, TxDone 清除) */
extern volatile uint8_t RF_TX_DONE;  /* 发送完成标志 (TxDone 置位) */
extern volatile uint8_t RF_RX_OVF;   /* 环形缓冲溢出标志 (置1 -> 主循环亮 SYS 灯) */
extern volatile int8_t  RF_LAST_RSSI;/* 最近一帧 RSSI (dBm, 主循环解析时更新) */
extern volatile int8_t  RF_LAST_SNR; /* 最近一帧 SNR (dB) */

#endif /* __RF_H */
