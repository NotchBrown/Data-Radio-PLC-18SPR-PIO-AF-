/*
 * rf_dbg.h - RA-01 SX1278 调试驱动 (test/rf_debug 独立工程)
 * 通过 UART3 命令行调试裸 RF 链路 (不涉及 frame 协议层)
 */
#ifndef __RF_DBG_H
#define __RF_DBG_H

#include <stdint.h>

/* ==================== SX1278 常用寄存器 (命令 reg/wreg 也可直接输号) ==================== */
#define RF_REG_FIFO          0x00
#define RF_REG_OPMODE        0x01
#define RF_REG_FRF_MSB       0x06
#define RF_REG_FRF_MID       0x07
#define RF_REG_FRF_LSB       0x08
#define RF_REG_PA_CONFIG     0x09
#define RF_REG_LNA           0x0C
#define RF_REG_FIFO_ADDR_PTR 0x0D
#define RF_REG_FIFO_TX_BASE  0x0E
#define RF_REG_FIFO_RX_BASE  0x0F
#define RF_REG_IRQ_FLAGS     0x12
#define RF_REG_RX_NB_BYTES   0x13
#define RF_REG_PKT_SNR       0x19
#define RF_REG_PKT_RSSI      0x1A
#define RF_REG_MODEM_CONFIG_1 0x1D
#define RF_REG_MODEM_CONFIG_2 0x1E
#define RF_REG_MODEM_CONFIG_3 0x26   /* LoRa 模式: 0x26 (0x1F 是 SymbTimeoutLsb!) */
#define RF_REG_SYMB_TIMEOUT_LSB 0x1F /* LoRa 模式: 0x1F (0x1B 是只读 RSSI!) */
#define RF_REG_PREAMBLE_MSB   0x20
#define RF_REG_PREAMBLE_LSB   0x21
#define RF_REG_PAYLOAD_LENGTH 0x22
#define RF_REG_MAX_PAYLOAD_LENGTH 0x23
#define RF_REG_HOP_PERIOD     0x24
#define RF_REG_IRQ_FLAGS_MASK 0x11
#define RF_REG_DETECT_OPTIMIZE 0x31
#define RF_REG_DETECT_THRESHOLD 0x37
#define RF_REG_DIO_MAPPING1   0x40
#define RF_REG_SYNC_WORD      0x39
#define RF_REG_IMAGECAL       0x3B  /* Rx 链校准 (上电后必须做) */
#define RF_REG_FEI_MSB        0x28  /* FrequencyError 高字节 */
#define RF_REG_FEI_MID        0x29  /* 中 */
#define RF_REG_FEI_LSB        0x2A  /* 低 (20位有符号, LSB=61.035Hz) */
#define RF_REG_VERSION        0x42

/* ==================== 接口 ==================== */
/* 复位 SX1278 + LoRa 模式 + 默认参数 (470M/SF7/BW125/CR4/5/显式头/CRC on/17dBm)
 * 返回 1=成功 0=失败(LongRange 未生效, 芯片不在 LoRa 模式) */
uint8_t rf_dbg_init(void);

uint8_t rf_dbg_read_reg(uint8_t addr);
void    rf_dbg_write_reg(uint8_t addr, uint8_t val);

/* 设载波频率 Hz (410~525M) */
void rf_dbg_set_freq(uint32_t hz);

/* 读 FrequencyError (20位有符号, 收到有效包后有效; 返回 Hz, 正=对端比本机高) */
int32_t rf_dbg_read_fe(void);
/* 读当前 FRF 对应频率 Hz */
uint32_t rf_dbg_get_freq(void);
/* 扫频精调: 在 base±range 内找能锁定对端的频率, 返回校正后频率; 失败返0 */
uint32_t rf_dbg_tune(uint32_t base, int32_t range);
/* 设调制: sf=6..12, bw_khz=125/250/500, cr=5..8(4/5..4/8); 显式头+CRC on 保持 */
void rf_dbg_set_modem(uint8_t sf, uint16_t bw_khz, uint8_t cr);
/* 设发射功率 dBm (PA_BOOST, 2~17) */
void rf_dbg_set_power(int8_t dbm);

/* 阻塞发一帧 (len<=255) */
void rf_dbg_tx(const uint8_t *buf, uint8_t len);

/* 阻塞收一包: 返回 1=收到OK 2=收到但CRC错(*len回填, 带rssi/snr) 0=超时 */
uint8_t rf_dbg_rx(uint8_t *buf, uint8_t *len, uint16_t timeout_ms,
                  int8_t *rssi_dbm, int8_t *snr_db);

/* 连续接收控制 (能量检测 scan 用) */
void rf_dbg_rx_start(void);
void rf_dbg_rx_stop(void);
/* 接收期间读包 RSSI (dBm, LoRa 下 0x1A 包 RSSI) */
int8_t rf_dbg_cur_rssi(void);
/* 读 IRQ_FLAGS: RxDone=0x40, CRC错=0x20 */
uint8_t rf_dbg_irq(void);

/* ---- 自动 ACK + 非阻塞接收 (平时一直 RXCONT) ---- */
#define RF_DBG_ACK_BYTE 0xAA      /* ACK 帧内容 */
extern uint8_t RF_DBG_ACK;        /* 1=开 0=关 (收到数据帧自动回 ACK) */
void rf_dbg_set_ack(uint8_t on);
/* 非阻塞接收检查(保持RXCONT): 0=无 1=数据帧 2=ACK帧 3=CRC错 */
uint8_t rf_dbg_rx_check(uint8_t *buf, uint8_t *len, int8_t *rssi, int8_t *snr);

#endif /* __RF_DBG_H */
