/*
 * config.h - 配置表 EEPROM 持久化接口
 *
 * 将"运行配置快照"(射频白名单 + 发射任务表)存入 STM8S208MB 内部数据 EEPROM,
 * 供 UART3 命令地址 0x1E(保存)/0x1F(恢复出厂) 使用。
 * 存储布局与协议细节见 doc/upperpc.md。
 */
#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>

/* 配置区魔法数 / 版本 (EEPROM 校验用; 布局变更需升版本使旧数据失效) */
#define CFG_MAGIC   0x5A
#define CFG_VERSION 0x08   /* v0.08: 加 FSK 快照(13B)于偏移265, long_range 于偏移15, 布局长度 278 */

/* 射频可持久化寄存器白名单数量 */
#define RF_CFG_N    13

/* FSK 快照索引 (UART3_FSK[] / config.fsk[] 布局; FSK 持久化) */
#define FSK_IDX_RADIO    0   /* 射频调制 0x2F (0=LoRa 1=FSK) */
#define FSK_IDX_BR_H     1   /* BitRate MSB (0x02) */
#define FSK_IDX_BR_L     2   /* BitRate LSB (0x03) */
#define FSK_IDX_FDEV_H   3   /* Fdev MSB (0x04) */
#define FSK_IDX_FDEV_L   4   /* Fdev LSB (0x05) */
#define FSK_IDX_RXCONFIG 5   /* RXCONFIG (0x0D) */
#define FSK_IDX_RXBW     6   /* RXBW (0x12) */
#define FSK_IDX_AFCBW    7   /* AFCBW (0x13) */
#define FSK_IDX_PKT1     8   /* PACKETCONFIG1 (0x30) */
#define FSK_IDX_PKT2     9   /* PACKETCONFIG2 (0x31) */
#define FSK_IDX_PAYLOAD  10  /* PAYLOADLENGTH (0x32) */
#define FSK_IDX_NODE     11  /* NODEADRS (0x33) */
#define FSK_IDX_BCAST    12  /* BROADCASTADRS (0x34) */
#define FSK_CFG_N        13

/* 配置表快照 (与 EEPROM 布局对应, 见 doc/upperpc.md) */
typedef struct {
    uint8_t  rf_cfg[RF_CFG_N];   /* 射频白名单寄存器值 (读自 SX1278) */
    uint8_t  tx_content[32];     /* 发射任务: CI₁ (主站→从站内容指示) */
    uint8_t  tx_ci2[32];         /* 发射任务: CI₂ (要求从站回传内容指示) */
    uint8_t  tx_ena[32];         /* 发射任务: 使能 bit0 */
    uint16_t tx_period_l[32];    /* 发射任务: 执行周期低 16bit (小端) */
    uint16_t tx_period_h[32];    /* 发射任务: 执行周期高 16bit (小端) */
    uint8_t  self_addr;          /* 本机 RF 地址 */
    uint8_t  peer_addr;          /* 对端 RF 地址 */
    uint8_t  rf_mode;            /* 收发模式: 固定 0 (统一异步主从) */
    uint8_t  rf_role;            /* 主从: 0=从机 1=主机 (0x19) */
    /* RF 高层参数 (0x30~0x38, 上电按此配 SX1278, 空速自动算) */
    uint32_t rf_freq;            /* 载波频率 Hz (0x30/0x31) */
    uint8_t  rf_sf;              /* 扩频因子 6~12 (0x32) */
    uint8_t  rf_bw;              /* 带宽 125/250/500 kHz (0x33) */
    uint8_t  rf_cr;              /* 编码率 5~8 = 4/5..4/8 (0x34) */
    uint8_t  rf_power;           /* 发射功率 dBm (0x35) */
    uint8_t  rf_preamble;        /* 前导码长度 (0x36) */
    uint8_t  rf_syncword;        /* 同步字 (0x37) */
    uint8_t  rf_lna;             /* LNA 增益 (0x38) */
    int16_t  fe_value;           /* 频偏校正值 (Hz, ±32767, 地址 0x27) */
    uint8_t  fe_enable;          /* 频偏校正开关 (0x28) */
    uint16_t uart1_baud;         /* UART1 485 波特率 (0x1A) */
    uint8_t  uart1_buf_max;      /* UART1 485 缓冲上限 (0x1B) */
    uint16_t uart1_timeout;      /* UART1 485 组帧超时 ms (0x1C) */
    uint8_t  uart1_en;           /* UART1 485 透传使能 (0x1D) */
    uint8_t  long_range;         /* 长距离模式标志 (EEPROM 偏移15; 1=按完整超帧放大 RF 超时) */
    uint8_t  fsk[FSK_CFG_N];     /* FSK 快照: 调制+物理(BitRate/Fdev/RxBw)+包格式 (FSK 持久化) */
} config_table_t;

/* 固件默认 RF 参数 (config 无效/未保存时用; 见 rf.c) */
#define CFG_DFLT_FREQ   470000000UL
#define CFG_DFLT_SF     7
#define CFG_DFLT_BW     125
#define CFG_DFLT_CR     5
#define CFG_DFLT_POWER  13
#define CFG_DFLT_PREAMBLE 8
#define CFG_DFLT_SYNCWORD 0x12
#define CFG_DFLT_LNA    0x23

/* 射频白名单寄存器号 (供上层组快照用, 定义在 config.c) */
extern const uint8_t RF_CFG_REGS[RF_CFG_N];

/* 保存配置到 EEPROM; 返回 1=成功 0=失败 */
uint8_t config_save(const config_table_t *cfg);

/* 从 EEPROM 读取并校验; 返回 1=有效 0=无效/未保存 */
uint8_t config_load(config_table_t *cfg);

/* 仅校验配置区是否有效 (不读出); 返回 1=有效 0=无效 */
uint8_t config_valid(void);

/* 清除 EEPROM 配置区 (恢复出厂, 擦回 0xFF) */
void    config_clear(void);

#endif /* __CONFIG_H */
