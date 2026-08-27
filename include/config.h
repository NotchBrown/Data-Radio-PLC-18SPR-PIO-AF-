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
#define CFG_VERSION 0x05

/* 射频可持久化寄存器白名单数量 */
#define RF_CFG_N    14

/* 配置表快照 (与 EEPROM 布局对应, 见 doc/upperpc.md) */
typedef struct {
    uint8_t  rf_cfg[RF_CFG_N];   /* 射频白名单寄存器值 (读自 SX1278) */
    uint8_t  tx_content[32];     /* 发射任务: 内容指示字段 */
    uint8_t  tx_ena[32];         /* 发射任务: 使能 bit0 */
    uint16_t tx_period_l[32];    /* 发射任务: 执行周期低 16bit (小端) */
    uint16_t tx_period_h[32];    /* 发射任务: 执行周期高 16bit (小端) */
    uint8_t  self_addr;          /* 本机 RF 地址 */
    uint8_t  peer_addr;          /* 对端 RF 地址 */
    uint8_t  rf_mode;            /* 收发模式 1..4 (见 doc/upperpc.md) */
    uint8_t  rf_role;            /* 模式3 主从: 0=从机 1=主机 */
    int16_t  fe_value;           /* 频偏校正值 (Hz, ±32767, 地址 0x27) */
    uint8_t  fe_enable;          /* 频偏校正开关: 0=关(默认) 1=开 (地址 0x28) */
    uint16_t uart1_baud;         /* UART1 485 波特率 (地址 0x1A, 数值) */
    uint8_t  uart1_buf_max;      /* UART1 485 接收缓冲上限 (地址 0x1B, 1~127) */
    uint16_t uart1_timeout;      /* UART1 485 组帧超时 ms (地址 0x1C) */
    uint8_t  uart1_en;           /* UART1 485 透传使能 (地址 0x1D): 0=关 1=开 */
} config_table_t;

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
