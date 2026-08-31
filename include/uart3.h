/*
 * uart3.h - UART3 上位机配置接口 (CH340N, 115200/8N1)
 *
 * 帧协议见 doc/upperpc.md:
 *   帧 = 定位头 + 地址 + 数据(2B小端) + CRC-8 + 定位尾
 *   定位头 0x37 写入 / 0x36 读取; 定位尾 = 定位头按位非
 *   接收用 RX 中断 + 短小状态机, 收完一帧置标志,
 *   主循环调 uart3_process() 解析执行并回对称帧
 */
#ifndef __UART3_H
#define __UART3_H

#include <stdint.h>

/* 初始化: 115200 8N1 + RX 中断 + 状态机复位 */
void uart3_init(void);

/* 上电主动上报 MCU ID: 发 3 帧 (地址 0x00~0x02, 每帧 2 字节小端) */
void uart3_send_id(void);

/* 使能/禁用 UART3 (运行模式禁用配置口) */
void uart3_enable(uint8_t on);

/* 设置是否允许写入: en=0 只读(写帧被拦截并回读当前值), en=1 读写 */
void uart3_set_write(uint8_t en);

/* 主循环调用: 处理完整帧 (解析/执行/回复), 含帧超时复位 */
void uart3_process(void);

/* 调试: 任意模式打印 UART3 收到的原始字节 (仅 RF_DEBUG, 诊断 RX 是否收帧) */
#ifdef RF_DEBUG
void uart3_dbg_poll(void);
#else
#define uart3_dbg_poll() ((void)0)
#endif

/* 上电恢复配置: 读 EEPROM 校验通过则回填射频/任务表/地址/模式 (rf_init 后调用) */
void uart3_config_restore(void);

/* RF 网络运行配置 (UART3 读写, 存 EEPROM, rf_app 使用) */
extern uint8_t UART3_SELF_ADDR;   /* 本机 RF 地址 */
extern uint8_t UART3_PEER_ADDR;   /* 对端 RF 地址 */
extern uint8_t UART3_RF_MODE;     /* 收发模式: 固定 0 (统一异步主从, 0x18) */
extern uint8_t UART3_RF_ROLE;     /* 模式3 主从: 0=从机 1=主机 */
extern uint8_t UART3_RF_RADIO;    /* 射频调制: 0=LoRa 1=FSK (0x2F) */

/* 发射流程任务表 (UART3 0x80~0xFF + 0x40~0x5F 读写; rf_app 做发送调度) */
extern uint8_t  UART3_TX_CONTENT[32];  /* 每任务 CI₁ (主站→从站内容指示) */
extern uint8_t  UART3_TX_CI2[32];      /* 每任务 CI₂ (要求从站回传内容指示, 0x40~0x5F) */
extern uint8_t  UART3_TX_ENA[32];      /* 每任务使能 bit0 */
extern uint16_t UART3_TX_PERIOD_L[32]; /* 每任务周期低16bit(单位=TIM4 6kHz节拍 166.7us) */
extern uint16_t UART3_TX_PERIOD_H[32]; /* 每任务周期高16bit */

/* RF 高层参数 (0x30~0x38, 存 EEPROM; 上电按此配 SX1278, 空速自动算) */
extern uint32_t UART3_RF_FREQ;     /* 载波频率 Hz (0x30/0x31) */
extern uint8_t  UART3_RF_SF;       /* 扩频因子 6~12 (0x32) */
extern uint16_t UART3_RF_BW;     /* 带宽 125/250/500 kHz (0x33, 16bit) */
extern uint8_t  UART3_RF_CR;       /* 编码率 5~8 (0x34) */
extern uint8_t  UART3_RF_POWER;    /* 发射功率 dBm (0x35) */
extern uint8_t  UART3_RF_PREAMBLE; /* 前导码 (0x36) */
extern uint8_t  UART3_RF_SYNCWORD; /* 同步字 (0x37) */
extern uint8_t  UART3_RF_LNA;      /* LNA 增益 (0x38) */

/* 频偏校正 (控制指令 0x26~0x28, 单位 Hz; 默认关闭) */
extern int16_t UART3_FE_VALUE;    /* 校正值 (Hz, 0x27, 存 EEPROM) */
extern uint8_t UART3_FE_EN;       /* 校正开关: 0=关 1=开 (0x28, 存 EEPROM) */
extern uint8_t UART3_FE_STATUS;   /* 校正状态: 0未 1成功 2失败 3校正中 (0x26) */

/* UART1 485 透传配置 (0x1A~0x1D, 存 EEPROM) */
extern uint16_t UART1_BAUD;       /* 485 波特率 (0x1A) */
extern uint8_t  UART1_BUF_MAX;    /* 485 接收缓冲上限 (0x1B, 1~127) */
extern uint16_t UART1_TIMEOUT;    /* 485 组帧超时 ms (0x1C) */
extern uint8_t  UART1_EN;         /* 485 透传使能 (0x1D): 0=关 1=开 */

#endif /* __UART3_H */
