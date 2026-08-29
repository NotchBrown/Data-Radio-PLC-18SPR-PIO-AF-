/*
 * uart3.c - UART3 上位机配置接口实现
 *
 * 接收: UART3 RX 中断 (向量21) 内用短小状态机收帧, 完成置 UART3_RX_OK,
 *       主循环 uart3_process() 解析执行并回对称帧 (帧协议见 doc/upperpc.md)
 *
 * 全局变量写法: 一律大写 + UART3_ 前缀; 高频原子量留 Page0;
 *   发射流程任务表 (32任务) 放 __xdata, 不挤占 Page0
 */
#include "uart3.h"
#include "timer.h"
#include "dio.h"
#include "adc.h"
#include "dac.h"
#include "rf.h"
#include "rf_app.h"
#include "uart1.h"
#include "config.h"
#include "dbg.h"
#include <Arduino.h>
#include <stm8s.h>

/* ==================== 常量 ==================== */
#define UART3_BAUD       115200UL
#define UART3_TIMEOUT_MS 5        /* 帧字节间隔超时(ms), 超时回 IDLE */
#define UART3_HEAD_WRITE 0x37     /* 写入帧定位头 */
#define UART3_HEAD_READ  0x36     /* 读取帧定位头 */
#define UART3_MCU_ID_BASE 0x48CD  /* STM8S208 唯一ID基址 (RM0016), 12字节 */

/* 状态快照 (0x2A): 连发的标准读帧数 */
#define UART3_SNAP_N  28

/* 接收状态机 */
#define ST_IDLE  0
#define ST_ADDR  1
#define ST_DATAL 2
#define ST_DATAH 3
#define ST_CRC   4
#define ST_TAIL  5

/* 发射流程任务参数 (地址低2bit) */
#define TX_PARAM_CONTENT   0   /* 发射内容 = 帧内容指示字段 */
#define TX_PARAM_ENA       1   /* 任务使能 bit0 */
#define TX_PARAM_PERIOD_L  2   /* 执行周期 低16bit */
#define TX_PARAM_PERIOD_H  3   /* 执行周期 高16bit */

/* 命令地址 (doc/upperpc.md) */
#define UART3_CMD_SAVE   0x1E  /* 写触发: 保存配置到 EEPROM */
#define UART3_CMD_CLEAR  0x1F  /* 写触发: 恢复出厂(清配置区) */

/* ==================== 全局变量 (Page0) ==================== */
static volatile uint8_t  UART3_STATE;    /* 状态机 */
static volatile uint8_t  UART3_HEAD;     /* 定位头 */
static volatile uint8_t  UART3_ADDR;     /* 地址码 */
static volatile uint8_t  UART3_DL;       /* 数据低字节 */
static volatile uint8_t  UART3_DH;       /* 数据高字节 */
static volatile uint8_t  UART3_CRC;      /* CRC 累积 */
static volatile uint8_t  UART3_READ;     /* 1=读取帧, 0=写入帧 */
static volatile uint8_t  UART3_RX_OK;    /* 完整帧标志 */
static volatile uint8_t  UART3_WRITE_EN; /* 0=只读(拦截写), 1=允许读写 */
static volatile uint16_t UART3_LAST_MS;  /* 最近收字节时刻 (超时用) */
static volatile uint8_t  UART3_SNAP_PENDING; /* 状态快照待发 (0x2A 触发) */

/* RX 回显诊断缓冲 (仅 RF_DEBUG): ISR 存收到的原始字节, 主循环打印,
 * 用于确认 UART3 是否真的收到上位机配置帧 (排查配置无响应) */
#ifdef RF_DEBUG
#define UART3_RX_ECHO_MAX 24
static volatile uint8_t  UART3_RX_ECHO[UART3_RX_ECHO_MAX];
static volatile uint8_t  UART3_RX_ECHO_N;
#endif

/* RF 网络运行配置 (UART3 读写, 存 EEPROM, 供 rf_app 使用; 上电恢复覆盖) */
uint8_t UART3_SELF_ADDR = 0x01;   /* 本机 RF 地址 */
uint8_t UART3_PEER_ADDR = 0x02;   /* 对端 RF 地址 */
uint8_t UART3_RF_MODE   = 0;      /* 收发模式: 固定 0 (统一异步主从, 0x18) */
uint8_t UART3_RF_ROLE   = 0;      /* 主从: 0=从机 1=主机 (0x19) */

/* 频偏校正 (doc/upperpc.md 0x26~0x28): 单位 Hz, 开关默认 0=关 */
int16_t UART3_FE_VALUE  = 0;      /* 校正值 (Hz, 0x27, 存 EEPROM) */
uint8_t UART3_FE_EN     = 0;      /* 校正开关: 0=关(默认) 1=开 (0x28, 存 EEPROM) */
uint8_t UART3_FE_STATUS = 0;      /* 校正状态: 0未 1成功 2失败 3校正中 (0x26) */

/* UART1 485 透传配置 (doc/upperpc.md 0x1A~0x1D, 存 EEPROM) */
uint16_t UART1_BAUD     = 9600;   /* 485 波特率 */
uint8_t  UART1_BUF_MAX  = 64;     /* 485 接收缓冲上限 (1~127) */
uint16_t UART1_TIMEOUT  = 5;      /* 485 组帧超时 (ms) */
uint8_t  UART1_EN       = 0;      /* 485 透传使能: 0=关(默认) 1=开 */

/* 发射流程任务表 (32任务); 由 rf_app 读取做发送调度 */
uint8_t  UART3_TX_CONTENT[32];    /* CI₁: 主站→从站内容指示 */
uint8_t  UART3_TX_CI2[32];        /* CI₂: 要求从站回传内容指示 (0x40~0x5F) */
uint8_t  UART3_TX_ENA[32];
uint16_t UART3_TX_PERIOD_L[32];
uint16_t UART3_TX_PERIOD_H[32];

/* RF 高层参数 (doc/upperpc.md 0x30~0x38; 存 EEPROM, 上电按此配 SX1278) */
uint32_t UART3_RF_FREQ     = CFG_DFLT_FREQ;
uint8_t  UART3_RF_SF       = CFG_DFLT_SF;
uint8_t  UART3_RF_BW       = CFG_DFLT_BW;
uint8_t  UART3_RF_CR       = CFG_DFLT_CR;
uint8_t  UART3_RF_POWER    = CFG_DFLT_POWER;
uint8_t  UART3_RF_PREAMBLE = CFG_DFLT_PREAMBLE;
uint8_t  UART3_RF_SYNCWORD = CFG_DFLT_SYNCWORD;
uint8_t  UART3_RF_LNA      = CFG_DFLT_LNA;

/* ==================== CRC-8 (poly 0x07, init 0x00) ==================== */
static uint8_t uart3_crc8(uint8_t crc, uint8_t byte)
{
    uint8_t i;
    crc ^= byte;
    for (i = 8; i; i--)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
}

/* ==================== 发射流程任务读写 ==================== */
static uint16_t uart3_tx_read(uint8_t addr)
{
    uint8_t task  = (addr >> 2) & 0x1F;
    uint8_t param = addr & 0x03;
    switch (param) {
    case TX_PARAM_CONTENT:  return UART3_TX_CONTENT[task];
    case TX_PARAM_ENA:      return UART3_TX_ENA[task];
    case TX_PARAM_PERIOD_L: return UART3_TX_PERIOD_L[task];
    default:                return UART3_TX_PERIOD_H[task];
    }
}

static uint16_t uart3_tx_write(uint8_t addr, uint16_t val)
{
    uint8_t task  = (addr >> 2) & 0x1F;
    uint8_t param = addr & 0x03;
    switch (param) {
    case TX_PARAM_CONTENT:  UART3_TX_CONTENT[task] = (uint8_t)val; break;
    case TX_PARAM_ENA:      UART3_TX_ENA[task]     = (uint8_t)val; break;
    case TX_PARAM_PERIOD_L: UART3_TX_PERIOD_L[task] = val; break;
    default:                UART3_TX_PERIOD_H[task] = val; break;
    }
    return uart3_tx_read(addr);
}

/* ==================== 地址读 (返回 16bit) ==================== */
static uint16_t uart3_read_addr(uint8_t addr)
{
    if (addr <= 0x02) {   /* MCU ID: 每地址2字节小端序 */
        uint16_t v = *(volatile uint8_t *)(UART3_MCU_ID_BASE + addr * 2);
        v |= (uint16_t)*(volatile uint8_t *)(UART3_MCU_ID_BASE + addr * 2 + 1) << 8;
        return v;
    }
    switch (addr) {
    case 0x03: return rtc_get_ms();
    case 0x04: return rtc_get_s();
    case 0x05: return rtc_get_min();
    case 0x06: return rtc_get_hour();
    case 0x07: return rtc_get_day();
    case 0x08: return rtc_get_mon();
    case 0x09: return rtc_get_year();
    case 0x0A: dio_read_di_low();  return PORT_DI_L;
    case 0x0B: dio_read_di_high(); return PORT_DI_H;
    case 0x0C: return PORT_DO_L;
    case 0x0D: return PORT_DO_H;
    case 0x0E: read_adc_ai0(); return (uint16_t)((PORT_AI0_H << 2) | (PORT_AI0_L >> 6));
    case 0x0F: read_adc_ai1(); return (uint16_t)((PORT_AI1_H << 2) | (PORT_AI1_L >> 6));
    case 0x10: read_adc_ai2(); return (uint16_t)((PORT_AI2_H << 2) | (PORT_AI2_L >> 6));
    case 0x11: read_adc_ai3(); return (uint16_t)((PORT_AI3_H << 2) | (PORT_AI3_L >> 6));
    case 0x12: return (uint16_t)((PORT_DO0_H << 2) | (PORT_DO0_L >> 6));
    case 0x13: return (uint16_t)((PORT_DO1_H << 2) | (PORT_DO1_L >> 6));
    case 0x14: return (uint16_t)((PORT_DO2_H << 2) | (PORT_DO2_L >> 6));
    case 0x15: return (uint16_t)((PORT_DO3_H << 2) | (PORT_DO3_L >> 6));
    case 0x16: return UART3_SELF_ADDR;
    case 0x17: return UART3_PEER_ADDR;
    case 0x18: return 0;                       /* 固定 0 (统一异步主从) */
    case 0x19: return UART3_RF_ROLE;
    /* RF 高层参数 (0x30~0x38) */
    case 0x30: return (uint16_t)(UART3_RF_FREQ & 0xFFFF);            /* 频率低16 */
    case 0x31: return (uint16_t)((UART3_RF_FREQ >> 16) & 0xFFFF);    /* 频率高16 */
    case 0x32: return UART3_RF_SF;
    case 0x33: return UART3_RF_BW;
    case 0x34: return UART3_RF_CR;
    case 0x35: return UART3_RF_POWER;
    case 0x36: return UART3_RF_PREAMBLE;
    case 0x37: return UART3_RF_SYNCWORD;
    case 0x38: return UART3_RF_LNA;
    /* UART1 485 透传配置 (0x1A~0x1D) */
    case 0x1A: return UART1_BAUD;       /* 485 波特率 */
    case 0x1B: return UART1_BUF_MAX;    /* 485 缓冲上限 */
    case 0x1C: return UART1_TIMEOUT;    /* 485 组帧超时 */
    case 0x1D: return UART1_EN;         /* 485 透传使能 */
    case 0x1E: return config_valid() ? 0x0001 : 0x0000;  /* 配置区有效性 */
    case 0x1F: return 0x0000;                            /* 保留 */
    /* 控制指令 (0x20~0x28, 见 doc/upperpc.md) */
    case 0x20: return 0x0000;                            /* RF 发送测试 (写触发) */
    case 0x21: return RF_APP_RX_CNT;                     /* 收到帧计数 */
    case 0x22: return RF_APP_CRC_CNT;                    /* CRC 错计数 */
    case 0x23: return RF_APP_OVERFLOW;                   /* 发送溢出计数 */
    case 0x24: return (uint16_t)(int16_t)RF_LAST_RSSI;   /* 最近 RSSI (有符号 dBm) */
    case 0x25: return (uint16_t)(int16_t)RF_LAST_SNR;    /* 最近 SNR (有符号 dB) */
    case 0x26: return UART3_FE_STATUS;                   /* 校正状态 */
    case 0x27: return (uint16_t)UART3_FE_VALUE;          /* 校正值 (Hz, 有符号) */
    case 0x28: return UART3_FE_EN;                       /* 校正开关 */
    case 0x29: return 0x0000;                            /* 应用 RF 配置 (写触发) */
    case 0x2B: return rf_app_get_link();                 /* 链路状态 (只读) */
    case 0x2C: return rf_app_get_state();                /* RF 状态机 (只读) */
    case 0x2D: return dbg_pos_get();                     /* 卡死位置码 (只读) */
    case 0x2E: return uart1_get_len();                   /* 485 缓冲长度 (只读) */
    }
    if (addr >= 0x40 && addr <= 0x5F)   /* CI₂[32]: 要求从站回传内容指示 */
        return UART3_TX_CI2[addr - 0x40];
    if (addr >= 0x60 && addr <= 0x7F)   /* 射频设置(调试): SX1278 寄存器直写 */
        return rf_read_reg(addr & 0x1F);
    if (addr >= 0x80)                   /* 发射流程设置 */
        return uart3_tx_read(addr);
    return 0;
}

static uint8_t uart3_cmd_save(void);   /* 前向声明 (定义在下方) */

/* ==================== 地址写 (返回设定后值) ==================== */
static uint16_t uart3_write_addr(uint8_t addr, uint16_t val)
{
    switch (addr) {
    case 0x03: rtc_set_ms(val);                return rtc_get_ms();
    case 0x04: rtc_set_s((uint8_t)val);        return rtc_get_s();
    case 0x05: rtc_set_min((uint8_t)val);      return rtc_get_min();
    case 0x06: rtc_set_hour((uint8_t)val);     return rtc_get_hour();
    case 0x07: rtc_set_day((uint8_t)val);      return rtc_get_day();
    case 0x08: rtc_set_mon((uint8_t)val);      return rtc_get_mon();
    case 0x09: rtc_set_year((uint8_t)val);     return rtc_get_year();
    case 0x0C: PORT_DO_L = (uint8_t)val; dio_write_do_low();  return PORT_DO_L;
    case 0x0D: PORT_DO_H = (uint8_t)val; dio_write_do_high(); return PORT_DO_H;
    case 0x12: PORT_DO0_H = (uint8_t)(val >> 2);
               PORT_DO0_L = (uint8_t)((val << 6) & 0xFF); write_dac_ao0();
               return (uint16_t)((PORT_DO0_H << 2) | (PORT_DO0_L >> 6));
    case 0x13: PORT_DO1_H = (uint8_t)(val >> 2);
               PORT_DO1_L = (uint8_t)((val << 6) & 0xFF); write_dac_ao1();
               return (uint16_t)((PORT_DO1_H << 2) | (PORT_DO1_L >> 6));
    case 0x14: PORT_DO2_H = (uint8_t)(val >> 2);
               PORT_DO2_L = (uint8_t)((val << 6) & 0xFF); write_dac_ao2();
               return (uint16_t)((PORT_DO2_H << 2) | (PORT_DO2_L >> 6));
    case 0x15: PORT_DO3_H = (uint8_t)(val >> 2);
               PORT_DO3_L = (uint8_t)((val << 6) & 0xFF); write_dac_ao3();
               return (uint16_t)((PORT_DO3_H << 2) | (PORT_DO3_L >> 6));
    case 0x16: UART3_SELF_ADDR = (uint8_t)val; return UART3_SELF_ADDR;
    case 0x17: UART3_PEER_ADDR = (uint8_t)val; return UART3_PEER_ADDR;
    case 0x18: return 0;                       /* 固定 0 (忽略写入) */
    case 0x19: UART3_RF_ROLE   = (uint8_t)val; return UART3_RF_ROLE;
    /* RF 高层参数 (0x30~0x38) */
    case 0x30: UART3_RF_FREQ = (UART3_RF_FREQ & 0xFFFF0000UL) | (val & 0xFFFF);
               return (uint16_t)(UART3_RF_FREQ & 0xFFFF);
    case 0x31: UART3_RF_FREQ = (UART3_RF_FREQ & 0x0000FFFFUL) | ((uint32_t)val << 16);
               return (uint16_t)((UART3_RF_FREQ >> 16) & 0xFFFF);
    case 0x32: UART3_RF_SF = (uint8_t)val; return UART3_RF_SF;
    case 0x33: UART3_RF_BW = (uint8_t)val; return UART3_RF_BW;
    case 0x34: UART3_RF_CR = (uint8_t)val; return UART3_RF_CR;
    case 0x35: UART3_RF_POWER = (uint8_t)val; return UART3_RF_POWER;
    case 0x36: UART3_RF_PREAMBLE = (uint8_t)val; return UART3_RF_PREAMBLE;
    case 0x37: UART3_RF_SYNCWORD = (uint8_t)val; return UART3_RF_SYNCWORD;
    case 0x38: UART3_RF_LNA = (uint8_t)val; return UART3_RF_LNA;
    /* UART1 485 透传配置 (0x1A~0x1D) */
    case 0x1A: UART1_BAUD     = val;          return UART1_BAUD;
    case 0x1B: UART1_BUF_MAX  = (uint8_t)val; return UART1_BUF_MAX;
    case 0x1C: UART1_TIMEOUT  = val;          return UART1_TIMEOUT;
    case 0x1D: UART1_EN       = (val) ? 1 : 0; return UART1_EN;
    case 0x1E:                                          /* 保存配置 */
        if (val == 0x0001)
            return uart3_cmd_save() ? 0x0001 : 0x0000;
        return 0x0000;
    case 0x1F:                                          /* 恢复出厂 */
        if (val == 0x0001) { config_clear(); return 0x0001; }
        return 0x0000;
    /* 控制指令 (0x20~0x28, 见 doc/upperpc.md) */
    case 0x20:                                          /* RF 发送测试 */
        if (val == 0x0001) rf_app_test_tx();
        return 0x0000;
    case 0x21: if (val == 0) RF_APP_RX_CNT = 0;  return RF_APP_RX_CNT;  /* 清零 */
    case 0x22: if (val == 0) RF_APP_CRC_CNT = 0; return RF_APP_CRC_CNT;
    case 0x23: if (val == 0) RF_APP_OVERFLOW = 0;return RF_APP_OVERFLOW;
    case 0x26:                                          /* 频偏校正触发 */
        if (val == 0x0001) {
            if (!UART3_FE_EN) return 0;   /* 校正开关关闭: 不校正 */
            UART3_FE_STATUS = 3;          /* 校正中 */
            RF_APP_FE_REQUEST = 1;        /* rf_app_run 执行 */
            return 0;
        }
        return UART3_FE_STATUS;
    case 0x27: UART3_FE_VALUE = (int16_t)val; return (uint16_t)UART3_FE_VALUE;
    case 0x28: UART3_FE_EN = (val) ? 1 : 0; return UART3_FE_EN;
    case 0x29:                                          /* 应用 RF 配置: C 类参数立即生效 */
        if (val == 0x0001) rf_apply_config();
        return 0x0000;
    case 0x2A:                                          /* 状态快照: 写0x0001 触发 */
        UART3_SNAP_PENDING = (val == 0x0001);
        return UART3_SNAP_N;                            /* 回复帧数据 = 快照帧数 */
    }
    if (addr >= 0x40 && addr <= 0x5F) {   /* CI₂[32]: 要求从站回传内容指示 */
        UART3_TX_CI2[addr - 0x40] = (uint8_t)val;
        return UART3_TX_CI2[addr - 0x40];
    }
    if (addr >= 0x60 && addr <= 0x7F) {   /* 射频设置(调试): SX1278 寄存器直写 */
        rf_write_reg(addr & 0x1F, (uint8_t)val);
        return rf_read_reg(addr & 0x1F);
    }
    if (addr >= 0x80)
        return uart3_tx_write(addr, val);
    return 0;   /* 只读地址写入返回 0 */
}

/* ==================== 命令: 保存配置到 EEPROM ====================
 * 组快照: 射频白名单读当前值 + 发射任务表;
 * 保存期间关 UART3 RX 中断 (EEPROM 逐字节编程约数百 ms, 上位机应等回复)
 */
static uint8_t uart3_cmd_save(void)
{
    config_table_t cfg;
    uint8_t i, ok;

    /* 射频白名单: 读 SX1278 当前寄存器值 */
    for (i = 0; i < RF_CFG_N; i++)
        cfg.rf_cfg[i] = rf_read_reg(RF_CFG_REGS[i]);

    /* 发射任务表: 当前运行配置 */
    for (i = 0; i < 32; i++) {
        cfg.tx_content[i]  = UART3_TX_CONTENT[i];
        cfg.tx_ci2[i]      = UART3_TX_CI2[i];
        cfg.tx_ena[i]      = UART3_TX_ENA[i];
        cfg.tx_period_l[i] = UART3_TX_PERIOD_L[i];
        cfg.tx_period_h[i] = UART3_TX_PERIOD_H[i];
    }

    /* 本机/对端地址 + 收发模式(固定0) + 主从位 */
    cfg.self_addr = UART3_SELF_ADDR;
    cfg.peer_addr = UART3_PEER_ADDR;
    cfg.rf_mode   = 0;
    cfg.rf_role   = UART3_RF_ROLE;

    /* RF 高层参数 */
    cfg.rf_freq     = UART3_RF_FREQ;
    cfg.rf_sf       = UART3_RF_SF;
    cfg.rf_bw       = UART3_RF_BW;
    cfg.rf_cr       = UART3_RF_CR;
    cfg.rf_power    = UART3_RF_POWER;
    cfg.rf_preamble = UART3_RF_PREAMBLE;
    cfg.rf_syncword = UART3_RF_SYNCWORD;
    cfg.rf_lna      = UART3_RF_LNA;

    /* 频偏校正值 + 校正开关 */
    cfg.fe_value  = UART3_FE_VALUE;
    cfg.fe_enable = UART3_FE_EN;

    /* UART1 485 配置 */
    cfg.uart1_baud    = UART1_BAUD;
    cfg.uart1_buf_max = UART1_BUF_MAX;
    cfg.uart1_timeout = UART1_TIMEOUT;
    cfg.uart1_en      = UART1_EN;

    /* 保存期间关 UART3 RX 中断, 避免收帧被打断 */
    UART3_ITConfig(UART3_IT_RXNE, DISABLE);
    ok = config_save(&cfg);
    UART3_ITConfig(UART3_IT_RXNE, ENABLE);
    return ok;
}

/* 发送一字节 (带超时保护, 防 UART3 异常死循环) */
static void uart3_putc(uint8_t c)
{
    uint16_t guard = 0;
    while (!(UART3->SR & 0x80)) {   /* 等 TXE */
        if (++guard > 2000) return;
    }
    UART3->DR = c;
}

/* ==================== 回复一帧 (对称帧, 数据小端) ==================== */
static void uart3_reply(uint8_t head, uint8_t addr, uint16_t val)
{
    uint8_t crc, tail;

    crc = uart3_crc8(uart3_crc8(uart3_crc8(uart3_crc8(0x00, head), addr),
                                (uint8_t)(val & 0xFF)),
                     (uint8_t)(val >> 8));
    tail = (uint8_t)~head;

    /* 发送 (ISR 不碰 TX, 无需关整个包; 仅保 TXE 轮询) */
    uart3_putc(head);
    uart3_putc(addr);
    uart3_putc((uint8_t)(val & 0xFF));
    uart3_putc((uint8_t)(val >> 8));
    uart3_putc(crc);
    uart3_putc(tail);
}

/* ==================== 状态快照 (0x2A 命令) ====================
 * 写 0x2A 0x0001 -> 回标准写回复帧(数据=快照帧数 UART3_SNAP_N), 随后连发
 * UART3_SNAP_N 个标准读帧覆盖全部"动态状态"地址, 供长期稳定性/死机定位。
 * 全为标准帧, 不破坏 UART3 帧协议。 (宏与标志定义见文件头部) */
static void uart3_send_snapshot(void)
{
    static const uint8_t addrs[UART3_SNAP_N] = {
        0x03,0x04,0x05,0x06,0x07,0x08,0x09,   /* RTC */
        0x0A,0x0B,0x0C,0x0D,                   /* DI/DO */
        0x0E,0x0F,0x10,0x11,                   /* AI */
        0x12,0x13,0x14,0x15,                   /* AO */
        0x21,0x22,0x23,0x24,0x25,              /* RF 统计 */
        0x2B,0x2C,0x2D,0x2E                    /* 内部状态 */
    };
    uint8_t i;
    for (i = 0; i < UART3_SNAP_N; i++)
        uart3_reply(UART3_HEAD_READ, addrs[i], uart3_read_addr(addrs[i]));
}

/* ==================== 初始化 ==================== */
void uart3_init(void)
{
    GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST); /* TX 推挽 */
    GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);      /* RX 浮空 */
    UART3_Init(UART3_BAUD, UART3_WORDLENGTH_8D, UART3_STOPBITS_1,
               UART3_PARITY_NO, UART3_MODE_TXRX_ENABLE);

    /* 关键: UART3_Init 只设 TEN/REN, 不置 UARTEN; 必须 Cmd(ENABLE) 使能 UART */
    UART3_Cmd(ENABLE);

    /* 使能 RXNE 中断 (接收状态机) */
    UART3_ITConfig(UART3_IT_RXNE, ENABLE);

    /* 状态机复位 */
    UART3_STATE = ST_IDLE;
    UART3_RX_OK = 0;
    UART3_WRITE_EN = 1;   /* 默认允许读写, 由模式子程序覆盖 */
}

/* ==================== 使能/禁用 ==================== */
void uart3_enable(uint8_t on)
{
    if (on) {
        UART3_ITConfig(UART3_IT_RXNE, ENABLE);
        UART3_Cmd(ENABLE);
    } else {
        UART3_ITConfig(UART3_IT_RXNE, DISABLE);
        UART3_Cmd(DISABLE);
    }
}

/* ==================== 只读/读写 切换 ==================== */
void uart3_set_write(uint8_t en)
{
    UART3_WRITE_EN = en;
}

/* ==================== 上电上报 MCU ID ====================
 * 按 upperpc.md: MCU ID 在地址 0x00~0x02 (每地址 2 字节小端)
 * 上电主动发 3 帧 (读取帧格式 head=0x36), 供上位机识别设备
 */
void uart3_send_id(void)
{
    uint8_t i;
    for (i = 0; i < 3; i++)
        uart3_reply(UART3_HEAD_READ, i, uart3_read_addr(i));
}

/* ==================== 上电恢复配置 ====================
 * 读 EEPROM 校验通过则回填: 射频白名单(SX1278) + 任务表 + 地址/模式
 * 需在 rf_init() 之后调用 (setup 末尾)
 */
void uart3_config_restore(void)
{
    config_table_t cfg;
    uint8_t i;

    DBG_STR("[D]c_load\r\n");
    if (!config_load(&cfg)) { DBG_STR("[D]c_invalid\r\n"); return; }   /* 无效/未保存: 用固件默认值 */
    DBG_STR("[D]c_ok\r\n");

    /* 射频白名单回填 SX1278 */
    for (i = 0; i < RF_CFG_N; i++)
        rf_write_reg(RF_CFG_REGS[i], cfg.rf_cfg[i]);

    /* 发射任务表 */
    for (i = 0; i < 32; i++) {
        UART3_TX_CONTENT[i]  = cfg.tx_content[i];
        UART3_TX_CI2[i]      = cfg.tx_ci2[i];
        UART3_TX_ENA[i]      = cfg.tx_ena[i];
        UART3_TX_PERIOD_L[i] = cfg.tx_period_l[i];
        UART3_TX_PERIOD_H[i] = cfg.tx_period_h[i];
    }

    /* 本机/对端地址 + 收发模式(固定0) + 主从位 */
    UART3_SELF_ADDR = cfg.self_addr;
    UART3_PEER_ADDR = cfg.peer_addr;
    UART3_RF_MODE   = 0;
    UART3_RF_ROLE   = cfg.rf_role;

    /* RF 高层参数 */
    UART3_RF_FREQ     = cfg.rf_freq;
    UART3_RF_SF       = cfg.rf_sf;
    UART3_RF_BW       = cfg.rf_bw;
    UART3_RF_CR       = cfg.rf_cr;
    UART3_RF_POWER    = cfg.rf_power;
    UART3_RF_PREAMBLE = cfg.rf_preamble;
    UART3_RF_SYNCWORD = cfg.rf_syncword;
    UART3_RF_LNA      = cfg.rf_lna;

    /* 频偏校正值 + 校正开关 */
    UART3_FE_VALUE  = cfg.fe_value;
    UART3_FE_EN     = cfg.fe_enable;

    /* UART1 485 配置 */
    UART1_BAUD     = cfg.uart1_baud;
    UART1_BUF_MAX  = cfg.uart1_buf_max;
    UART1_TIMEOUT  = cfg.uart1_timeout;
    UART1_EN       = cfg.uart1_en;
}

/* 调试: 任意模式打印 UART3 收到的原始字节 (仅 RF_DEBUG)
 * 放主循环(非 process), 这样即使板子在模式3(process 不调用)也能看到是否收到字节 */
#ifdef RF_DEBUG
void uart3_dbg_poll(void)
{
    if (UART3_RX_ECHO_N) {
        uint8_t en, i;
        __critical { en = UART3_RX_ECHO_N; UART3_RX_ECHO_N = 0; }
        DBG_STR("[D]rx:");
        for (i = 0; i < en; i++) { DBG_STR(" "); DBG_HEX8(UART3_RX_ECHO[i]); }
        DBG_NL();
    }
}
#endif

/* ==================== 主循环处理 ==================== */
void uart3_process(void)
{
    uint8_t head, addr, dl, dh;
    uint16_t val;

    /* 帧超时: 状态机非 IDLE 且长时间无字节 -> 复位 */
    if (UART3_STATE != ST_IDLE &&
        (uint16_t)(rtc_get_ms() - UART3_LAST_MS) > UART3_TIMEOUT_MS) {
        __critical { UART3_STATE = ST_IDLE; }
    }

    if (!UART3_RX_OK)
        return;

    /* 原子取出帧内容并清标志 */
    __critical {
        head = UART3_HEAD;
        addr = UART3_ADDR;
        dl   = UART3_DL;
        dh   = UART3_DH;
        UART3_RX_OK = 0;
    }

    val = (uint16_t)((dh << 8) | dl);

    if (UART3_READ || !UART3_WRITE_EN)
        val = uart3_read_addr(addr);   /* 读取帧, 或只读模式下拦截写入 */
    else
        val = uart3_write_addr(addr, val);

    uart3_reply(head, addr, val);

    /* 状态快照: 0x2A 触发后连发快照帧 (在回复帧之后) */
    if (UART3_SNAP_PENDING) {
        UART3_SNAP_PENDING = 0;
        uart3_send_snapshot();
    }
}

/* ==================== UART3 接收中断 (向量21) ====================
 * 短小状态机: 只收帧字节并置标志, 执行/回复交给主循环
 * 带 __interrupt(ITC_IRQ_UART3_RX) 挂到中断向量;
 * 向量槽由 stm8s_it.h 中的 INTERRUPT_HANDLER(UART3_RX_IRQHandler,21) 注册
 */
void UART3_RX_IRQHandler(void) __interrupt(ITC_IRQ_UART3_RX)
{
    uint8_t b = UART3->DR;   /* 读数据并清 RXNE */

    UART3_LAST_MS = rtc_get_ms();

#ifdef RF_DEBUG
    if (UART3_RX_ECHO_N < UART3_RX_ECHO_MAX)
        UART3_RX_ECHO[UART3_RX_ECHO_N++] = b;   /* 存收到的原始字节 */
#endif

    switch (UART3_STATE) {
    case ST_IDLE:
        if (b == UART3_HEAD_WRITE || b == UART3_HEAD_READ) {
            UART3_HEAD  = b;
            UART3_READ  = (b == UART3_HEAD_READ);
            UART3_CRC   = uart3_crc8(0x00, b);
            UART3_STATE = ST_ADDR;
        }
        break;
    case ST_ADDR:
        UART3_ADDR  = b;
        UART3_CRC   = uart3_crc8(UART3_CRC, b);
        UART3_STATE = ST_DATAL;
        break;
    case ST_DATAL:
        UART3_DL    = b;
        UART3_CRC   = uart3_crc8(UART3_CRC, b);
        UART3_STATE = ST_DATAH;
        break;
    case ST_DATAH:
        UART3_DH    = b;
        UART3_CRC   = uart3_crc8(UART3_CRC, b);
        UART3_STATE = ST_CRC;
        break;
    case ST_CRC:
        if (b == UART3_CRC)
            UART3_STATE = ST_TAIL;
        else
            UART3_STATE = ST_IDLE;
        break;
    case ST_TAIL:
        if (b == (uint8_t)~UART3_HEAD)
            UART3_RX_OK = 1;
        UART3_STATE = ST_IDLE;
        break;
    }
}

/* UART3 TX 中断 (向量20): 未使用, 空实现 (仅满足 stm8s_it.h 注册) */
void UART3_TX_IRQHandler(void) __interrupt(ITC_IRQ_UART3_TX)
{
}
