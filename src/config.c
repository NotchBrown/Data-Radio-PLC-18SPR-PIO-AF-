/*
 * config.c - 配置表 EEPROM 持久化实现
 *
 * 布局 (见 doc/upperpc.md):
 *   偏移 0    : 魔法数 CFG_MAGIC
 *   偏移 1    : 版本 CFG_VERSION
 *   偏移 2    : 射频白名单 RF_CFG_N 字节
 *   偏移 16   : 发射任务表 128 字节 (content/ena/period_l/period_h 各 32)
 *   偏移 144  : 保留 (原 CRC-8 位置, 布局升级后废弃)
 *   偏移 208~ : 本机/对端地址 + 模式 + 主从 + 频偏校正值/开关
 *   偏移 215~ : UART1 485 波特率/缓冲上限/组帧超时/使能
 *   偏移 221  : CRC-8 (poly 0x07, init 0x00, 覆盖偏移 0~220)
 * 共 222 字节。
 *
 * EEPROM 写入: sduino 库逐字节编程 (每字节约 3.7ms), 全量保存约数百 ms,
 *   调用方 (uart3) 应在保存期间屏蔽 UART3 RX 中断。
 */
#include "config.h"
#include <EEPROM.h>

/* ==================== EEPROM 布局偏移 ==================== */
#define CFG_OFF_MAGIC   0
#define CFG_OFF_VERSION 1
#define CFG_OFF_RF      2          /* 射频白名单 RF_CFG_N 字节 */
#define CFG_OFF_CONTENT 16         /* 发射任务: CONTENT(CI₁)[32] */
#define CFG_OFF_CI2     48         /* 发射任务: CI2[32] (要求从站回传) */
#define CFG_OFF_ENA     80         /* 发射任务: ENA[32] */
#define CFG_OFF_PERIODL 112        /* 发射任务: PERIOD_L[32] (u16 小端) */
#define CFG_OFF_PERIODH 176        /* 发射任务: PERIOD_H[32] (u16 小端) */
#define CFG_OFF_SELF    240        /* 本机 RF 地址 */
#define CFG_OFF_PEER    241        /* 对端 RF 地址 */
#define CFG_OFF_MODE    242        /* 收发模式 (固定 0) */
#define CFG_OFF_ROLE    243        /* 主从: 0=从机 1=主机 */
#define CFG_OFF_FREQ_LO 244        /* 载波频率 低16 (Hz) */
#define CFG_OFF_FREQ_HI 246        /* 载波频率 高16 (Hz) */
#define CFG_OFF_SF      248        /* 扩频因子 */
#define CFG_OFF_BW      249        /* 带宽 */
#define CFG_OFF_CR      250        /* 编码率 */
#define CFG_OFF_POWER   251        /* 发射功率 */
#define CFG_OFF_PREAMBLE 252       /* 前导码 */
#define CFG_OFF_SYNCWORD 253       /* 同步字 */
#define CFG_OFF_LNA     254        /* LNA 增益 */
#define CFG_OFF_FE_LO   255        /* 频偏校正值 低字节 */
#define CFG_OFF_FE_HI   256        /* 频偏校正值 高字节 */
#define CFG_OFF_FE_EN   257        /* 频偏校正开关 */
#define CFG_OFF_U1_BDL  258        /* UART1 485 波特率 低字节 */
#define CFG_OFF_U1_BDH  259        /* UART1 485 波特率 高字节 */
#define CFG_OFF_U1_BMAX 260        /* UART1 485 缓冲上限 */
#define CFG_OFF_U1_TOL  261        /* UART1 485 组帧超时 低字节 */
#define CFG_OFF_U1_TOH  262        /* UART1 485 组帧超时 高字节 */
#define CFG_OFF_U1_EN   263        /* UART1 485 透传使能 */
#define CFG_OFF_CRC     264        /* CRC-8, 覆盖 [0..263] */
#define CFG_LEN         265        /* 配置区总长 */

/* ==================== 射频白名单 ====================
 * 仅这些寄存器会被保存/恢复: LoRa 配置类, 排除只读/状态/FIFO/指针寄存器
 * (0x0D~0x13, 0x19/0x1A, 0x28~0x2A, 0x42 等不保存)
 */
const uint8_t RF_CFG_REGS[RF_CFG_N] = {
    0x06, 0x07, 0x08,   /* FRF MSB/MID/LSB 载波频率      */
    0x09,               /* PA_CONFIG 发射功率            */
    0x0C,               /* LNA 接收增益                  */
    0x1D, 0x1E, 0x1F,   /* MODEM_CONFIG_1/2/3 (BW/CR/SF/CRC/AGC) */
    0x20, 0x21,         /* PREAMBLE_MSB/LSB 前导码长度   */
    0x22,               /* PAYLOAD_LENGTH 净负荷长度     */
    0x24,               /* HOP_PERIOD 跳频周期           */
    0x39                /* SYNCWORD 同步字               */
};

/* ==================== CRC-8 (poly 0x07, init 0x00, 与 UART3 帧一致) ==================== */
static uint8_t config_crc8(uint8_t crc, uint8_t byte)
{
    uint8_t i;
    crc ^= byte;
    for (i = 8; i; i--)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
}

static uint8_t config_compute_crc(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0x00;
    while (len--)
        crc = config_crc8(crc, *buf++);
    return crc;
}

/* ==================== 保存 ==================== */
uint8_t config_save(const config_table_t *cfg)
{
    uint8_t buf[CFG_LEN];
    uint8_t i;

    buf[CFG_OFF_MAGIC]   = CFG_MAGIC;
    buf[CFG_OFF_VERSION] = CFG_VERSION;

    /* 射频白名单 */
    for (i = 0; i < RF_CFG_N; i++)
        buf[CFG_OFF_RF + i] = cfg->rf_cfg[i];

    /* 发射任务表: CONTENT(CI₁)(32)+CI2(32)+ENA(32)+PERIOD_L(64)+PERIOD_H(64), 小端 */
    for (i = 0; i < 32; i++) {
        buf[CFG_OFF_CONTENT + i]        = cfg->tx_content[i];
        buf[CFG_OFF_CI2 + i]            = cfg->tx_ci2[i];
        buf[CFG_OFF_ENA + i]            = cfg->tx_ena[i];
        buf[CFG_OFF_PERIODL + i * 2]     = (uint8_t)(cfg->tx_period_l[i] & 0xFF);
        buf[CFG_OFF_PERIODL + i * 2 + 1] = (uint8_t)(cfg->tx_period_l[i] >> 8);
        buf[CFG_OFF_PERIODH + i * 2]     = (uint8_t)(cfg->tx_period_h[i] & 0xFF);
        buf[CFG_OFF_PERIODH + i * 2 + 1] = (uint8_t)(cfg->tx_period_h[i] >> 8);
    }

    /* 本机/对端地址 + 收发模式 + 主从位 */
    buf[CFG_OFF_SELF] = cfg->self_addr;
    buf[CFG_OFF_PEER] = cfg->peer_addr;
    buf[CFG_OFF_MODE] = cfg->rf_mode;
    buf[CFG_OFF_ROLE] = cfg->rf_role;

    /* RF 高层参数: 频率(32bit 小端) + SF/BW/CR/功率/前导/同步/LNA */
    buf[CFG_OFF_FREQ_LO] = (uint8_t)(cfg->rf_freq & 0xFF);
    buf[CFG_OFF_FREQ_LO + 1] = (uint8_t)((cfg->rf_freq >> 8) & 0xFF);
    buf[CFG_OFF_FREQ_HI] = (uint8_t)((cfg->rf_freq >> 16) & 0xFF);
    buf[CFG_OFF_FREQ_HI + 1] = (uint8_t)((cfg->rf_freq >> 24) & 0xFF);
    buf[CFG_OFF_SF]      = cfg->rf_sf;
    buf[CFG_OFF_BW]      = cfg->rf_bw;
    buf[CFG_OFF_CR]      = cfg->rf_cr;
    buf[CFG_OFF_POWER]   = cfg->rf_power;
    buf[CFG_OFF_PREAMBLE]= cfg->rf_preamble;
    buf[CFG_OFF_SYNCWORD]= cfg->rf_syncword;
    buf[CFG_OFF_LNA]     = cfg->rf_lna;

    /* 频偏校正值(有符号 Hz, 小端) + 校正开关 */
    buf[CFG_OFF_FE_LO] = (uint8_t)(cfg->fe_value & 0xFF);
    buf[CFG_OFF_FE_HI] = (uint8_t)(((uint16_t)cfg->fe_value) >> 8);
    buf[CFG_OFF_FE_EN] = cfg->fe_enable;

    /* UART1 485: 波特率(u16) + 缓冲上限 + 组帧超时(u16) + 使能 */
    buf[CFG_OFF_U1_BDL]  = (uint8_t)(cfg->uart1_baud & 0xFF);
    buf[CFG_OFF_U1_BDH]  = (uint8_t)(cfg->uart1_baud >> 8);
    buf[CFG_OFF_U1_BMAX] = cfg->uart1_buf_max;
    buf[CFG_OFF_U1_TOL]  = (uint8_t)(cfg->uart1_timeout & 0xFF);
    buf[CFG_OFF_U1_TOH]  = (uint8_t)(cfg->uart1_timeout >> 8);
    buf[CFG_OFF_U1_EN]   = cfg->uart1_en;

    /* CRC-8 覆盖 [0..263] */
    buf[CFG_OFF_CRC] = config_compute_crc(buf, CFG_OFF_CRC);

    /* 写入 (内部 unlock/lock; 仅写有变化的字节) */
    eeprom_update_block(0, buf, CFG_LEN);
    return 1;
}

/* ==================== 读取 ==================== */
uint8_t config_load(config_table_t *cfg)
{
    uint8_t buf[CFG_LEN];
    uint8_t i;

    for (i = 0; i < CFG_LEN; i++)
        buf[i] = EEPROM_read(i);

    /* 校验 */
    if (buf[CFG_OFF_MAGIC] != CFG_MAGIC) return 0;
    if (buf[CFG_OFF_VERSION] != CFG_VERSION) return 0;
    if (config_compute_crc(buf, CFG_OFF_CRC) != buf[CFG_OFF_CRC]) return 0;

    /* 解包: 射频白名单 */
    for (i = 0; i < RF_CFG_N; i++)
        cfg->rf_cfg[i] = buf[CFG_OFF_RF + i];

    /* 解包: 发射任务表 (小端) */
    for (i = 0; i < 32; i++) {
        cfg->tx_content[i]  = buf[CFG_OFF_CONTENT + i];
        cfg->tx_ci2[i]      = buf[CFG_OFF_CI2 + i];
        cfg->tx_ena[i]      = buf[CFG_OFF_ENA + i];
        cfg->tx_period_l[i] = (uint16_t)buf[CFG_OFF_PERIODL + i * 2]
                            | ((uint16_t)buf[CFG_OFF_PERIODL + i * 2 + 1] << 8);
        cfg->tx_period_h[i] = (uint16_t)buf[CFG_OFF_PERIODH + i * 2]
                            | ((uint16_t)buf[CFG_OFF_PERIODH + i * 2 + 1] << 8);
    }

    /* 本机/对端地址 + 收发模式 + 主从位 */
    cfg->self_addr = buf[CFG_OFF_SELF];
    cfg->peer_addr = buf[CFG_OFF_PEER];
    cfg->rf_mode   = buf[CFG_OFF_MODE];
    cfg->rf_role   = buf[CFG_OFF_ROLE];

    /* RF 高层参数: 频率(32bit 小端) + SF/BW/CR/功率/前导/同步/LNA */
    cfg->rf_freq = (uint32_t)buf[CFG_OFF_FREQ_LO]
                 | ((uint32_t)buf[CFG_OFF_FREQ_LO + 1] << 8)
                 | ((uint32_t)buf[CFG_OFF_FREQ_HI] << 16)
                 | ((uint32_t)buf[CFG_OFF_FREQ_HI + 1] << 24);
    cfg->rf_sf       = buf[CFG_OFF_SF];
    cfg->rf_bw       = buf[CFG_OFF_BW];
    cfg->rf_cr       = buf[CFG_OFF_CR];
    cfg->rf_power    = buf[CFG_OFF_POWER];
    cfg->rf_preamble = buf[CFG_OFF_PREAMBLE];
    cfg->rf_syncword = buf[CFG_OFF_SYNCWORD];
    cfg->rf_lna      = buf[CFG_OFF_LNA];

    /* 频偏校正值 + 校正开关 (开关: 非1一律当 0=关, 默认不校正) */
    cfg->fe_value  = (int16_t)((uint16_t)buf[CFG_OFF_FE_LO]
                             | ((uint16_t)buf[CFG_OFF_FE_HI] << 8));
    cfg->fe_enable = (buf[CFG_OFF_FE_EN] == 1) ? 1 : 0;

    /* UART1 485: 波特率 + 缓冲上限 + 组帧超时 + 使能 (非1一律当 0=关) */
    cfg->uart1_baud    = (uint16_t)buf[CFG_OFF_U1_BDL]
                       | ((uint16_t)buf[CFG_OFF_U1_BDH] << 8);
    cfg->uart1_buf_max = buf[CFG_OFF_U1_BMAX];
    cfg->uart1_timeout = (uint16_t)buf[CFG_OFF_U1_TOL]
                       | ((uint16_t)buf[CFG_OFF_U1_TOH] << 8);
    cfg->uart1_en      = (buf[CFG_OFF_U1_EN] == 1) ? 1 : 0;
    return 1;
}

/* ==================== 有效性检查 (不构造结构体, 省栈) ==================== */
uint8_t config_valid(void)
{
    uint16_t i;
    uint8_t crc = 0x00;

    if (EEPROM_read(CFG_OFF_MAGIC) != CFG_MAGIC) return 0;
    if (EEPROM_read(CFG_OFF_VERSION) != CFG_VERSION) return 0;
    for (i = 0; i < CFG_OFF_CRC; i++)
        crc = config_crc8(crc, EEPROM_read(i));
    return (crc == EEPROM_read(CFG_OFF_CRC)) ? 1 : 0;
}

/* ==================== 清除 (恢复出厂, 擦回 0xFF) ==================== */
void config_clear(void)
{
    uint8_t buf[CFG_LEN];
    uint8_t i;

    for (i = 0; i < CFG_LEN; i++)
        buf[i] = 0xFF;
    eeprom_update_block(0, buf, CFG_LEN);
}
