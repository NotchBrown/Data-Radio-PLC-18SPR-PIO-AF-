/*
 * config.c - 配置表 EEPROM 持久化实现
 *
 * 布局 (见 doc/upperpc.md):
 *   偏移 0    : 魔法数 CFG_MAGIC
 *   偏移 1    : 版本 CFG_VERSION
 *   偏移 2    : 射频白名单 RF_CFG_N 字节 (13)
 *   偏移 16   : 发射任务表: CONTENT(CI₁)[32]
 *   偏移 48   : 发射任务表: CI2[32]
 *   偏移 80   : 发射任务表: ENA[32]
 *   偏移 112  : 发射任务表: PERIOD_L[32] (u16 小端)
 *   偏移 176  : 发射任务表: PERIOD_H[32] (u16 小端)
 *   偏移 240  : 本机/对端地址+模式+主从+RF参数+频偏+UART1
 *   偏移 264  : CRC-8 (poly 0x07, init 0x00, 覆盖偏移 0~263)
 * 共 265 字节。
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
#define CFG_OFF_BW      249        /* 带宽 (2 字节 16bit 小端, 支持 500kHz) */
#define CFG_OFF_CR      251        /* 编码率 */
#define CFG_OFF_POWER   252        /* 发射功率 */
#define CFG_OFF_PREAMBLE 253       /* 前导码 */
#define CFG_OFF_SYNCWORD 254       /* 同步字 */
#define CFG_OFF_LNA     255        /* LNA 增益 */
#define CFG_OFF_FE_LO   256        /* 频偏校正值 低字节 */
#define CFG_OFF_FE_HI   257        /* 频偏校正值 高字节 */
#define CFG_OFF_FE_EN   258        /* 频偏校正开关 */
#define CFG_OFF_U1_BDL  259        /* UART1 485 波特率 低字节 */
#define CFG_OFF_U1_BDH  260        /* UART1 485 波特率 高字节 */
#define CFG_OFF_U1_BMAX 261        /* UART1 485 缓冲上限 */
#define CFG_OFF_U1_TOL  262        /* UART1 485 组帧超时 低字节 */
#define CFG_OFF_U1_TOH  263        /* UART1 485 组帧超时 高字节 */
#define CFG_OFF_U1_EN   264        /* UART1 485 透传使能 */
#define CFG_OFF_CRC     265        /* CRC-8, 覆盖 [0..264] */
#define CFG_LEN         266        /* 配置区总长 */

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

/* 配置表 EEPROM 偏移 -> 字节值 (无大 buf, 供逐字节保存/CRC) */
static uint8_t cfg_byte_at(const config_table_t *cfg, uint16_t off)
{
    uint8_t i;
    if (off == CFG_OFF_MAGIC) return CFG_MAGIC;
    if (off == CFG_OFF_VERSION) return CFG_VERSION;
    if (off >= CFG_OFF_RF && off < CFG_OFF_RF + RF_CFG_N) return cfg->rf_cfg[off - CFG_OFF_RF];
    if (off >= CFG_OFF_CONTENT && off < CFG_OFF_CONTENT + 32) return cfg->tx_content[off - CFG_OFF_CONTENT];
    if (off >= CFG_OFF_CI2 && off < CFG_OFF_CI2 + 32) return cfg->tx_ci2[off - CFG_OFF_CI2];
    if (off >= CFG_OFF_ENA && off < CFG_OFF_ENA + 32) return cfg->tx_ena[off - CFG_OFF_ENA];
    if (off >= CFG_OFF_PERIODL && off < CFG_OFF_PERIODL + 64) {
        i = (uint8_t)((off - CFG_OFF_PERIODL) >> 1);
        return ((off - CFG_OFF_PERIODL) & 1) ? (uint8_t)(cfg->tx_period_l[i] >> 8)
                                             : (uint8_t)(cfg->tx_period_l[i] & 0xFF);
    }
    if (off >= CFG_OFF_PERIODH && off < CFG_OFF_PERIODH + 64) {
        i = (uint8_t)((off - CFG_OFF_PERIODH) >> 1);
        return ((off - CFG_OFF_PERIODH) & 1) ? (uint8_t)(cfg->tx_period_h[i] >> 8)
                                             : (uint8_t)(cfg->tx_period_h[i] & 0xFF);
    }
    if (off == CFG_OFF_SELF) return cfg->self_addr;
    if (off == CFG_OFF_PEER) return cfg->peer_addr;
    if (off == CFG_OFF_MODE) return cfg->rf_mode;
    if (off == CFG_OFF_ROLE) return cfg->rf_role;
    if (off == CFG_OFF_FREQ_LO) return (uint8_t)(cfg->rf_freq & 0xFF);
    if (off == CFG_OFF_FREQ_LO + 1) return (uint8_t)((cfg->rf_freq >> 8) & 0xFF);
    if (off == CFG_OFF_FREQ_HI) return (uint8_t)((cfg->rf_freq >> 16) & 0xFF);
    if (off == CFG_OFF_FREQ_HI + 1) return (uint8_t)((cfg->rf_freq >> 24) & 0xFF);
    if (off == CFG_OFF_SF) return cfg->rf_sf;
    if (off == CFG_OFF_BW) return (uint8_t)(cfg->rf_bw & 0xFF);
    if (off == CFG_OFF_BW + 1) return (uint8_t)(cfg->rf_bw >> 8);
    if (off == CFG_OFF_CR) return cfg->rf_cr;
    if (off == CFG_OFF_POWER) return cfg->rf_power;
    if (off == CFG_OFF_PREAMBLE) return cfg->rf_preamble;
    if (off == CFG_OFF_SYNCWORD) return cfg->rf_syncword;
    if (off == CFG_OFF_LNA) return cfg->rf_lna;
    if (off == CFG_OFF_FE_LO) return (uint8_t)(cfg->fe_value & 0xFF);
    if (off == CFG_OFF_FE_HI) return (uint8_t)(((uint16_t)cfg->fe_value) >> 8);
    if (off == CFG_OFF_FE_EN) return cfg->fe_enable;
    if (off == CFG_OFF_U1_BDL) return (uint8_t)(cfg->uart1_baud & 0xFF);
    if (off == CFG_OFF_U1_BDH) return (uint8_t)(cfg->uart1_baud >> 8);
    if (off == CFG_OFF_U1_BMAX) return cfg->uart1_buf_max;
    if (off == CFG_OFF_U1_TOL) return (uint8_t)(cfg->uart1_timeout & 0xFF);
    if (off == CFG_OFF_U1_TOH) return (uint8_t)(cfg->uart1_timeout >> 8);
    if (off == CFG_OFF_U1_EN) return cfg->uart1_en;
    return 0;
}

/* ==================== 保存 (无大 buf, 逐字节写, 避免栈溢出) ==================== */
uint8_t config_save(const config_table_t *cfg)
{
    uint16_t i;
    uint8_t crc = 0x00;
    /* CRC-8 覆盖 [0..263] */
    for (i = 0; i < CFG_OFF_CRC; i++)
        crc = config_crc8(crc, cfg_byte_at(cfg, i));
    /* 逐字节写 (EEPROM_update 只写变化字节) */
    for (i = 0; i < CFG_OFF_CRC; i++)
        EEPROM_update(i, cfg_byte_at(cfg, i));
    EEPROM_update(CFG_OFF_CRC, crc);
    return 1;
}

/* ==================== 读取 ==================== */
uint8_t config_load(config_table_t *cfg)
{
    uint8_t i;

    if (!config_valid()) return 0;   /* 逐字节校验, 不构造大 buf */

    /* 解包: 射频白名单 */
    for (i = 0; i < RF_CFG_N; i++)
        cfg->rf_cfg[i] = EEPROM_read(CFG_OFF_RF + i);

    /* 解包: 发射任务表 (小端) */
    for (i = 0; i < 32; i++) {
        cfg->tx_content[i]  = EEPROM_read(CFG_OFF_CONTENT + i);
        cfg->tx_ci2[i]      = EEPROM_read(CFG_OFF_CI2 + i);
        cfg->tx_ena[i]      = EEPROM_read(CFG_OFF_ENA + i);
        cfg->tx_period_l[i] = (uint16_t)EEPROM_read(CFG_OFF_PERIODL + i * 2)
                            | ((uint16_t)EEPROM_read(CFG_OFF_PERIODL + i * 2 + 1) << 8);
        cfg->tx_period_h[i] = (uint16_t)EEPROM_read(CFG_OFF_PERIODH + i * 2)
                            | ((uint16_t)EEPROM_read(CFG_OFF_PERIODH + i * 2 + 1) << 8);
    }

    /* 本机/对端地址 + 收发模式 + 主从位 */
    cfg->self_addr = EEPROM_read(CFG_OFF_SELF);
    cfg->peer_addr = EEPROM_read(CFG_OFF_PEER);
    cfg->rf_mode   = EEPROM_read(CFG_OFF_MODE);
    cfg->rf_role   = EEPROM_read(CFG_OFF_ROLE);

    /* RF 高层参数: 频率(32bit 小端) + SF/BW/CR/功率/前导/同步/LNA */
    cfg->rf_freq = (uint32_t)EEPROM_read(CFG_OFF_FREQ_LO)
                 | ((uint32_t)EEPROM_read(CFG_OFF_FREQ_LO + 1) << 8)
                 | ((uint32_t)EEPROM_read(CFG_OFF_FREQ_HI) << 16)
                 | ((uint32_t)EEPROM_read(CFG_OFF_FREQ_HI + 1) << 24);
    cfg->rf_sf       = EEPROM_read(CFG_OFF_SF);
    cfg->rf_bw       = (uint16_t)EEPROM_read(CFG_OFF_BW)
                     | ((uint16_t)EEPROM_read(CFG_OFF_BW + 1) << 8);
    cfg->rf_cr       = EEPROM_read(CFG_OFF_CR);
    cfg->rf_power    = EEPROM_read(CFG_OFF_POWER);
    cfg->rf_preamble = EEPROM_read(CFG_OFF_PREAMBLE);
    cfg->rf_syncword = EEPROM_read(CFG_OFF_SYNCWORD);
    cfg->rf_lna      = EEPROM_read(CFG_OFF_LNA);

    /* 频偏校正值 + 校正开关 (开关: 非1一律当 0=关, 默认不校正) */
    cfg->fe_value  = (int16_t)((uint16_t)EEPROM_read(CFG_OFF_FE_LO)
                             | ((uint16_t)EEPROM_read(CFG_OFF_FE_HI) << 8));
    cfg->fe_enable = (EEPROM_read(CFG_OFF_FE_EN) == 1) ? 1 : 0;

    /* UART1 485: 波特率 + 缓冲上限 + 组帧超时 + 使能 (非1一律当 0=关) */
    cfg->uart1_baud    = (uint16_t)EEPROM_read(CFG_OFF_U1_BDL)
                       | ((uint16_t)EEPROM_read(CFG_OFF_U1_BDH) << 8);
    cfg->uart1_buf_max = EEPROM_read(CFG_OFF_U1_BMAX);
    cfg->uart1_timeout = (uint16_t)EEPROM_read(CFG_OFF_U1_TOL)
                       | ((uint16_t)EEPROM_read(CFG_OFF_U1_TOH) << 8);
    cfg->uart1_en      = (EEPROM_read(CFG_OFF_U1_EN) == 1) ? 1 : 0;
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
    uint16_t i;
    for (i = 0; i < CFG_LEN; i++)
        EEPROM_write(i, 0xFF);   /* 逐字节, 无大 buf, 避免栈溢出 */
}
