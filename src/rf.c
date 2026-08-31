/*
 * rf.c - RA-01 (SX1278) LoRa 驱动实现
 *
 * 硬件: NSS=PH6 (spi), RST=PH4, DIO0=PH5; SPI Mode0/6MHz (见 spi.h)
 *
 * 调制参数 (待用户确认, 当前为常用默认值):
 *   频率 470MHz, SF7, BW125kHz, CR4/5, 显式头, 发射 13dBm
 * 修改下方宏后, 需同步更新 rf_init() 中 ModemConfig/PA 的预计算值
 */
#include "rf.h"
#include "spi.h"
#include "rf_app.h"
#include "uart3.h"
#include "timer.h"
#include "dbg.h"
#include <Arduino.h>
#include <stm8s.h>

/* ==================== 调制参数 (待确认) ==================== */
#define RF_FREQ_HZ      470000000UL  /* 载波频率 Hz */
#define RF_SF           7            /* 扩频因子 6..12 */
#define RF_BW           125          /* 带宽 kHz: 125/250/500 */
#define RF_CR           5            /* 编码率 5..8 (= 4/5..4/8) */
#define RF_POWER_DBM    13           /* 发射功率 dBm */

/* ==================== FSK 参数 (UART3_RF_RADIO=1 时生效) ====================
 * 默认值对应"极限速率留 0.5 余地" (极限 ~200kbps -> 100kbps)
 * 可被 0x60~0x7F 直写 SX1278 0x02~0x1F 覆盖 (BitRate/Fdev/RxBw 等)
 */
#define RF_FSK_BITRATE   100000UL    /* BitRate bps */
#define RF_FSK_FDEV      50000UL     /* Fdev Hz */
#define RF_FSK_RXBW      200000UL    /* RxBw Hz */
#define RF_FSK_SYNC      0xC1        /* FSK 同步字节 (SYNCVALUE1) */
#define RF_FSK_FIXED     0           /* 包格式: 0=可变 1=固定 */
#define RF_FSK_CRC       1           /* CRC: 1=开 0=关 */

/* ==================== SX1278 寄存器 ==================== */
#define REG_FIFO            0x00
#define REG_OPMODE          0x01
#define REG_FRF_MSB         0x06
#define REG_FRF_MID         0x07
#define REG_FRF_LSB         0x08
#define REG_PA_CONFIG       0x09
#define REG_LNA             0x0C
#define REG_FIFO_ADDR_PTR   0x0D
#define REG_FIFO_TX_BASE    0x0E
#define REG_FIFO_RX_BASE    0x0F
#define REG_IRQ_FLAGS_MASK  0x11
#define REG_IRQ_FLAGS       0x12
#define REG_RX_NB_BYTES     0x13
#define REG_PKT_SNR         0x19
#define REG_PKT_RSSI        0x1A
#define REG_RSSI_VALUE      0x1B
#define REG_MODEM_CONFIG_1  0x1D
#define REG_MODEM_CONFIG_2  0x1E
#define REG_SYMB_TIMEOUT_LSB 0x1F   /* 0x1F 是符号超时低字节(不是 ModemConfig3!) */
#define REG_PREAMBLE_MSB    0x20
#define REG_PREAMBLE_LSB    0x21
#define REG_PAYLOAD_LENGTH  0x22
#define REG_MAX_PAYLOAD_LENGTH 0x23
#define REG_HOP_PERIOD      0x24
#define REG_MODEM_CONFIG_3  0x26   /* 真正地址, 非 0x1F */
#define REG_FEI_MSB         0x28
#define REG_FEI_MID         0x29
#define REG_FEI_LSB         0x2A
#define REG_DETECTOPTIMIZE  0x31
#define REG_DETECTIONTHRESHOLD 0x37
#define REG_TEST30          0x30
#define REG_TEST2F          0x2F
#define REG_TEST36          0x36
#define REG_INVERTIQ        0x33
#define REG_INVERTIQ2       0x3B   /* LoRa 模式 0x3B=INVERTIQ2 (FSK=IMAGECAL) */
#define REG_IMAGECAL        0x3B   /* FSK 模式 0x3B=IMAGECAL */
#define REG_SYNC_WORD       0x39
#define REG_VERSION         0x42

/* ---- FSK 模式专属寄存器 (LoRa 模式这些地址语义不同) ---- */
#define REG_BITRATEMSB      0x02
#define REG_BITRATELSB      0x03
#define REG_FDEVMSB         0x04
#define REG_FDEVLSB         0x05
#define REG_RXCONFIG        0x0D
#define REG_RSSIVALUE       0x11
#define REG_RXBW            0x12
#define REG_AFCBW           0x13
#define REG_PREAMBLEMSB_FSK 0x25
#define REG_PREAMBLELSB_FSK 0x26
#define REG_SYNCCONFIG      0x27
#define REG_SYNCVALUE1      0x28
#define REG_PACKETCONFIG1   0x30
#define REG_PACKETCONFIG2   0x31
#define REG_PAYLOADLEN_FSK  0x32
#define REG_NODEADRS        0x33
#define REG_BROADCASTADRS   0x34
#define REG_IRQFLAGS1       0x3E
#define REG_IRQFLAGS2       0x3F
#define REG_DIOMAPPING1     0x40
#define REG_DIOMAPPING2     0x41

/* FSK IRQ 标志 (IRQFLAGS2=0x3F) */
#define FSK_IRQ2_PACKETSENT   0x08
#define FSK_IRQ2_PAYLOADREADY 0x04
#define FSK_IRQ2_CRCOK        0x02
#define FSK_IRQ2_FIFOOVERRUN  0x10
/* FSK IRQ 标志 (IRQFLAGS1=0x3E) */
#define FSK_IRQ1_PREAMBLEDETECT 0x02
#define FSK_IRQ1_SYNCADDRMATCH  0x01

/* OPMODE 位 */
#define OPMODE_LONGRANGE    0x80   /* LoRa 模式 */
#define OPMODE_MODE_MASK    0x07
#define OPMODE_SLEEP        0x00
#define OPMODE_STDBY        0x01
#define OPMODE_TX           0x03
#define OPMODE_RXCONT       0x05

/* IRQ 标志 */
#define IRQ_TX_DONE         0x08
#define IRQ_PAYLOAD_CRC_ERR 0x20
#define IRQ_RX_DONE         0x40

/* 引脚 */
#define RF_RST_PIN  PH4
#define RF_DIO0_PIN PH5

/* 非阻塞收发状态 (TIM4 轮询; 见下方非阻塞区) */
static uint8_t RF_ENABLED = 0;        /* rf_init 完成置位, 供 TIM4 轮询判断 */
volatile uint8_t RF_SPI_BUSY = 0; /* SPI 占用计数(>0=占用): spi_begin/end + 长事务; TIM4 rf_poll 让路 */

/* 简单空循环延时 (项目约定不调用 delay()/millis()) */
static void rf_delay(volatile uint32_t n)
{
    while (n) n--;
}

/* ==================== 寄存器读写 ==================== */
uint8_t rf_read_reg(uint8_t addr)
{
    uint8_t v;
    spi_begin(SPI_SLAVE_RF);
    spi_transfer(addr & 0x7F);          /* 读指令: bit7=0 */
    v = spi_transfer(0x00);
    spi_end(SPI_SLAVE_RF);
    return v;
}

void rf_write_reg(uint8_t addr, uint8_t val)
{
    spi_begin(SPI_SLAVE_RF);
    spi_transfer(0x80 | (addr & 0x7F)); /* 写指令: bit7=1 */
    spi_transfer(val);
    spi_end(SPI_SLAVE_RF);
}

/* 切换操作模式 (读-改-写) */
static void rf_set_opmode(uint8_t mode)
{
    rf_write_reg(REG_OPMODE, (rf_read_reg(REG_OPMODE) & ~OPMODE_MODE_MASK) | mode);
}

uint8_t rf_read_version(void)
{
    return rf_read_reg(REG_VERSION);
}

/* 设置载波频率 (Frf = f*2^19/32MHz; 用浮点避免 SDCC (uint64)<<19 溢出 bug) */
void rf_set_frequency(uint32_t freq_hz)
{
    double d = (double)freq_hz * 524288.0 / 32000000.0;
    uint32_t frf = (uint32_t)d;
    rf_write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF);
    rf_write_reg(REG_FRF_MID, (frf >> 8) & 0xFF);
    rf_write_reg(REG_FRF_LSB, frf & 0xFF);
}

/* ==================== 频偏校正测量 (控制指令 0x26/模式3 自动) ====================
 * FEI (REG 0x28~0x2A) 是 LoRa 收到一帧后的频率误差估计 (19bit 补码步数),
 * 仅在收到帧后有效。Fstep = Fxtal/2^19 = 32M/2^19 ≈ 61.035Hz。
 */
uint8_t rf_freq_correct_measure(int32_t *out_hz)
{
    uint8_t msb, mid, lsb, sign;
    int32_t fei;

    if (!RF_ENABLED || !out_hz) return 0;

    msb  = rf_read_reg(REG_FEI_MSB);
    mid  = rf_read_reg(REG_FEI_MID);
    lsb  = rf_read_reg(REG_FEI_LSB);
    sign = (uint8_t)(msb & 0x08);              /* bit19 符号位 */
    fei  = ((int32_t)(msb & 0x07) << 16)
         | ((int32_t)mid << 8) | (int32_t)lsb; /* 19bit */
    if (sign) fei -= 524288;                   /* 2^19 补码 -> 有符号 */

    *out_hz = fei * 61;                        /* Fstep ≈ 61.035Hz -> Hz */
    return 1;
}

/* 应用频偏校正: 载波 = UART3_RF_FREQ + offset_hz */
void rf_set_freq_offset(int16_t offset_hz)
{
    rf_set_frequency((uint32_t)((int32_t)UART3_RF_FREQ + offset_hz));
}

/* ==================== 接收链校准 ====================
 * 必须在 reset 后、FSK 模式(0x3B=IMAGECAL)下做!
 * 关键: 0x3B 在 FSK=IMAGECAL, LoRa=INVERTIQ2。
 * 若在 LoRa 模式校准会写错寄存器 -> I/Q 解调异常(RSSI 正常但解调不出帧)。 */
static void rf_rx_chain_cal(void)
{
    uint8_t pa = rf_read_reg(REG_PA_CONFIG);
    volatile uint32_t guard = 0;
    rf_write_reg(REG_PA_CONFIG, 0x00);        /* 切断 PA */
    rf_write_reg(REG_IMAGECAL, 0x40);         /* IMAGECAL_START (LF) */
    while ((rf_read_reg(REG_IMAGECAL) & 0x20) && (guard++ < 100000)) ;  /* 等 RUNNING 清 */
    rf_write_reg(REG_PA_CONFIG, pa);
}

/* 进 RXCONT (轻量, 无校准): 每次重做 I/Q 极性 + ERRATA 2.3 + 接收检测配置 + mask。
 * 校准只在 init 做一次(FSK 模式); 这里不含校准以保持 ISR 轻量。 */
static void rf_rx_start(void)
{
    rf_set_opmode(OPMODE_STDBY);
    rf_write_reg(REG_INVERTIQ, (rf_read_reg(REG_INVERTIQ) & 0xBE) | 0x01);  /* RX off + TX off */
    rf_write_reg(REG_INVERTIQ2, 0x1D);        /* INVERTIQ2 off (LoRa 模式 0x3B) */
    rf_write_reg(REG_DETECTOPTIMIZE, (rf_read_reg(REG_DETECTOPTIMIZE) & 0x7F) | 0x03); /* SF7-12 + 清bit7 */
    rf_write_reg(REG_TEST30, 0x00);
    rf_write_reg(REG_TEST2F, 0x40);           /* ERRATA 2.3 BW125 */
    rf_write_reg(REG_IRQ_FLAGS_MASK, 0x1F);   /* RX: 屏蔽 TxDone/ValidHeader 等 */
    rf_write_reg(REG_IRQ_FLAGS, 0xFF);
    rf_write_reg(REG_FIFO_RX_BASE, 0x00);
    rf_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    rf_set_opmode(OPMODE_RXCONT);
}

/* ==================== FSK RxBw 寄存器值查表 ==================== */
static uint8_t rf_fsk_bw_reg(uint32_t bw)
{
    /* {带宽Hz, RegValue} (参考 Semtech FskBandwidths 表) */
    static const uint16_t bw_tbl[][2] = {
        {2600,0x17},{3100,0x0F},{3900,0x07},{5200,0x16},{6300,0x0E},{7800,0x06},
        {10400,0x15},{12500,0x0D},{15600,0x05},{20800,0x14},{25000,0x0C},{31300,0x04},
        {41700,0x13},{50000,0x0B},{62500,0x03},{83333,0x12},{100000,0x0A},{125000,0x02},
        {166700,0x11},{200000,0x09},{250000,0x01},
    };
    uint8_t i;
    for (i = 0; i < 21; i++)
        if (bw <= bw_tbl[i][0]) return (uint8_t)bw_tbl[i][1];
    return 0x00;   /* 超出 -> 最大带宽 */
}

/* ==================== FSK 进 RX (轻量) ====================
 * DIO0=PayloadReady(00), DIO2=SyncAddr(11); AFCAUTO+AGCAUTO+RxTrig=PreambleDetect
 * (参考 Semtech SX1276SetRx FSK 分支)
 */
static void rf_fsk_rx_start(void)
{
    rf_set_opmode(OPMODE_STDBY);
    rf_write_reg(REG_DIOMAPPING1, 0x0C);   /* DIO2=11 SyncAddr, DIO0/1=00 */
    rf_write_reg(REG_DIOMAPPING2, 0x30);   /* DIO5=ModeReady */
    rf_write_reg(REG_RXCONFIG, 0x1E);      /* AFCAUTO|AGCAUTO|RxTrigPreambleDetect */
    rf_write_reg(REG_IRQFLAGS1, 0xFF);     /* 清 FSK 中断 */
    rf_write_reg(REG_IRQFLAGS2, 0xFF);
    rf_set_opmode(OPMODE_RXCONT);
}

/* ==================== FSK 模式配置 (UART3_RF_RADIO=1) ====================
 * 物理参数(BitRate/Fdev/RxBw)用宏默认, 可被 0x60~0x7F 直写 SX1278 0x02~0x1F 覆盖
 */
static void rf_fsk_apply_config(void)
{
    uint16_t br, fdev;
    uint8_t pc1;

    /* 切 FSK: Sleep -> 清 LONGRANGE -> STDBY */
    rf_write_reg(REG_OPMODE, 0x00);   /* Sleep (FSK) */
    rf_delay(500);
    rf_write_reg(REG_OPMODE, 0x01);   /* STDBY FSK (无 LONGRANGE) */
    rf_delay(500);

    rf_set_frequency(UART3_RF_FREQ);

    /* BitRate = 32MHz / BR */
    br = (uint16_t)(32000000UL / RF_FSK_BITRATE);
    rf_write_reg(REG_BITRATEMSB, (uint8_t)(br >> 8));
    rf_write_reg(REG_BITRATELSB, (uint8_t)(br & 0xFF));
    /* Fdev = Fdev / 61.035 */
    fdev = (uint16_t)(RF_FSK_FDEV / 61UL);
    rf_write_reg(REG_FDEVMSB, (uint8_t)(fdev >> 8));
    rf_write_reg(REG_FDEVLSB, (uint8_t)(fdev & 0xFF));
    /* RxBw + AfcBw */
    rf_write_reg(REG_RXBW,  rf_fsk_bw_reg(RF_FSK_RXBW));
    rf_write_reg(REG_AFCBW, rf_fsk_bw_reg(RF_FSK_RXBW));

    /* PA + LNA */
    rf_write_reg(REG_PA_CONFIG, 0x80 | (UART3_RF_POWER & 0x0F));
    rf_write_reg(REG_LNA, UART3_RF_LNA);

    /* 前导 (FSK 0x25/0x26) */
    rf_write_reg(REG_PREAMBLEMSB_FSK, (uint8_t)(UART3_RF_PREAMBLE >> 8));
    rf_write_reg(REG_PREAMBLELSB_FSK, (uint8_t)(UART3_RF_PREAMBLE & 0xFF));

    /* 包格式: PACKETCONFIG1 = 包格式(可变/固定) + CRC */
    pc1 = (uint8_t)((RF_FSK_FIXED ? 0x00 : 0x80) | (RF_FSK_CRC ? 0x10 : 0x00));
    rf_write_reg(REG_PACKETCONFIG1, pc1);
    rf_write_reg(REG_PACKETCONFIG2, 0x40);   /* DATAMODE_PACKET */
    /* 载荷长度: 固定包=RF_RX_MAX, 可变包=0xFF(最大) */
    rf_write_reg(REG_PAYLOADLEN_FSK, RF_FSK_FIXED ? RF_RX_MAX : 0xFF);

    /* 同步字: SYNCCONFIG = SYNC_ON + 1字节(SYNCSIZE=0), 同步字节用 UART3_RF_SYNCWORD */
    rf_write_reg(REG_SYNCCONFIG, 0x10);
    rf_write_reg(REG_SYNCVALUE1, UART3_RF_SYNCWORD);
    /* 节点/广播地址过滤关闭 (参考驱动默认) */
    rf_write_reg(REG_NODEADRS, 0x00);
    rf_write_reg(REG_BROADCASTADRS, 0x00);

    /* 进 FSK RXCONT */
    rf_fsk_rx_start();
    RF_ENABLED = 1;
}

/* ==================== 按配置应用 RF 参数 ====================
 * 按 UART3_RF_* (0x30~0x38, 可存 EEPROM) 配置 SX1278: 载波频率 + 调制(SF/BW/CR) +
 * 功率 + 前导 + 同步字 + LNA, 然后进 RXCONT。空速由 SF/BW/CR 自动算。
 * 在 rf_init 后调用; uart3_config_restore 恢复 EEPROM 后也需重新调用。 */
void rf_apply_config(void)
{
    uint8_t bw_code, cr_code, sf_code, mc1, mc2;

    if (UART3_RF_RADIO) {              /* FSK 模式 (0x2F=1) */
        rf_fsk_apply_config();
        return;
    }

    /* 载波频率 */
    rf_set_frequency(UART3_RF_FREQ);

    /* ModemConfig1: BW(bit7:4) | CR(bit3:1) | 显式头(bit0=0) */
    bw_code = (UART3_RF_BW == 125) ? 7 : (UART3_RF_BW == 250) ? 8 : 9;
    cr_code = (uint8_t)(UART3_RF_CR - 4);          /* 5->1, 6->2, 7->3, 8->4 */
    if (cr_code < 1 || cr_code > 4) cr_code = 1;
    mc1 = (uint8_t)((bw_code << 4) | (cr_code << 1));
    rf_write_reg(REG_MODEM_CONFIG_1, mc1);

    /* ModemConfig2: SF(bit7:4) | RX CRC on(bit2) */
    if (UART3_RF_SF < 6) UART3_RF_SF = 7;
    if (UART3_RF_SF > 12) UART3_RF_SF = 12;
    sf_code = (uint8_t)((UART3_RF_SF << 4) & 0xF0);
    mc2 = (uint8_t)(sf_code | 0x04);
    rf_write_reg(REG_MODEM_CONFIG_2, mc2);

    rf_write_reg(REG_MODEM_CONFIG_3, 0x04);        /* AGC 自动 */
    rf_write_reg(REG_SYMB_TIMEOUT_LSB, 0x64);      /* 符号超时 */

    /* 前导码 */
    rf_write_reg(REG_PREAMBLE_MSB, (uint8_t)(UART3_RF_PREAMBLE >> 8));
    rf_write_reg(REG_PREAMBLE_LSB, (uint8_t)(UART3_RF_PREAMBLE & 0xFF));

    /* 发射功率 (PA_BOOST | dBm) */
    rf_write_reg(REG_PA_CONFIG, 0x80 | (UART3_RF_POWER & 0x0F));
    /* LNA */
    rf_write_reg(REG_LNA, UART3_RF_LNA);
    /* 同步字 */
    rf_write_reg(REG_SYNC_WORD, UART3_RF_SYNCWORD);

    /* 进 RXCONT (含 I/Q 极性 + ERRATA 2.3 + mask) */
    rf_rx_start();
    RF_ENABLED = 1;              /* 唤醒: 允许 TIM4 rf_poll 接管收发 */
}

/* ==================== RF 休眠/唤醒 (模式切换用) ====================
 * rf_sleep(): 进 STDBY, 停止 RF 收发 (省电+彻底静默; 模式1/4 用)
 * 唤醒: 直接调用 rf_apply_config() 重新进 RXCONT (模式2/3 用)
 */
void rf_sleep(void)
{
    RF_ENABLED = 0;              /* 让 TIM4 rf_poll 让路, 不再收发 */
    rf_set_opmode(OPMODE_STDBY); /* STDBY: 停止 RF, 保留 LoRa 配置 */
}

/* ==================== 初始化 ==================== */
void rf_init(void)
{
    /* 引脚: RST 输出, DIO0 输入 (当前用轮询, 暂不接中断) */
    pinMode(RF_RST_PIN, OUTPUT);
    pinMode(RF_DIO0_PIN, INPUT);

    /* 硬件复位脉冲: 低->高 (SX1278 要求 RST 低电平 >100us) */
    digitalWrite(RF_RST_PIN, HIGH);
    digitalWrite(RF_RST_PIN, LOW);
    rf_delay(1000);
    digitalWrite(RF_RST_PIN, HIGH);
    rf_delay(1000);

    /* Rx 链校准: reset 后、FSK 模式(0x3B=IMAGECAL)下做 (官方时序) */
    rf_write_reg(REG_OPMODE, 0x01);           /* FSK STDBY */
    rf_delay(500);
    rf_rx_chain_cal();

    /* Sleep + LongRange (LoRa) */
    rf_write_reg(REG_OPMODE, 0x00);           /* Sleep (FSK) */
    rf_delay(500);
    rf_write_reg(REG_OPMODE, OPMODE_LONGRANGE | OPMODE_SLEEP);
    rf_delay(500);
    rf_set_opmode(OPMODE_STDBY);

    /* 按 UART3_RF_* 配置载波/调制/功率/前导/同步/LNA + 进 RXCONT */
    rf_apply_config();

    rf_write_reg(REG_PAYLOAD_LENGTH, 255);
    rf_write_reg(REG_MAX_PAYLOAD_LENGTH, 255);
    rf_write_reg(REG_HOP_PERIOD, 0x00);

    /* 接收机检测配置 (官方 SX1276SetModem): TEST36=0x03 (非500k, ERRATA 2.1) */
    rf_write_reg(REG_TEST36, 0x03);
    rf_delay(500);

    RF_ENABLED = 1;   /* 允许 TIM4 轮询接管收发 */
}

/* ==================== 发送 ==================== */
void rf_send(const uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint16_t w;

    rf_set_opmode(OPMODE_STDBY);
    rf_write_reg(REG_IRQ_FLAGS_MASK, 0x17);   /* 允许 TxDone, 否则等不到(会死锁) */
    rf_write_reg(REG_IRQ_FLAGS, 0xFF);        /* 清中断标志 */

    /* 写 FIFO (用 0x80, 避免与 RX 基址 0x00 重叠) */
    rf_write_reg(REG_FIFO_TX_BASE, 0x80);
    rf_write_reg(REG_FIFO_ADDR_PTR, 0x80);
    for (i = 0; i < len; i++)
        rf_write_reg(REG_FIFO, buf[i]);
    rf_write_reg(REG_PAYLOAD_LENGTH, len);

    /* 进入 TX, 阻塞等 TxDone (带超时防死锁) */
    rf_set_opmode(OPMODE_TX);
    for (w = 0; w < 500; w++) {
        if (rf_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE) break;
        rf_delay(500);
    }
    rf_write_reg(REG_IRQ_FLAGS, 0xFF);
    rf_rx_start();       /* 发完回接收 (含 I/Q/ERRATA/mask 配置) */
}

/* ==================== 非阻塞收发状态 ==================== */
volatile uint8_t RF_TX_BUSY = 0;      /* 发送忙 */
volatile uint8_t RF_TX_DONE = 0;      /* 发送完成标志 */
volatile uint16_t RF_TX_START_MS = 0; /* 发送开始时刻 (TICK_MS, 超时兜底用) */

/* ---- 环形接收缓冲: ISR 快速搬入原始帧, 主循环解析 (防溢出) ---- */
typedef struct {
    uint8_t data[RF_RX_MAX];   /* 原始帧 */
    uint8_t len;
    int8_t  rssi;              /* 该帧 RSSI (dBm) */
    int8_t  snr;               /* 该帧 SNR (dB) */
} RF_QITEM;
static volatile RF_QITEM RF_RX_Q[RF_RX_QUEUE];
static volatile uint8_t  RF_RX_Q_WR = 0;   /* 写指针 (ISR) */
static volatile uint8_t  RF_RX_Q_RD = 0;   /* 读指针 (主循环) */
volatile uint8_t RF_RX_OVF = 0;            /* 溢出标志: 主循环亮 SYS 灯 */
volatile int8_t  RF_LAST_RSSI = -127;      /* 最近一帧 RSSI */
volatile int8_t  RF_LAST_SNR  = 0;         /* 最近一帧 SNR */

/* 非阻塞发送: 提交一帧后立即返回; TxDone 由 TIM4 轮询清除忙状态 */
uint8_t rf_tx_start(const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if (RF_TX_BUSY) return 1;         /* 忙: 拒绝新任务 (上层据此亮 SYSTEM_LED) */

    RF_SPI_BUSY++;                  /* 长事务占用 SPI (计数), TIM4 轮询让路 */
    rf_set_opmode(OPMODE_STDBY);
    if (UART3_RF_RADIO) {
        /* FSK: 无 FIFO 基址, 直接 REG_FIFO; 清 FSK 中断 */
        rf_write_reg(REG_IRQFLAGS1, 0xFF);
        rf_write_reg(REG_IRQFLAGS2, 0xFF);
        for (i = 0; i < len; i++)
            rf_write_reg(REG_FIFO, buf[i]);
        rf_write_reg(REG_PAYLOADLEN_FSK, len);
    } else {
        rf_write_reg(REG_IRQ_FLAGS_MASK, 0x17);   /* 关键: 允许 TxDone, 否则 rf_poll 等不到 */
        rf_write_reg(REG_IRQ_FLAGS, 0xFF);  /* 清中断标志 */
        rf_write_reg(REG_FIFO_TX_BASE, 0x80);
        rf_write_reg(REG_FIFO_ADDR_PTR, 0x80);
        for (i = 0; i < len; i++)
            rf_write_reg(REG_FIFO, buf[i]);
        rf_write_reg(REG_PAYLOAD_LENGTH, len);
    }
    rf_set_opmode(OPMODE_TX);         /* 进入 TX (此后 TxDone 由 TIM4 轮询收尾) */
    RF_SPI_BUSY--;                    /* 释放 (内部各步 spi_begin/end 已 ++/--) */

    RF_TX_BUSY = 1;
    RF_TX_DONE = 0;
    RF_TX_START_MS = TICK_MS;
    return 0;
}

/* ==================== 发送超时兜底 ====================
 * TxDone 长时间未检测到时强制回接收, 防 RF_TX_BUSY 卡死 (SYS 灯/后续发送/接收全堵)
 * 由主循环 rf_app_poll 检测 500ms 超时后调用
 */
void rf_abort_tx(void)
{
    RF_TX_BUSY = 0;
    if (UART3_RF_RADIO) rf_fsk_rx_start();
    else                rf_rx_start();
}

/* ==================== TIM4 ISR 轮询 ====================
 * 由 timer.c 的 TIM4 中断每 167us 调用一次:
 *   平时只读 DIO0 电平, 无事件则立即返回 (开销极小);
 *   有事件才做 SPI 读 + 收包入缓冲 + 清忙。
 * 半双工: 处理完回 RXCONT (接收常态)。
 */
void rf_poll(void)
{
    uint8_t flags;



    /* SPI 被主循环占用(RF 发送/DAC)时让路: 接收数据留芯片内 FIFO(RxDone 保持),
     * 发送 SPI 事务先完成, 之后 SPI 空闲再搬 -> 时序不乱 + 不丢数据 */
    if (!RF_ENABLED || RF_SPI_BUSY) return;

    /* 发送忙: 直接轮询查 TxDone/PacketSent (不依赖 DIO0 的 TX 模式映射,
     * 实测 DIO0 发送完成后不拉高 -> 卡 RF_TX_BUSY + 卡 TX 模式不接收) */
    if (RF_TX_BUSY) {
        if (UART3_RF_RADIO) {
            flags = rf_read_reg(REG_IRQFLAGS2);      /* FSK: PacketSent 在 IRQFLAGS2 */
            if (flags & FSK_IRQ2_PACKETSENT) {       /* 发送完成 */
                rf_write_reg(REG_IRQFLAGS1, 0xFF);
                rf_write_reg(REG_IRQFLAGS2, 0xFF);
                RF_TX_BUSY = 0;
                RF_TX_DONE = 1;
                rf_fsk_rx_start();                   /* 发完回接收 */
            }
        } else {
            flags = rf_read_reg(REG_IRQ_FLAGS);
            if (flags & IRQ_TX_DONE) {               /* 发送完成 */
                rf_write_reg(REG_IRQ_FLAGS, 0xFF);
                RF_TX_BUSY = 0;
                RF_TX_DONE = 1;
                rf_rx_start();                       /* 发完回接收(含 I/Q/ERRATA/mask 配置) */
            }
        }
        return;
    }

    if (!(GPIOH->IDR & 0x20)) return;         /* 非发送: DIO0(PH5) 低无事件 */

    if (UART3_RF_RADIO) {
        /* ---- FSK 收包 (IRQFLAGS2 PayloadReady) ---- */
        flags = rf_read_reg(REG_IRQFLAGS2);
        if (flags & FSK_IRQ2_PAYLOADREADY) {
            uint8_t n, i;
            if ((rf_read_reg(REG_IRQFLAGS2) & FSK_IRQ2_CRCOK)) {
                n = rf_read_reg(REG_PAYLOADLEN_FSK);   /* 可变包: 硬件更新实际长度 */
                if (n > RF_RX_MAX) n = RF_RX_MAX;
                {
                    uint8_t nxt = (uint8_t)(RF_RX_Q_WR + 1) % RF_RX_QUEUE;
                    if (nxt == RF_RX_Q_RD) {
                        RF_RX_OVF = 1;                 /* 环形缓冲满: 溢出 */
                    } else {
                        RF_RX_Q[RF_RX_Q_WR].len = n;
                        for (i = 0; i < n; i++)
                            RF_RX_Q[RF_RX_Q_WR].data[i] = rf_read_reg(REG_FIFO);
                        RF_RX_Q[RF_RX_Q_WR].rssi = (int8_t)(-((int16_t)rf_read_reg(REG_RSSIVALUE) >> 1));
                        RF_RX_Q[RF_RX_Q_WR].snr  = 0;  /* FSK 无 SNR */
                        RF_RX_Q_WR = nxt;
                    }
                }
            } else {
                RF_APP_CRC_CNT++;                      /* 统计: CRC 错帧 */
            }
            rf_write_reg(REG_IRQFLAGS1, 0xFF);
            rf_write_reg(REG_IRQFLAGS2, 0xFF);
            rf_fsk_rx_start();                         /* 回接收 */
        }
        return;
    }

    flags = rf_read_reg(REG_IRQ_FLAGS);

    if (flags & IRQ_RX_DONE) {                /* 收到一包: 快速搬入环形缓冲(不解析) */
        uint8_t n, i;
        if (!(flags & IRQ_PAYLOAD_CRC_ERR)) {
            n = rf_read_reg(REG_RX_NB_BYTES);
            if (n > RF_RX_MAX) n = RF_RX_MAX;
            rf_write_reg(REG_FIFO_ADDR_PTR, rf_read_reg(REG_FIFO_RX_BASE));
            {
                uint8_t nxt = (uint8_t)(RF_RX_Q_WR + 1) % RF_RX_QUEUE;
                if (nxt == RF_RX_Q_RD) {
                    RF_RX_OVF = 1;            /* 环形缓冲满: 溢出, 主循环亮 SYS 灯 */
                } else {
                    RF_RX_Q[RF_RX_Q_WR].len = n;
                    for (i = 0; i < n; i++)
                        RF_RX_Q[RF_RX_Q_WR].data[i] = rf_read_reg(REG_FIFO);
                    RF_RX_Q[RF_RX_Q_WR].rssi = (int8_t)((int16_t)rf_read_reg(REG_PKT_RSSI) - 164);
                    RF_RX_Q[RF_RX_Q_WR].snr  = (int8_t)((int8_t)rf_read_reg(REG_PKT_SNR) / 4);
                    RF_RX_Q_WR = nxt;
                }
            }
        } else {
            RF_APP_CRC_CNT++;                 /* 统计: CRC 错帧 */
        }
        rf_write_reg(REG_IRQ_FLAGS, 0xFF);
        rf_rx_start();                        /* 回接收(含配置) */
    }
}

/* 主循环: 从环形接收缓冲取一帧; 取到则清溢出标志(恢复, 灭溢出灯) */
uint8_t rf_rx_pop(uint8_t *buf, uint8_t *len, int8_t *rssi, int8_t *snr)
{
    if (RF_RX_Q_RD == RF_RX_Q_WR) return 0;
    {
        uint8_t n = RF_RX_Q[RF_RX_Q_RD].len, i;
        for (i = 0; i < n; i++) buf[i] = RF_RX_Q[RF_RX_Q_RD].data[i];
        *len = n;
        if (rssi) *rssi = RF_RX_Q[RF_RX_Q_RD].rssi;
        if (snr)  *snr  = RF_RX_Q[RF_RX_Q_RD].snr;
        RF_RX_Q_RD = (uint8_t)(RF_RX_Q_RD + 1) % RF_RX_QUEUE;
        RF_RX_OVF = 0;                        /* 已恢复处理, 灭溢出灯 */
        return 1;
    }
}

/* ==================== 接收 ==================== */
uint8_t rf_receive(uint8_t *buf, uint8_t *len, uint16_t timeout_ms)
{
    uint16_t t0 = rtc_get_ms();
    uint8_t n, i;

    rf_rx_start();   /* 完整接收配置 (I/Q/ERRATA/mask/FIFO) */

    for (;;) {
        uint8_t flags = rf_read_reg(REG_IRQ_FLAGS);
        if (flags & IRQ_RX_DONE) {
            if (flags & IRQ_PAYLOAD_CRC_ERR) {
                /* CRC 错误: 清标志继续监听 */
                rf_write_reg(REG_IRQ_FLAGS, 0xFF);
                continue;
            }
            n = rf_read_reg(REG_RX_NB_BYTES);
            rf_write_reg(REG_FIFO_ADDR_PTR, rf_read_reg(REG_FIFO_RX_BASE));
            for (i = 0; i < n; i++)
                buf[i] = rf_read_reg(REG_FIFO);
            *len = n;
            rf_write_reg(REG_IRQ_FLAGS, 0xFF);
            rf_set_opmode(OPMODE_STDBY);
            return 1;
        }
        /* 超时 (RTC 毫秒, 处理回绕) */
        if ((uint16_t)(rtc_get_ms() - t0) >= timeout_ms) {
            rf_set_opmode(OPMODE_STDBY);
            return 0;
        }
    }
}
