/*
 * rf_dbg.c - RA-01 SX1278 调试驱动实现 (test/rf_debug 独立工程)
 * 硬件: RST=PH4, DIO0=PH5, NSS=PH6(SPI); SPI Mode0/6MHz (见 spi.h)
 */
#include "rf_dbg.h"
#include "spi.h"
#include <Arduino.h>
#include <stm8s.h>

/* 自动 ACK 开关 (收到数据帧自动回 0xAA; 平时一直 RXCONT) */
uint8_t RF_DBG_ACK = 1;

/* OPMODE 位 */
#define OPMODE_LONGRANGE 0x80
/* 注意: SX1278 是固定低频芯片(410-525MHz), OPMODE bit3(LowFrequencyModeOn)
 * 是保留位, 不能设 1 (设 1 反而导致 RX 模式进不去; 该位仅 SX1276 高低频切换用) */
#define OPMODE_MODE_MASK 0x07
#define OPMODE_STDBY     0x01
#define OPMODE_TX        0x03
#define OPMODE_RXCONT    0x05
#define OPMODE_RXSINGLE  0x06

/* IRQ 标志 */
#define IRQ_TX_DONE         0x08
#define IRQ_PAYLOAD_CRC_ERR 0x20
#define IRQ_RX_DONE         0x40

/* 引脚 */
#define RF_RST_PIN  PH4

/* ==================== 寄存器读写 ==================== */
uint8_t rf_dbg_read_reg(uint8_t addr)
{
    uint8_t v;
    spi_begin(SPI_SLAVE_RF);
    spi_transfer(addr & 0x7F);          /* 读指令 bit7=0 */
    v = spi_transfer(0x00);
    spi_end(SPI_SLAVE_RF);
    return v;
}

void rf_dbg_write_reg(uint8_t addr, uint8_t val)
{
    spi_begin(SPI_SLAVE_RF);
    spi_transfer(0x80 | (addr & 0x7F)); /* 写指令 bit7=1 */
    spi_transfer(val);
    spi_end(SPI_SLAVE_RF);
}

/* 切换操作模式: 直接写完整值 (参考 RadioHead/Semtech)
 * bit7(LongRange) 只能在 Sleep 模式修改, 非 Sleep 写被硬件忽略(保持), 仅 mode 位生效
 */
static void opmode(uint8_t mode)
{
    uint8_t i;
    rf_dbg_write_reg(RF_REG_OPMODE, OPMODE_LONGRANGE | mode);
    /* 等模式切换 + PLL 锁定(~2ms)。不用读回确认:
     * 进 RXCONT 后读回有 STDBY 假象, 确认循环会阻塞 50ms,
     * 导致接收板长时间不在 RXCONT 错过单帧(link 失败根因)。 */
    for (i = 0; i < 2; i++) delay(1);
}

/* Rx 链校准 (SX1278 上电/复位后必须做, 否则接收链 PLL 无法锁定 -> 进 RX 被拒回 STDBY)
 * SX1278 是纯低频芯片, 只做 LF 频段校准 (官方切 868MHz 是 SX1276 HF 用的)
 * 做法: 切断 PA -> 写 IMAGECAL_START -> 等 RUNNING 清除 -> 恢复 PA */
static void rf_dbg_rx_chain_cal(void)
{
    uint8_t pa = rf_dbg_read_reg(RF_REG_PA_CONFIG);
    volatile uint32_t guard = 0;

    rf_dbg_write_reg(RF_REG_PA_CONFIG, 0x00);        /* 切断 PA (校准要求) */
    rf_dbg_write_reg(RF_REG_IMAGECAL, 0x40);         /* IMAGECAL_START (LF 校准) */
    while ((rf_dbg_read_reg(RF_REG_IMAGECAL) & 0x20) && (guard++ < 100000)) ; /* 等 RUNNING 清除 */
    rf_dbg_write_reg(RF_REG_PA_CONFIG, pa);          /* 恢复 PA */
}

/* ==================== 初始化 ==================== */
uint8_t rf_dbg_init(void)
{
    volatile uint32_t i;
    uint8_t om;

    pinMode(RF_RST_PIN, OUTPUT);
    pinMode(PH5, INPUT);                 /* DIO0 (调试用, 轮询) */

    /* 硬件复位脉冲: 低->高 (SX1278 要求 RST 低 >100us) */
    digitalWrite(RF_RST_PIN, HIGH);
    digitalWrite(RF_RST_PIN, LOW);
    for (i = 0; i < 20000; i++) ;
    digitalWrite(RF_RST_PIN, HIGH);
    for (i = 0; i < 20000; i++) ;

    /* Rx 链校准: 必须在 reset 后、未设 LongRange(LoRa) 前做!
     * 关键: 0x3B 在 FSK 模式=IMAGECAL, LoRa 模式=INVERTIQ2。
     * 之前在校准前先设了 LongRange(LoRa), 写 0x3B 实际写的是 INVERTIQ2(错)
     * -> I/Q 解调异常(RSSI 正常但解调不出帧)。先 FSK STDBY 再校准(0x3B=IMAGECAL)。 */
    rf_dbg_write_reg(RF_REG_OPMODE, 0x01);   /* FSK STDBY (0x3B=IMAGECAL 有效) */
    for (i = 0; i < 1000; i++) ;
    rf_dbg_rx_chain_cal();

    /* 强制进 Sleep (LongRange 位只能在 Sleep 模式修改;
     * 重烧 MCU 不会复位 SX1278, 芯片可能停在 STDBY/RX 等状态) */
    rf_dbg_write_reg(RF_REG_OPMODE, 0x00);
    for (i = 0; i < 10000; i++) ;
    /* Sleep + LongRange (0x80), 并等接管 + 读回验证 */
    rf_dbg_write_reg(RF_REG_OPMODE, OPMODE_LONGRANGE | 0x00);
    for (i = 0; i < 10000; i++) ;
    om = rf_dbg_read_reg(RF_REG_OPMODE);
    if ((om & OPMODE_LONGRANGE) == 0) {
        /* 重试一次 (偶发时序) */
        rf_dbg_write_reg(RF_REG_OPMODE, OPMODE_LONGRANGE | 0x00);
        for (i = 0; i < 10000; i++) ;
        om = rf_dbg_read_reg(RF_REG_OPMODE);
        if ((om & OPMODE_LONGRANGE) == 0)
            return 0;   /* LongRange 未生效 */
    }

    /* Rx 链校准(已在 reset 后 FSK 模式做过) */

    /* 默认参数 (对齐官方驱动; 关键: 显式头模式必须设 PayloadLength/MaxPayloadLength) */
    opmode(OPMODE_STDBY);
    rf_dbg_set_freq(470000000UL);

    /* I/Q 极性 (对齐官方 SetRx IqInverted=false):
     * 校准写 0x3B=0x40 会残留到 LoRa 模式的 INVERTIQ2, 必须重设 0x1D(off);
     * INVERTIQ(0x33): RX off + TX off = (读&0xBE)|0x01 */
    rf_dbg_write_reg(0x33, (rf_dbg_read_reg(0x33) & 0xBE) | 0x01);
    rf_dbg_write_reg(0x3B, 0x1D);

    rf_dbg_write_reg(RF_REG_MODEM_CONFIG_1, 0x72);   /* BW125 CR4/5 显式头 */
    rf_dbg_write_reg(RF_REG_MODEM_CONFIG_2, 0x74);   /* SF7 + CRC on */
    rf_dbg_write_reg(RF_REG_MODEM_CONFIG_3, 0x04);   /* AGC 自动 (0x26) */
    rf_dbg_write_reg(RF_REG_SYMB_TIMEOUT_LSB, 0x64); /* 符号超时 100 (0x1F) */
    rf_dbg_write_reg(RF_REG_PREAMBLE_MSB, 0x00);     /* 前导码 8 符号 */
    rf_dbg_write_reg(RF_REG_PREAMBLE_LSB, 0x08);
    rf_dbg_write_reg(RF_REG_PAYLOAD_LENGTH, 255);    /* 显式头接收最大长度 */
    rf_dbg_write_reg(RF_REG_MAX_PAYLOAD_LENGTH, 255);
    rf_dbg_write_reg(RF_REG_HOP_PERIOD, 0x00);       /* 关跳频 */
    rf_dbg_write_reg(RF_REG_SYNC_WORD, 0x12);        /* 同步字 (两板一致) */
    rf_dbg_write_reg(RF_REG_PA_CONFIG, 0x80 | 0x0F); /* +17dBm PA_BOOST (最大) */
    rf_dbg_write_reg(RF_REG_LNA, 0x23);              /* 增益默认 + boost */

    /* 接收机检测配置 (对齐官方 SX1276SetModem, 漏了 RA-01 可能检测不到帧):
     * DETECTOPTIMIZE(0x31): SF7-12 低3位=0x03; ERRATA2.3 清 bit7(BW125)
     * DETECTIONTHRESHOLD(0x37)=0x0A (SF7-12)
     * TEST36(0x36)=0x03 (ERRATA 2.1, 非500kHz) */
    rf_dbg_write_reg(0x31, (rf_dbg_read_reg(0x31) & 0x7F) | 0x03);
    rf_dbg_write_reg(0x37, 0x0A);
    rf_dbg_write_reg(0x36, 0x03);
    /* ERRATA 2.3 (官方 SX1276SetRx 必做): BW125 TEST30/TEST2F */
    rf_dbg_write_reg(0x30, 0x00);                         /* TEST30 */
    rf_dbg_write_reg(0x2F, 0x40);                         /* TEST2F (BW125) */

    /* 对齐官方 SX1276SetRx: 屏蔽杂散中断(轮询收, DIO 映射无妨) */
    rf_dbg_write_reg(RF_REG_IRQ_FLAGS_MASK, 0x1F);   /* 屏蔽 VALIDHEADER/TXDONE/CADDONE/FHSS/CADDETECTED */
    rf_dbg_write_reg(RF_REG_DIO_MAPPING1, 0x00);     /* DIO0=RxDone */

    /* 进 RXCONT: 先清 IRQ 标志 + 设 FIFO */
    rf_dbg_rx_start();
    /* 等模式切换 + PLL 锁定完成 (~数ms), 避免 init 返回后立即读回仍是 STDBY(0x81)
     * 造成"RX 没进"假象 (实际 RX 能进, dump 显示 0x85 稳定) */
    for (i = 0; i < 100000; i++) ;
    return 1;
}

/* ==================== 参数设置 ==================== */
void rf_dbg_set_freq(uint32_t hz)
{
    /* frf = hz * 2^19 / 32e6。用浮点避免 SDCC 64 位左移溢出 bug
     * (uint64<<19 在 SDCC 下会出错, 导致频率算错)。 */
    double d = (double)hz * 524288.0 / 32000000.0;
    uint32_t frf = (uint32_t)d;
    opmode(OPMODE_STDBY);
    rf_dbg_write_reg(RF_REG_FRF_MSB, (frf >> 16) & 0xFF);
    rf_dbg_write_reg(RF_REG_FRF_MID, (frf >> 8) & 0xFF);
    rf_dbg_write_reg(RF_REG_FRF_LSB, frf & 0xFF);
}

void rf_dbg_set_modem(uint8_t sf, uint16_t bw_khz, uint8_t cr)
{
    uint8_t mc1, mc2, bw, crc;

    if (sf < 6) sf = 6; if (sf > 12) sf = 12;
    if (cr < 5) cr = 5; if (cr > 8)  cr = 8;
    if (bw_khz == 250) bw = 0x8;
    else if (bw_khz == 500) bw = 0x9;
    else { bw_khz = 125; bw = 0x7; }
    crc = (uint8_t)(cr - 4);

    opmode(OPMODE_STDBY);
    mc1 = (uint8_t)((bw << 4) | (crc << 1));          /* 显式头 bit0=0 */
    mc2 = (uint8_t)((sf << 4) | 0x04);                /* SF + RxPayloadCrcOn */
    rf_dbg_write_reg(RF_REG_MODEM_CONFIG_1, mc1);
    rf_dbg_write_reg(RF_REG_MODEM_CONFIG_2, mc2);
    rf_dbg_write_reg(RF_REG_MODEM_CONFIG_3, 0x04);    /* AGC */
}

void rf_dbg_set_power(int8_t dbm)
{
    if (dbm < 2) dbm = 2;
    if (dbm > 17) dbm = 17;
    opmode(OPMODE_STDBY);
    rf_dbg_write_reg(RF_REG_PA_CONFIG, 0x80 | (uint8_t)(dbm - 2));  /* PA_BOOST */
}

/* ==================== 频偏测量与校正 ==================== */
/* 读当前 FRF 对应频率 Hz */
uint32_t rf_dbg_get_freq(void)
{
    uint32_t frf = ((uint32_t)rf_dbg_read_reg(RF_REG_FRF_MSB) << 16) |
                   ((uint32_t)rf_dbg_read_reg(RF_REG_FRF_MID) << 8) |
                   rf_dbg_read_reg(RF_REG_FRF_LSB);
    /* frf = hz * 2^19 / 32e6 -> hz = frf * 32e6 / 2^19 */
    return (uint32_t)((double)frf * 32000000.0 / 524288.0);
}

/* 读 FrequencyError (20位有符号): FEI 单位 = FXTAL/2^19 = 61.035 Hz。
 * 收到对端有效 LoRa 包后读才有意义。返回 Hz (正=对端频率比本机高)。 */
int32_t rf_dbg_read_fe(void)
{
    uint32_t fei = ((uint32_t)rf_dbg_read_reg(RF_REG_FEI_MSB) << 16) |
                   ((uint32_t)rf_dbg_read_reg(RF_REG_FEI_MID) << 8) |
                   rf_dbg_read_reg(RF_REG_FEI_LSB);
    int32_t fei_s = (int32_t)(fei & 0xFFFFF);   /* 低 20 位 */
    if (fei_s & 0x80000)                        /* 20 位符号位 */
        fei_s -= 0x100000;
    return (int32_t)((double)fei_s * 32000000.0 / 524288.0);
}

/* 扫频精调: 在 base±range 内扫描 FRF, 对每个候选频率进 RX 等一帧,
 * 收到任意一帧就读 FrequencyError 并据此校正本机频率 (= 候选频率 - FE),
 * 一次校正即对准 (不设 FE 阈值, 因为频偏可能很大)。
 * 返回校正后本机频率 Hz; 全扫无果返回 0。
 * 说明: 对端须持续发帧 (脚本 autotune 配合 txc 连发)。 */
uint32_t rf_dbg_tune(uint32_t base, int32_t range)
{
    int32_t step = 25000;                /* 细扫 25kHz 步进(频率偏差225kHz级, 100kHz步进会漏掉实际匹配点) */
    int32_t off;
    for (off = -range; off <= range; off += step) {
        uint8_t buf[64], len;
        int32_t fe;
        uint8_t r;
        uint32_t f = (uint32_t)((int32_t)base + off);
        rf_dbg_set_freq(f);
        /* 关键修复: 改频后必须重新做 RxChainCalibration!
         * 记忆已记录: 校准必须在设频率后、进RX前做, 否则 RX 链未校准,
         * 芯片拒绝正常接收 -> 扫频所有点都收不到 (表现为"扫描无果")。
         * crcscan 能通是因为板B 固定 470, 只用 init 时校准过的频率;
         * tune 每次改频没校准, 这就是 autotune 三次失败的根因。 */
        rf_dbg_rx_chain_cal();
        /* 进 RX 等一帧 (阻塞 600ms) */
        r = rf_dbg_rx(buf, &len, 600, NULL, NULL);
        /* 收到任一帧(RxDone, 不管 CRC 对=1 或 错=2)都算锁定 -> 读 FE 精调。
         * FrequencyError 在帧同步后即有效, 不依赖 CRC 正确。 */
        if (r == 1 || r == 2) {
            fe = rf_dbg_read_fe();
            /* 收到帧: 用 FE 精调。本机频率 = 当前候选频率 - FE */
            {
                uint32_t tuned = (uint32_t)((int32_t)f - fe);
                rf_dbg_set_freq(tuned);
                return tuned;
            }
        }
    }
    return 0;
}

/* ==================== 发送 (阻塞, 带超时) ==================== */
void rf_dbg_tx(const uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint16_t w;

    opmode(OPMODE_STDBY);
    /* 关键: 允许 TxDone 中断(mask=0x17, 0x1F 屏蔽了 bit3=TxDone)。
     * 若 mask 屏蔽 TxDone, 下面等 TxDone 会卡 500ms 超时 -> 发送方 500ms 后才
     * 回接收, 错过对端早已发出的 ACK/响应 (link/link2 发送方收不到 ACK 根因) */
    rf_dbg_write_reg(RF_REG_IRQ_FLAGS_MASK, 0x17);   /* 屏蔽 CAD/FHSS/ValidHeader, 允许 TxDone */
    rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
    rf_dbg_write_reg(RF_REG_FIFO_TX_BASE, 0x80);   /* 官方默认, 避免与 RX 基址 0x00 重叠 */
    rf_dbg_write_reg(RF_REG_FIFO_ADDR_PTR, 0x80);
    for (i = 0; i < len; i++)
        rf_dbg_write_reg(RF_REG_FIFO, buf[i]);
    rf_dbg_write_reg(RF_REG_PAYLOAD_LENGTH, len);
    opmode(OPMODE_TX);

    /* 等 TxDone, 最多 ~500ms (防死锁) */
    for (w = 0; w < 500; w++) {
        if (rf_dbg_read_reg(RF_REG_IRQ_FLAGS) & IRQ_TX_DONE) break;
        delay(1);
    }

    rf_dbg_rx_full();    /* 发完自动回接收: 完整恢复(TX会破坏RX链, 需切FSK重校准再回LoRa) */
}

/* ==================== 接收 (阻塞, 超时) ====================
 * 返回: 1=收到OK  2=收到但CRC错(*len回填字节数, 带RSSI/SNR)  0=超时
 * RSSI(dBm) = -164 + RegPktRssiValue (SX1278 低频); SNR(dB) = RegPktSnrValue/4
 */
uint8_t rf_dbg_rx(uint8_t *buf, uint8_t *len, uint16_t timeout_ms,
                  int8_t *rssi_dbm, int8_t *snr_db)
{
    uint16_t t;
    uint8_t flags;

    opmode(OPMODE_STDBY);
    rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
    rf_dbg_write_reg(RF_REG_FIFO_RX_BASE, 0x00);
    rf_dbg_write_reg(RF_REG_FIFO_ADDR_PTR, 0x00);
    opmode(OPMODE_RXCONT);

    for (t = 0; t < timeout_ms; t++) {
        flags = rf_dbg_read_reg(RF_REG_IRQ_FLAGS);
        if (flags & IRQ_RX_DONE) {
            uint8_t n, i;
            opmode(OPMODE_STDBY);
            n = rf_dbg_read_reg(RF_REG_RX_NB_BYTES);
            if (rssi_dbm)
                *rssi_dbm = (int8_t)((int16_t)rf_dbg_read_reg(RF_REG_PKT_RSSI) - 164);
            if (snr_db)
                *snr_db = (int8_t)((int8_t)rf_dbg_read_reg(RF_REG_PKT_SNR) / 4);

            /* 关键修复: FIFO 指针设置要在 CRC 检查前, 否则 CRC 错时跳过该行导致读垃圾 */
            rf_dbg_write_reg(RF_REG_FIFO_ADDR_PTR, rf_dbg_read_reg(RF_REG_FIFO_RX_BASE));
            for (i = 0; i < n; i++)
                buf[i] = rf_dbg_read_reg(RF_REG_FIFO);

            if (flags & IRQ_PAYLOAD_CRC_ERR) {
                *len = n;                        /* 收到但 CRC 错 */
                rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
                return 2;
            }
            *len = n;
            rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
            return 1;
        }
        delay(1);
    }

    opmode(OPMODE_STDBY);
    rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
    return 0;
}

/* ==================== 连续接收控制 (能量检测) ==================== */
void rf_dbg_rx_start(void)
{
    opmode(OPMODE_STDBY);
    /* 对齐官方 SX1276SetRx: 每次进 RX 前重做 I/Q 极性 + ERRATA 2.3(BW125)
     * + 接收检测配置, 防止模式切换后寄存器被重置导致解调异常 */
    rf_dbg_write_reg(0x33, (rf_dbg_read_reg(0x33) & 0xBE) | 0x01);  /* INVERTIQ RX off */
    rf_dbg_write_reg(0x3B, 0x1D);                                     /* INVERTIQ2 off */
    rf_dbg_write_reg(0x31, (rf_dbg_read_reg(0x31) & 0x7F) | 0x03);
    rf_dbg_write_reg(0x30, 0x00);
    rf_dbg_write_reg(0x2F, 0x40);
    rf_dbg_write_reg(RF_REG_IRQ_FLAGS_MASK, 0x1F);   /* RX 模式: 屏蔽 TxDone/ValidHeader 等 */
    rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
    rf_dbg_write_reg(RF_REG_FIFO_RX_BASE, 0x00);
    rf_dbg_write_reg(RF_REG_FIFO_ADDR_PTR, 0x00);
    opmode(OPMODE_RXCONT);   /* 连续接收: 一直监听不超时 */
}

/* TX→RX 完整恢复: TX 模式会破坏 RX 链校准状态, 发完帧后若直接回 RXCONT 收不到
 * 对端响应(link/link2 里发送方收不到 ACK 的根因)。
 * 校准必须在 FSK 模式做(0x3B=IMAGECAL, LoRa 下 0x3B=INVERTIQ2), 所以:
 * Sleep(清LongRange) -> FSK STDBY -> 重校准 -> Sleep -> LoRa STDBY -> rx_start */
void rf_dbg_rx_full(void)
{
    volatile uint32_t i;
    rf_dbg_write_reg(RF_REG_OPMODE, 0x00);   /* Sleep (清 LongRange) */
    for (i = 0; i < 1000; i++) ;
    rf_dbg_write_reg(RF_REG_OPMODE, 0x01);   /* FSK STDBY (0x3B=IMAGECAL 有效) */
    for (i = 0; i < 1000; i++) ;
    rf_dbg_rx_chain_cal();                    /* 重新校准 RX 链 */
    rf_dbg_write_reg(RF_REG_OPMODE, 0x00);   /* Sleep */
    for (i = 0; i < 1000; i++) ;
    rf_dbg_write_reg(RF_REG_OPMODE, OPMODE_LONGRANGE | 0x01);  /* LoRa STDBY */
    for (i = 0; i < 1000; i++) ;
    rf_dbg_rx_start();                        /* 完整配置 + RXCONT */
}

void rf_dbg_rx_stop(void)
{
    opmode(OPMODE_STDBY);
    rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
}

int8_t rf_dbg_cur_rssi(void)
{
    /* 实时 RSSI: 读 RegRssiValue(0x1B), 不是 PktRssiValue(0x1A=上一包的RSSI,
     * 没收到包时为 0, 会显示 -164 溢出假象)。0x1B 才是当前信道实时 RSSI。 */
    return (int8_t)((int16_t)rf_dbg_read_reg(0x1B) - 164);
}

uint8_t rf_dbg_irq(void)
{
    return rf_dbg_read_reg(RF_REG_IRQ_FLAGS);
}

/* ==================== 自动 ACK + 非阻塞接收 ==================== */
void rf_dbg_set_ack(uint8_t on)
{
    RF_DBG_ACK = on ? 1 : 0;
}

uint8_t rf_dbg_rx_check(uint8_t *buf, uint8_t *len, int8_t *rssi, int8_t *snr)
{
    uint8_t flags = rf_dbg_read_reg(RF_REG_IRQ_FLAGS);

    if (!(flags & IRQ_RX_DONE)) {
        /* RXCONT 会一直监听; 若掉出接收(超时/异常)则清标志重进 */
        if ((rf_dbg_read_reg(RF_REG_OPMODE) & 0x07) != 0x05) {
            rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
            rf_dbg_write_reg(RF_REG_FIFO_RX_BASE, 0x00);
            rf_dbg_write_reg(RF_REG_FIFO_ADDR_PTR, 0x00);
            opmode(OPMODE_RXCONT);
        }
        return 0;          /* 无事件 */
    }

    opmode(OPMODE_STDBY);
    {
        uint8_t n = rf_dbg_read_reg(RF_REG_RX_NB_BYTES);
        if (rssi)
            *rssi = (int8_t)((int16_t)rf_dbg_read_reg(RF_REG_PKT_RSSI) - 164);
        if (snr)
            *snr = (int8_t)((int8_t)rf_dbg_read_reg(RF_REG_PKT_SNR) / 4);

        if (flags & IRQ_PAYLOAD_CRC_ERR) {
            rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
            opmode(OPMODE_RXCONT);
            return 3;                               /* 收到但 CRC 错 */
        }

        rf_dbg_write_reg(RF_REG_FIFO_ADDR_PTR, rf_dbg_read_reg(RF_REG_FIFO_RX_BASE));
        {
            uint8_t i;
            for (i = 0; i < n && i < 64; i++)
                buf[i] = rf_dbg_read_reg(RF_REG_FIFO);
        }
        *len = n;
        rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
        opmode(OPMODE_RXCONT);

        if (n == 1 && buf[0] == RF_DBG_ACK_BYTE)
            return 2;                               /* ACK 帧 */
        return 1;                                   /* 数据帧 */
    }
}
