/*
 * rf.c - RA-01 (SX1278) LoRa 椹卞姩瀹炵幇
 *
 * 纭欢: NSS=PH6 (spi), RST=PH4, DIO0=PH5; SPI Mode0/6MHz (瑙?spi.h)
 *
 * 璋冨埗鍙傛暟 (寰呯敤鎴风‘璁? 褰撳墠涓哄父鐢ㄩ粯璁ゅ€?:
 *   棰戠巼 470MHz, SF7, BW125kHz, CR4/5, 鏄惧紡澶? 鍙戝皠 13dBm
 * 淇敼涓嬫柟瀹忓悗, 闇€鍚屾鏇存柊 rf_init() 涓?ModemConfig/PA 鐨勯璁＄畻鍊? */
#include "rf.h"
#include "spi.h"
#include "rf_app.h"
#include "uart3.h"
#include "timer.h"
#include "dbg.h"
#include <Arduino.h>
#include <stm8s.h>

/* ==================== 璋冨埗鍙傛暟 (寰呯‘璁? ==================== */
#define RF_FREQ_HZ      470000000UL  /* 杞芥尝棰戠巼 Hz */
#define RF_SF           7            /* 鎵╅鍥犲瓙 6..12 */
#define RF_BW           125          /* 甯﹀ kHz: 125/250/500 */
#define RF_CR           5            /* 缂栫爜鐜?5..8 (= 4/5..4/8) */
#define RF_POWER_DBM    13           /* 鍙戝皠鍔熺巼 dBm */

/* ==================== FSK 鍙傛暟 (UART3_RF_RADIO=1 鏃剁敓鏁? ====================
 * 榛樿鍊煎搴?鏋侀檺閫熺巼鐣?0.5 浣欏湴" (鏋侀檺 ~200kbps -> 100kbps)
 * 鍙 0x60~0x7F 鐩村啓 SX1278 0x02~0x1F 瑕嗙洊 (BitRate/Fdev/RxBw 绛?
 */
#define RF_FSK_BITRATE   100000UL    /* BitRate bps */
#define RF_FSK_FDEV      50000UL     /* Fdev Hz */
#define RF_FSK_RXBW      200000UL    /* RxBw Hz */
#define RF_FSK_SYNC      0xC1        /* FSK 鍚屾瀛楄妭 (SYNCVALUE1) */
#define RF_FSK_FIXED     0           /* 鍖呮牸寮? 0=鍙彉 1=鍥哄畾 */
#define RF_FSK_CRC       1           /* CRC: 1=寮€ 0=鍏?*/

/* ==================== SX1278 瀵勫瓨鍣?==================== */
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
#define REG_SYMB_TIMEOUT_LSB 0x1F   /* 0x1F 鏄鍙疯秴鏃朵綆瀛楄妭(涓嶆槸 ModemConfig3!) */
#define REG_PREAMBLE_MSB    0x20
#define REG_PREAMBLE_LSB    0x21
#define REG_PAYLOAD_LENGTH  0x22
#define REG_MAX_PAYLOAD_LENGTH 0x23
#define REG_HOP_PERIOD      0x24
#define REG_MODEM_CONFIG_3  0x26   /* 鐪熸鍦板潃, 闈?0x1F */
#define REG_FEI_MSB         0x28
#define REG_FEI_MID         0x29
#define REG_FEI_LSB         0x2A
#define REG_DETECTOPTIMIZE  0x31
#define REG_DETECTIONTHRESHOLD 0x37
#define REG_TEST30          0x30
#define REG_TEST2F          0x2F
#define REG_TEST36          0x36
#define REG_INVERTIQ        0x33
#define REG_INVERTIQ2       0x3B   /* LoRa 妯″紡 0x3B=INVERTIQ2 (FSK=IMAGECAL) */
#define REG_IMAGECAL        0x3B   /* FSK 妯″紡 0x3B=IMAGECAL */
#define REG_SYNC_WORD       0x39
#define REG_VERSION         0x42

/* ---- FSK 妯″紡涓撳睘瀵勫瓨鍣?(LoRa 妯″紡杩欎簺鍦板潃璇箟涓嶅悓) ---- */
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
#define REG_SYNCVALUE2      0x29
#define REG_SYNCVALUE3      0x2A
#define REG_PACKETCONFIG1   0x30
#define REG_PACKETCONFIG2   0x31
#define REG_PAYLOADLEN_FSK  0x32
#define REG_NODEADRS        0x33
#define REG_BROADCASTADRS   0x34
#define REG_IRQFLAGS1       0x3E
#define REG_IRQFLAGS2       0x3F
#define REG_DIOMAPPING1     0x40
#define REG_DIOMAPPING2     0x41
#define REG_RSSICONFIG      0x0E
#define REG_AFCFEI          0x1A
#define REG_PREAMBLEDETECT  0x1F
#define REG_OSC             0x24
#define REG_FIFOTHRESH      0x35

/* FSK IRQ 鏍囧織 (IRQFLAGS2=0x3F) */
#define FSK_IRQ2_PACKETSENT   0x08
#define FSK_IRQ2_PAYLOADREADY 0x04
#define FSK_IRQ2_CRCOK        0x02
#define FSK_IRQ2_FIFOOVERRUN  0x10
/* FSK IRQ 鏍囧織 (IRQFLAGS1=0x3E) */
#define FSK_IRQ1_PREAMBLEDETECT 0x02
#define FSK_IRQ1_SYNCADDRMATCH  0x01

/* OPMODE 浣?*/
#define OPMODE_LONGRANGE    0x80   /* LoRa 妯″紡 */
#define OPMODE_MODE_MASK    0x07
#define OPMODE_SLEEP        0x00
#define OPMODE_STDBY        0x01
#define OPMODE_TX           0x03
#define OPMODE_RXCONT       0x05

/* IRQ 鏍囧織 */
#define IRQ_TX_DONE         0x08
#define IRQ_PAYLOAD_CRC_ERR 0x20
#define IRQ_RX_DONE         0x40

/* 寮曡剼 */
#define RF_RST_PIN  PH4
#define RF_DIO0_PIN PH5

/* 闈為樆濉炴敹鍙戠姸鎬?(TIM4 杞; 瑙佷笅鏂归潪闃诲鍖? */
static uint8_t RF_ENABLED = 0;        /* rf_init 瀹屾垚缃綅, 渚?TIM4 杞鍒ゆ柇 */
volatile uint8_t RF_SPI_BUSY = 0; /* SPI 鍗犵敤璁℃暟(>0=鍗犵敤): spi_begin/end + 闀夸簨鍔? TIM4 rf_poll 璁╄矾 */

/* 绠€鍗曠┖寰幆寤舵椂 (椤圭洰绾﹀畾涓嶈皟鐢?delay()/millis()) */
static void rf_delay(volatile uint32_t n)
{
    while (n) n--;
}

/* ==================== 瀵勫瓨鍣ㄨ鍐?==================== */
uint8_t rf_read_reg(uint8_t addr)
{
    uint8_t v;
    spi_begin(SPI_SLAVE_RF);
    spi_transfer(addr & 0x7F);          /* 璇绘寚浠? bit7=0 */
    v = spi_transfer(0x00);
    spi_end(SPI_SLAVE_RF);
    return v;
}

void rf_write_reg(uint8_t addr, uint8_t val)
{
    spi_begin(SPI_SLAVE_RF);
    spi_transfer(0x80 | (addr & 0x7F)); /* 鍐欐寚浠? bit7=1 */
    spi_transfer(val);
    spi_end(SPI_SLAVE_RF);
}

/* 鍒囨崲鎿嶄綔妯″紡 (璇?鏀?鍐? */
static void rf_set_opmode(uint8_t mode)
{
    rf_write_reg(REG_OPMODE, (rf_read_reg(REG_OPMODE) & ~OPMODE_MODE_MASK) | mode);
}

uint8_t rf_read_version(void)
{
    return rf_read_reg(REG_VERSION);
}

/* 璁剧疆杞芥尝棰戠巼 (Frf = f*2^19/32MHz; 鐢ㄦ诞鐐归伩鍏?SDCC (uint64)<<19 婧㈠嚭 bug) */
void rf_set_frequency(uint32_t freq_hz)
{
    double d = (double)freq_hz * 524288.0 / 32000000.0;
    uint32_t frf = (uint32_t)d;
    rf_write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF);
    rf_write_reg(REG_FRF_MID, (frf >> 8) & 0xFF);
    rf_write_reg(REG_FRF_LSB, frf & 0xFF);
}

/* ==================== 棰戝亸鏍℃娴嬮噺 (鎺у埗鎸囦护 0x26/妯″紡3 鑷姩) ====================
 * FEI (REG 0x28~0x2A) 鏄?LoRa 鏀跺埌涓€甯у悗鐨勯鐜囪宸及璁?(19bit 琛ョ爜姝ユ暟),
 * 浠呭湪鏀跺埌甯у悗鏈夋晥銆侳step = Fxtal/2^19 = 32M/2^19 鈮?61.035Hz銆? */
uint8_t rf_freq_correct_measure(int32_t *out_hz)
{
    uint8_t msb, mid, lsb, sign;
    int32_t fei;

    if (!RF_ENABLED || !out_hz) return 0;

    msb  = rf_read_reg(REG_FEI_MSB);
    mid  = rf_read_reg(REG_FEI_MID);
    lsb  = rf_read_reg(REG_FEI_LSB);
    sign = (uint8_t)(msb & 0x08);              /* bit19 绗﹀彿浣?*/
    fei  = ((int32_t)(msb & 0x07) << 16)
         | ((int32_t)mid << 8) | (int32_t)lsb; /* 19bit */
    if (sign) fei -= 524288;                   /* 2^19 琛ョ爜 -> 鏈夌鍙?*/

    *out_hz = fei * 61;                        /* Fstep 鈮?61.035Hz -> Hz */
    return 1;
}

/* 搴旂敤棰戝亸鏍℃: 杞芥尝 = UART3_RF_FREQ + offset_hz */
void rf_set_freq_offset(int16_t offset_hz)
{
    rf_set_frequency((uint32_t)((int32_t)UART3_RF_FREQ + offset_hz));
}

/* ==================== 鎺ユ敹閾炬牎鍑?====================
 * 蹇呴』鍦?reset 鍚庛€丗SK 妯″紡(0x3B=IMAGECAL)涓嬪仛!
 * 鍏抽敭: 0x3B 鍦?FSK=IMAGECAL, LoRa=INVERTIQ2銆? * 鑻ュ湪 LoRa 妯″紡鏍″噯浼氬啓閿欏瘎瀛樺櫒 -> I/Q 瑙ｈ皟寮傚父(RSSI 姝ｅ父浣嗚В璋冧笉鍑哄抚)銆?*/
static void rf_rx_chain_cal(void)
{
    uint8_t pa = rf_read_reg(REG_PA_CONFIG);
    volatile uint32_t guard = 0;
    rf_write_reg(REG_PA_CONFIG, 0x00);        /* 鍒囨柇 PA */
    rf_write_reg(REG_IMAGECAL, 0x40);         /* IMAGECAL_START (LF) */
    while ((rf_read_reg(REG_IMAGECAL) & 0x20) && (guard++ < 100000)) ;  /* 绛?RUNNING 娓?*/
    rf_write_reg(REG_PA_CONFIG, pa);
}

/* 杩?RXCONT (杞婚噺, 鏃犳牎鍑?: 姣忔閲嶅仛 I/Q 鏋佹€?+ ERRATA 2.3 + 鎺ユ敹妫€娴嬮厤缃?+ mask銆? * 鏍″噯鍙湪 init 鍋氫竴娆?FSK 妯″紡); 杩欓噷涓嶅惈鏍″噯浠ヤ繚鎸?ISR 杞婚噺銆?*/
static void rf_rx_start(void)
{
    rf_set_opmode(OPMODE_STDBY);
    rf_write_reg(REG_INVERTIQ, (rf_read_reg(REG_INVERTIQ) & 0xBE) | 0x01);  /* RX off + TX off */
    rf_write_reg(REG_INVERTIQ2, 0x1D);        /* INVERTIQ2 off (LoRa 妯″紡 0x3B) */
    rf_write_reg(REG_DETECTOPTIMIZE, (rf_read_reg(REG_DETECTOPTIMIZE) & 0x7F) | 0x03); /* SF7-12 + 娓卋it7 */
    rf_write_reg(REG_TEST30, 0x00);
    rf_write_reg(REG_TEST2F, 0x40);           /* ERRATA 2.3 BW125 */
    rf_write_reg(REG_IRQ_FLAGS_MASK, 0x1F);   /* RX: 灞忚斀 TxDone/ValidHeader 绛?*/
    rf_write_reg(REG_IRQ_FLAGS, 0xFF);
    rf_write_reg(REG_FIFO_RX_BASE, 0x00);
    rf_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    rf_set_opmode(OPMODE_RXCONT);
}

/* ==================== FSK RxBw 瀵勫瓨鍣ㄥ€兼煡琛?==================== */
static uint8_t rf_fsk_bw_reg(uint32_t bw)
{
    /* {甯﹀Hz, RegValue} (鍙傝€?Semtech FskBandwidths 琛? */
    static const uint16_t bw_tbl[][2] = {
        {500000,0x00},{2600,0x17},{3100,0x0F},{3900,0x07},{5200,0x16},{6300,0x0E},{7800,0x06},
        {10400,0x15},{12500,0x0D},{15600,0x05},{20800,0x14},{25000,0x0C},{31300,0x04},
        {41700,0x13},{50000,0x0B},{62500,0x03},{83333,0x12},{100000,0x0A},{125000,0x02},
        {166700,0x11},{200000,0x09},{250000,0x01},{500000,0x00},  /* 0x00 官方标Invalid(500k), 测试用 */
    };
    uint8_t i;
    for (i = 0; i < 22; i++)
        if (bw <= bw_tbl[i][0]) return (uint8_t)bw_tbl[i][1];
    return 0x00;   /* 瓒呭嚭 -> 鏈€澶у甫瀹?*/
}

/* ==================== FSK 杩?RX (杞婚噺) ====================
 * DIO0=PayloadReady(00), DIO2=SyncAddr(11); AFCAUTO+AGCAUTO+RxTrig=PreambleDetect
 * (鍙傝€?Semtech SX1276SetRx FSK 鍒嗘敮)
 */
static void rf_fsk_rx_start(void)
{
    rf_set_opmode(OPMODE_STDBY);
    rf_write_reg(REG_DIOMAPPING1, 0x0C);   /* DIO2=11 SyncAddr, DIO0/1=00 */
    rf_write_reg(REG_DIOMAPPING2, 0xC1);   /* DIO4=11 Preamble + MAP=PreambleDetect(01), 官方 */
    rf_write_reg(REG_RXCONFIG, 0x1E);      /* AFCAUTO|AGCAUTO|RxTrigPreambleDetect */
    rf_write_reg(REG_IRQFLAGS1, 0xFF);     /* 娓?FSK 涓柇 */
    rf_write_reg(REG_IRQFLAGS2, 0xFF);
    rf_set_opmode(OPMODE_RXCONT);
}

/* ==================== FSK 妯″紡閰嶇疆 (UART3_RF_RADIO=1) ====================
 * 鐗╃悊鍙傛暟(BitRate/Fdev/RxBw)鐢ㄥ畯榛樿, 鍙 0x60~0x7F 鐩村啓 SX1278 0x02~0x1F 瑕嗙洊
 */
static void rf_fsk_apply_config(void)
{
    

    /* 鍒?FSK: Sleep -> 娓?LONGRANGE -> STDBY */
    rf_write_reg(REG_OPMODE, 0x00);   /* Sleep (FSK) */
    rf_delay(500);
    rf_write_reg(REG_OPMODE, 0x01);   /* STDBY FSK (鏃?LONGRANGE) */
    rf_delay(500);

    /* 重新做 Rx 链校准 (0x3B=IMAGECAL): LoRa 切 FSK 时 0x3B 残留 INVERTIQ2(0x1D),
     * 不校准会致 FSK 接收链异常 -> 完全收不到 */
    rf_rx_chain_cal();

    rf_set_frequency(UART3_RF_FREQ);

    /* BitRate = 32MHz / BR */
    rf_write_reg(REG_BITRATEMSB,  UART3_FSK[FSK_IDX_BR_H]);
    rf_write_reg(REG_BITRATELSB,  UART3_FSK[FSK_IDX_BR_L]);
    rf_write_reg(REG_FDEVMSB,    UART3_FSK[FSK_IDX_FDEV_H]);
    rf_write_reg(REG_FDEVLSB,    UART3_FSK[FSK_IDX_FDEV_L]);
    rf_write_reg(REG_RXBW,  UART3_FSK[FSK_IDX_RXBW]);
    rf_write_reg(REG_AFCBW, UART3_FSK[FSK_IDX_AFCBW]);

    /* PA + LNA */
    rf_write_reg(REG_PA_CONFIG, 0x80 | (UART3_RF_POWER & 0x0F));
    rf_write_reg(REG_LNA, UART3_RF_LNA);

    /* 鍓嶅 (FSK 0x25/0x26) */
    rf_write_reg(REG_PREAMBLEMSB_FSK, (uint8_t)(UART3_RF_PREAMBLE >> 8));
    rf_write_reg(REG_PREAMBLELSB_FSK, (uint8_t)(UART3_RF_PREAMBLE & 0xFF));

    /* 鍖呮牸寮? PACKETCONFIG1 = 鍖呮牸寮?鍙彉/鍥哄畾) + CRC */
    rf_write_reg(REG_PACKETCONFIG1, UART3_FSK[FSK_IDX_PKT1]);
    rf_write_reg(REG_PACKETCONFIG2, UART3_FSK[FSK_IDX_PKT2]);
    /* 杞借嵎闀垮害: 鍥哄畾鍖?RF_RX_MAX, 鍙彉鍖?0xFF(鏈€澶? */
    rf_write_reg(REG_PAYLOADLEN_FSK, UART3_FSK[FSK_IDX_PAYLOAD]);

    /* 同步字: 官方 3 字节 C1 94 C1, SYNCCONFIG=0x12 (SYNC_ON + SYNCSIZE=2) */
    rf_write_reg(REG_SYNCCONFIG, 0x12);
    rf_write_reg(REG_SYNCVALUE1, 0xC1);
    rf_write_reg(REG_SYNCVALUE2, 0x94);
    rf_write_reg(REG_SYNCVALUE3, 0xC1);
    /* 鑺傜偣/骞挎挱鍦板潃杩囨护鍏抽棴 (鍙傝€冮┍鍔ㄩ粯璁? */
    rf_write_reg(REG_NODEADRS, UART3_FSK[FSK_IDX_NODE]);
    rf_write_reg(REG_BROADCASTADRS, UART3_FSK[FSK_IDX_BCAST]);

    /* FSK 接收链初始化寄存器 (官方 sx1276 初始化表, 缺省会致接收不工作) */
    rf_write_reg(REG_RSSICONFIG,     0xD2);  /* RSSI smoothing */
    rf_write_reg(REG_AFCFEI,         0x01);  /* AFC auto-clear on */
    rf_write_reg(REG_PREAMBLEDETECT, 0xAA);  /* 前导检测器 (FSK 接收必需) */
    rf_write_reg(REG_OSC,            0x07);  /* clkout off */
    rf_write_reg(REG_FIFOTHRESH,     0x8F);  /* FIFO 阈值 */

    /* 杩?FSK RXCONT */
    rf_fsk_rx_start();
    RF_ENABLED = 1;
}

/* ==================== 鎸夐厤缃簲鐢?RF 鍙傛暟 ====================
 * 鎸?UART3_RF_* (0x30~0x38, 鍙瓨 EEPROM) 閰嶇疆 SX1278: 杞芥尝棰戠巼 + 璋冨埗(SF/BW/CR) +
 * 鍔熺巼 + 鍓嶅 + 鍚屾瀛?+ LNA, 鐒跺悗杩?RXCONT銆傜┖閫熺敱 SF/BW/CR 鑷姩绠椼€? * 鍦?rf_init 鍚庤皟鐢? uart3_config_restore 鎭㈠ EEPROM 鍚庝篃闇€閲嶆柊璋冪敤銆?*/
void rf_apply_config(void)
{
    uint8_t bw_code, cr_code, sf_code, mc1, mc2;

    if (UART3_RF_RADIO) {              /* FSK 妯″紡 (0x2F=1) */
        rf_fsk_apply_config();
        return;
    }

    /* 鍒囧埌 LoRa 妯″紡: Sleep + LongRange + STDBY (纭繚浠?FSK 鍒囧洖鏃?OPMODE bit7=LONGRANGE) */
    rf_write_reg(REG_OPMODE, 0x00);                             /* Sleep (FSK) */
    rf_write_reg(REG_OPMODE, OPMODE_LONGRANGE | OPMODE_SLEEP);  /* Sleep + LongRange (LoRa) */
    rf_set_opmode(OPMODE_STDBY);

    /* 杞芥尝棰戠巼 */
    rf_set_frequency(UART3_RF_FREQ);

    /* ModemConfig1: BW(bit7:4) | CR(bit3:1) | 鏄惧紡澶?bit0=0) */
    bw_code = (UART3_RF_BW == 125) ? 7 : (UART3_RF_BW == 250) ? 8 : 9;
    cr_code = (uint8_t)(UART3_RF_CR - 4);          /* 5->1, 6->2, 7->3, 8->4 */
    if (cr_code < 1 || cr_code > 4) cr_code = 1;
    mc1 = (uint8_t)((bw_code << 4) | (cr_code << 1));
    if (UART3_RF_SF < 6) UART3_RF_SF = 7;
    if (UART3_RF_SF > 12) UART3_RF_SF = 12;
    /* LowDataRateOptimize (bit0): SF>=11 且 BW=125 时符号时长>16ms 必须开启,
     * 否则 SF11/12 @125kHz 无法可靠解调。 */
    rf_write_reg(REG_MODEM_CONFIG_1, mc1);

    /* ModemConfig2: SF(bit7:4) | RX CRC on(bit2) */
    if (UART3_RF_SF < 6) UART3_RF_SF = 7;
    if (UART3_RF_SF > 12) UART3_RF_SF = 12;
    sf_code = (uint8_t)((UART3_RF_SF << 4) & 0xF0);
    mc2 = (uint8_t)(sf_code | 0x04);
    rf_write_reg(REG_MODEM_CONFIG_2, mc2);

    /* LowDataRateOptimize (ModemConfig3 bit3) */
    if (UART3_RF_SF >= 11 && bw_code == 7)
        rf_write_reg(REG_MODEM_CONFIG_3, 0x0C);
    else
        rf_write_reg(REG_MODEM_CONFIG_3, 0x04);

    rf_write_reg(REG_SYMB_TIMEOUT_LSB, 0x64);      /* 绗﹀彿瓒呮椂 */

    /* 鍓嶅鐮?*/
    rf_write_reg(REG_PREAMBLE_MSB, (uint8_t)(UART3_RF_PREAMBLE >> 8));
    rf_write_reg(REG_PREAMBLE_LSB, (uint8_t)(UART3_RF_PREAMBLE & 0xFF));

    /* 鍙戝皠鍔熺巼 (PA_BOOST | dBm) */
    rf_write_reg(REG_PA_CONFIG, 0x80 | (UART3_RF_POWER & 0x0F));
    /* LNA */
    rf_write_reg(REG_LNA, UART3_RF_LNA);
    /* 鍚屾瀛?*/
    rf_write_reg(REG_SYNC_WORD, UART3_RF_SYNCWORD);

    /* 杩?RXCONT (鍚?I/Q 鏋佹€?+ ERRATA 2.3 + mask) */
    rf_rx_start();
    RF_ENABLED = 1;              /* 鍞ら啋: 鍏佽 TIM4 rf_poll 鎺ョ鏀跺彂 */
}

/* ==================== RF 浼戠湢/鍞ら啋 (妯″紡鍒囨崲鐢? ====================
 * rf_sleep(): 杩?STDBY, 鍋滄 RF 鏀跺彂 (鐪佺數+褰诲簳闈欓粯; 妯″紡1/4 鐢?
 * 鍞ら啋: 鐩存帴璋冪敤 rf_apply_config() 閲嶆柊杩?RXCONT (妯″紡2/3 鐢?
 */
void rf_sleep(void)
{
    RF_ENABLED = 0;              /* 璁?TIM4 rf_poll 璁╄矾, 涓嶅啀鏀跺彂 */
    rf_set_opmode(OPMODE_STDBY); /* STDBY: 鍋滄 RF, 淇濈暀 LoRa 閰嶇疆 */
}

/* ==================== 鍒濆鍖?==================== */
void rf_init(void)
{
    /* 寮曡剼: RST 杈撳嚭, DIO0 杈撳叆 (褰撳墠鐢ㄨ疆璇? 鏆備笉鎺ヤ腑鏂? */
    pinMode(RF_RST_PIN, OUTPUT);
    pinMode(RF_DIO0_PIN, INPUT);

    /* 纭欢澶嶄綅鑴夊啿: 浣?>楂?(SX1278 瑕佹眰 RST 浣庣數骞?>100us) */
    digitalWrite(RF_RST_PIN, HIGH);
    digitalWrite(RF_RST_PIN, LOW);
    rf_delay(1000);
    digitalWrite(RF_RST_PIN, HIGH);
    rf_delay(1000);

    /* Rx 閾炬牎鍑? reset 鍚庛€丗SK 妯″紡(0x3B=IMAGECAL)涓嬪仛 (瀹樻柟鏃跺簭) */
    rf_write_reg(REG_OPMODE, 0x01);           /* FSK STDBY */
    rf_delay(500);
    rf_rx_chain_cal();

    /* Sleep + LongRange (LoRa) */
    rf_write_reg(REG_OPMODE, 0x00);           /* Sleep (FSK) */
    rf_delay(500);
    rf_write_reg(REG_OPMODE, OPMODE_LONGRANGE | OPMODE_SLEEP);
    rf_delay(500);
    rf_set_opmode(OPMODE_STDBY);

    /* 鎸?UART3_RF_* 閰嶇疆杞芥尝/璋冨埗/鍔熺巼/鍓嶅/鍚屾/LNA + 杩?RXCONT */
    rf_apply_config();

    rf_write_reg(REG_PAYLOAD_LENGTH, 255);
    rf_write_reg(REG_MAX_PAYLOAD_LENGTH, 255);
    rf_write_reg(REG_HOP_PERIOD, 0x00);

    /* 鎺ユ敹鏈烘娴嬮厤缃?(瀹樻柟 SX1276SetModem): TEST36=0x03 (闈?00k, ERRATA 2.1) */
    rf_write_reg(REG_TEST36, 0x03);
    rf_delay(500);

    RF_ENABLED = 1;   /* 鍏佽 TIM4 杞鎺ョ鏀跺彂 */
}

/* ==================== 鍙戦€?==================== */
void rf_send(const uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint16_t w;

    rf_set_opmode(OPMODE_STDBY);
    rf_write_reg(REG_IRQ_FLAGS_MASK, 0x17);   /* 鍏佽 TxDone, 鍚﹀垯绛変笉鍒?浼氭閿? */
    rf_write_reg(REG_IRQ_FLAGS, 0xFF);        /* 娓呬腑鏂爣蹇?*/

    /* 鍐?FIFO (鐢?0x80, 閬垮厤涓?RX 鍩哄潃 0x00 閲嶅彔) */
    rf_write_reg(REG_FIFO_TX_BASE, 0x80);
    rf_write_reg(REG_FIFO_ADDR_PTR, 0x80);
    for (i = 0; i < len; i++)
        rf_write_reg(REG_FIFO, buf[i]);
    rf_write_reg(REG_PAYLOAD_LENGTH, len);

    /* 杩涘叆 TX, 闃诲绛?TxDone (甯﹁秴鏃堕槻姝婚攣) */
    rf_set_opmode(OPMODE_TX);
    for (w = 0; w < 500; w++) {
        if (rf_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE) break;
        rf_delay(500);
    }
    rf_write_reg(REG_IRQ_FLAGS, 0xFF);
    rf_rx_start();       /* 鍙戝畬鍥炴帴鏀?(鍚?I/Q/ERRATA/mask 閰嶇疆) */
}

/* ==================== 闈為樆濉炴敹鍙戠姸鎬?==================== */
volatile uint8_t RF_TX_BUSY = 0;      /* 鍙戦€佸繖 */
volatile uint8_t RF_TX_DONE = 0;      /* 鍙戦€佸畬鎴愭爣蹇?*/
volatile uint16_t RF_TX_START_MS = 0; /* 鍙戦€佸紑濮嬫椂鍒?(TICK_MS, 瓒呮椂鍏滃簳鐢? */

/* ---- 鐜舰鎺ユ敹缂撳啿: ISR 蹇€熸惉鍏ュ師濮嬪抚, 涓诲惊鐜В鏋?(闃叉孩鍑? ---- */
typedef struct {
    uint8_t data[RF_RX_MAX];   /* 鍘熷甯?*/
    uint8_t len;
    int8_t  rssi;              /* 璇ュ抚 RSSI (dBm) */
    int8_t  snr;               /* 璇ュ抚 SNR (dB) */
} RF_QITEM;
static volatile RF_QITEM RF_RX_Q[RF_RX_QUEUE];
static volatile uint8_t  RF_RX_Q_WR = 0;   /* 鍐欐寚閽?(ISR) */
static volatile uint8_t  RF_RX_Q_RD = 0;   /* 璇绘寚閽?(涓诲惊鐜? */
volatile uint8_t RF_RX_OVF = 0;            /* 婧㈠嚭鏍囧織: 涓诲惊鐜寒 SYS 鐏?*/
volatile int8_t  RF_LAST_RSSI = -127;      /* 鏈€杩戜竴甯?RSSI */
volatile int8_t  RF_LAST_SNR  = 0;         /* 鏈€杩戜竴甯?SNR */

/* 闈為樆濉炲彂閫? 鎻愪氦涓€甯у悗绔嬪嵆杩斿洖; TxDone 鐢?TIM4 杞娓呴櫎蹇欑姸鎬?*/
uint8_t rf_tx_start(const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if (RF_TX_BUSY) return 1;         /* 蹇? 鎷掔粷鏂颁换鍔?(涓婂眰鎹浜?SYSTEM_LED) */

    RF_SPI_BUSY++;                  /* 闀夸簨鍔″崰鐢?SPI (璁℃暟), TIM4 杞璁╄矾 */
    rf_set_opmode(OPMODE_STDBY);
    if (UART3_RF_RADIO) {
        /* FSK: 发送前切 Sleep 复位 FIFO 写指针 (FSK 无 FIFO 地址指针寄存器,
         * 必须从 FIFO 起始写, 否则残留写指针导致发送读错位置 -> PacketSent 不置位) */
        rf_write_reg(REG_OPMODE, 0x00);
        rf_delay(200);
        rf_set_opmode(OPMODE_STDBY);
        rf_write_reg(REG_IRQFLAGS1, 0xFF);
        rf_write_reg(REG_IRQFLAGS2, 0xFF);
        /* FSK 可变包: 先写 1 字节长度前缀到 FIFO, 再写数据 (官方 Semtech 要求) */
        rf_write_reg(REG_FIFO, len);
        for (i = 0; i < len; i++)
            rf_write_reg(REG_FIFO, buf[i]);
        rf_write_reg(REG_PAYLOADLEN_FSK, 0xFF);  /* 可变包: 设最大长度, 硬件自动加长度字段 */
    } else {
        rf_write_reg(REG_IRQ_FLAGS_MASK, 0x17);   /* 鍏抽敭: 鍏佽 TxDone, 鍚﹀垯 rf_poll 绛変笉鍒?*/
        rf_write_reg(REG_IRQ_FLAGS, 0xFF);  /* 娓呬腑鏂爣蹇?*/
        rf_write_reg(REG_FIFO_TX_BASE, 0x80);
        rf_write_reg(REG_FIFO_ADDR_PTR, 0x80);
        for (i = 0; i < len; i++)
            rf_write_reg(REG_FIFO, buf[i]);
        rf_write_reg(REG_PAYLOAD_LENGTH, len);
    }
    rf_set_opmode(OPMODE_TX);         /* 杩涘叆 TX (姝ゅ悗 TxDone 鐢?TIM4 杞鏀跺熬) */
    RF_SPI_BUSY--;                    /* 閲婃斁 (鍐呴儴鍚勬 spi_begin/end 宸?++/--) */

    RF_TX_BUSY = 1;
    RF_TX_DONE = 0;
    RF_TX_START_MS = TICK_MS;
    return 0;
}

/* ==================== 鍙戦€佽秴鏃跺厹搴?====================
 * TxDone 闀挎椂闂存湭妫€娴嬪埌鏃跺己鍒跺洖鎺ユ敹, 闃?RF_TX_BUSY 鍗℃ (SYS 鐏?鍚庣画鍙戦€?鎺ユ敹鍏ㄥ牭)
 * 鐢变富寰幆 rf_app_poll 妫€娴?500ms 瓒呮椂鍚庤皟鐢? */
void rf_abort_tx(void)
{
    RF_TX_BUSY = 0;
    if (UART3_RF_RADIO) rf_fsk_rx_start();
    else                rf_rx_start();
}

/* ==================== TIM4 ISR 杞 ====================
 * 鐢?timer.c 鐨?TIM4 涓柇姣?167us 璋冪敤涓€娆?
 *   骞虫椂鍙 DIO0 鐢靛钩, 鏃犱簨浠跺垯绔嬪嵆杩斿洖 (寮€閿€鏋佸皬);
 *   鏈変簨浠舵墠鍋?SPI 璇?+ 鏀跺寘鍏ョ紦鍐?+ 娓呭繖銆? * 鍗婂弻宸? 澶勭悊瀹屽洖 RXCONT (鎺ユ敹甯告€?銆? */
void rf_poll(void)
{
    uint8_t flags;



    /* SPI 琚富寰幆鍗犵敤(RF 鍙戦€?DAC)鏃惰璺? 鎺ユ敹鏁版嵁鐣欒姱鐗囧唴 FIFO(RxDone 淇濇寔),
     * 鍙戦€?SPI 浜嬪姟鍏堝畬鎴? 涔嬪悗 SPI 绌洪棽鍐嶆惉 -> 鏃跺簭涓嶄贡 + 涓嶄涪鏁版嵁 */
    if (!RF_ENABLED || RF_SPI_BUSY) return;

    /* 鍙戦€佸繖: 鐩存帴杞鏌?TxDone/PacketSent (涓嶄緷璧?DIO0 鐨?TX 妯″紡鏄犲皠,
     * 瀹炴祴 DIO0 鍙戦€佸畬鎴愬悗涓嶆媺楂?-> 鍗?RF_TX_BUSY + 鍗?TX 妯″紡涓嶆帴鏀? */
    if (RF_TX_BUSY) {
        if (UART3_RF_RADIO) {
            flags = rf_read_reg(REG_IRQFLAGS2);      /* FSK: PacketSent 鍦?IRQFLAGS2 */
            if (flags & FSK_IRQ2_PACKETSENT) {       /* 鍙戦€佸畬鎴?*/
                rf_write_reg(REG_IRQFLAGS1, 0xFF);
                rf_write_reg(REG_IRQFLAGS2, 0xFF);
                RF_TX_BUSY = 0;
                RF_TX_DONE = 1;
                rf_fsk_rx_start();                   /* 鍙戝畬鍥炴帴鏀?*/
            }
        } else {
            flags = rf_read_reg(REG_IRQ_FLAGS);
            if (flags & IRQ_TX_DONE) {               /* 鍙戦€佸畬鎴?*/
                rf_write_reg(REG_IRQ_FLAGS, 0xFF);
                RF_TX_BUSY = 0;
                RF_TX_DONE = 1;
                rf_rx_start();                       /* 鍙戝畬鍥炴帴鏀?鍚?I/Q/ERRATA/mask 閰嶇疆) */
            }
        }
        return;
    }

    if (!(GPIOH->IDR & 0x20)) return;         /* 闈炲彂閫? DIO0(PH5) 浣庢棤浜嬩欢 */

    if (UART3_RF_RADIO) {
        /* ---- FSK 鏀跺寘 (IRQFLAGS2 PayloadReady) ---- */
        flags = rf_read_reg(REG_IRQFLAGS2);
        if (flags & FSK_IRQ2_PAYLOADREADY) {
            uint8_t n, i;
            if ((rf_read_reg(REG_IRQFLAGS2) & FSK_IRQ2_CRCOK)) {
                n = rf_read_reg(REG_FIFO);   /* FSK 可变包: FIFO 首字节=实际长度 (官方: ReadFifo(&Size,1)) */
                if (n > RF_RX_MAX) n = RF_RX_MAX;
                {
                    uint8_t nxt = (uint8_t)(RF_RX_Q_WR + 1) % RF_RX_QUEUE;
                    if (nxt == RF_RX_Q_RD) {
                        RF_RX_OVF = 1;                 /* 鐜舰缂撳啿婊? 婧㈠嚭 */
                    } else {
                        RF_RX_Q[RF_RX_Q_WR].len = n;
                        for (i = 0; i < n; i++)
                            RF_RX_Q[RF_RX_Q_WR].data[i] = rf_read_reg(REG_FIFO);
                        RF_RX_Q[RF_RX_Q_WR].rssi = (int8_t)(-((int16_t)rf_read_reg(REG_RSSIVALUE) >> 1));
                        RF_RX_Q[RF_RX_Q_WR].snr  = 0;  /* FSK 鏃?SNR */
                        RF_RX_Q_WR = nxt;
                    }
                }
            } else {
                RF_APP_CRC_CNT++;                      /* 缁熻: CRC 閿欏抚 */
            }
            rf_write_reg(REG_IRQFLAGS1, 0xFF);
            rf_write_reg(REG_IRQFLAGS2, 0xFF);
            rf_fsk_rx_start();                         /* 鍥炴帴鏀?*/
        }
        return;
    }

    flags = rf_read_reg(REG_IRQ_FLAGS);

    if (flags & IRQ_RX_DONE) {                /* 鏀跺埌涓€鍖? 蹇€熸惉鍏ョ幆褰㈢紦鍐?涓嶈В鏋? */
        uint8_t n, i;
        if (!(flags & IRQ_PAYLOAD_CRC_ERR)) {
            n = rf_read_reg(REG_RX_NB_BYTES);
            if (n > RF_RX_MAX) n = RF_RX_MAX;
            rf_write_reg(REG_FIFO_ADDR_PTR, rf_read_reg(REG_FIFO_RX_BASE));
            {
                uint8_t nxt = (uint8_t)(RF_RX_Q_WR + 1) % RF_RX_QUEUE;
                if (nxt == RF_RX_Q_RD) {
                    RF_RX_OVF = 1;            /* 鐜舰缂撳啿婊? 婧㈠嚭, 涓诲惊鐜寒 SYS 鐏?*/
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
            RF_APP_CRC_CNT++;                 /* 缁熻: CRC 閿欏抚 */
        }
        rf_write_reg(REG_IRQ_FLAGS, 0xFF);
        rf_rx_start();                        /* 鍥炴帴鏀?鍚厤缃? */
    }
}

/* 涓诲惊鐜? 浠庣幆褰㈡帴鏀剁紦鍐插彇涓€甯? 鍙栧埌鍒欐竻婧㈠嚭鏍囧織(鎭㈠, 鐏孩鍑虹伅) */
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
        RF_RX_OVF = 0;                        /* 宸叉仮澶嶅鐞? 鐏孩鍑虹伅 */
        return 1;
    }
}

/* ==================== 鎺ユ敹 ==================== */
uint8_t rf_receive(uint8_t *buf, uint8_t *len, uint16_t timeout_ms)
{
    uint16_t t0 = TICK_MS;
    uint8_t n, i;

    rf_rx_start();   /* 瀹屾暣鎺ユ敹閰嶇疆 (I/Q/ERRATA/mask/FIFO) */

    for (;;) {
        uint8_t flags = rf_read_reg(REG_IRQ_FLAGS);
        if (flags & IRQ_RX_DONE) {
            if (flags & IRQ_PAYLOAD_CRC_ERR) {
                /* CRC 閿欒: 娓呮爣蹇楃户缁洃鍚?*/
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
        /* 瓒呮椂 (RTC 姣, 澶勭悊鍥炵粫) */
        if ((uint16_t)(TICK_MS - t0) >= timeout_ms) {
            rf_set_opmode(OPMODE_STDBY);
            return 0;
        }
    }
}
