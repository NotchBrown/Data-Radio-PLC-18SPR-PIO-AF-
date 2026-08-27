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
#include "timer.h"
#include <Arduino.h>
#include <stm8s.h>

/* ==================== 调制参数 (待确认) ==================== */
#define RF_FREQ_HZ      470000000UL  /* 载波频率 Hz */
#define RF_SF           7            /* 扩频因子 6..12 */
#define RF_BW           125          /* 带宽 kHz: 125/250/500 */
#define RF_CR           5            /* 编码率 5..8 (= 4/5..4/8) */
#define RF_POWER_DBM    13           /* 发射功率 dBm */

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
#define REG_IRQ_FLAGS       0x12
#define REG_RX_NB_BYTES     0x13
#define REG_MODEM_CONFIG_1  0x1D
#define REG_MODEM_CONFIG_2  0x1E
#define REG_MODEM_CONFIG_3  0x1F
#define REG_PAYLOAD_LENGTH  0x22
#define REG_VERSION         0x42

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
static volatile uint8_t RF_SPI_BUSY = 0; /* 主循环用 SPI 期间置位, TIM4 轮询让路 */

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

/* 设置载波频率 (Frf = f*2^19/32MHz) */
void rf_set_frequency(uint32_t freq_hz)
{
    uint32_t frf = (uint32_t)(((uint64_t)freq_hz << 19) / 32000000UL);
    rf_write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF);
    rf_write_reg(REG_FRF_MID, (frf >> 8) & 0xFF);
    rf_write_reg(REG_FRF_LSB, frf & 0xFF);
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

    /* 进入 Sleep 并切到 LoRa 模式 */
    rf_write_reg(REG_OPMODE, OPMODE_LONGRANGE | OPMODE_SLEEP);
    rf_set_opmode(OPMODE_STDBY);

    /* 载波频率 */
    rf_set_frequency(RF_FREQ_HZ);

    /* 调制参数 (与上方宏对应, 改宏时同步改这里):
     * MC1: BW125=0x70 | CR4/5=0x02 | 显式头 = 0x72
     * MC2: SF7=0x70 | RX CRC 使能=0x04 = 0x74
     * MC3: AGC 自动 = 0x04
     */
    rf_write_reg(REG_MODEM_CONFIG_1, 0x72);
    rf_write_reg(REG_MODEM_CONFIG_2, 0x74);
    rf_write_reg(REG_MODEM_CONFIG_3, 0x04);

    /* 发射功率: PA_BOOST(bit7) | OutputPower(13dBm=0x0B) */
    rf_write_reg(REG_PA_CONFIG, 0x80 | 0x0B);

    /* LNA: 增益默认 + 开启 boost */
    rf_write_reg(REG_LNA, 0x23);

    RF_ENABLED = 1;   /* 允许 TIM4 轮询接管收发 */
}

/* ==================== 发送 ==================== */
void rf_send(const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    rf_set_opmode(OPMODE_STDBY);
    rf_write_reg(REG_IRQ_FLAGS, 0xFF);          /* 清中断标志 */

    /* 写 FIFO */
    rf_write_reg(REG_FIFO_TX_BASE, 0x00);
    rf_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    for (i = 0; i < len; i++)
        rf_write_reg(REG_FIFO, buf[i]);
    rf_write_reg(REG_PAYLOAD_LENGTH, len);

    /* 进入 TX, 阻塞等 TxDone */
    rf_set_opmode(OPMODE_TX);
    while (!(rf_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE));
    rf_write_reg(REG_IRQ_FLAGS, 0xFF);
    rf_set_opmode(OPMODE_STDBY);
}

/* ==================== 非阻塞收发状态 ==================== */
volatile uint8_t RF_TX_BUSY = 0;      /* 发送忙 */
volatile uint8_t RF_TX_DONE = 0;      /* 发送完成标志 */
volatile uint8_t RF_RX_FLAG = 0;      /* 收到一包标志 */
volatile uint8_t RF_RX_LEN  = 0;      /* 收到的长度 */
volatile uint8_t RF_RX_BUF[RF_RX_MAX];/* 接收缓冲 */

/* 非阻塞发送: 提交一帧后立即返回; TxDone 由 TIM4 轮询清除忙状态 */
uint8_t rf_tx_start(const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if (RF_TX_BUSY) return 1;         /* 忙: 拒绝新任务 (上层据此亮 SYSTEM_LED) */

    RF_SPI_BUSY = 1;                  /* 占用 SPI, TIM4 轮询让路 */
    rf_set_opmode(OPMODE_STDBY);
    rf_write_reg(REG_IRQ_FLAGS, 0xFF);  /* 清中断标志 */
    rf_write_reg(REG_FIFO_TX_BASE, 0x00);
    rf_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    for (i = 0; i < len; i++)
        rf_write_reg(REG_FIFO, buf[i]);
    rf_write_reg(REG_PAYLOAD_LENGTH, len);
    rf_set_opmode(OPMODE_TX);         /* 进入 TX (此后 TxDone 由 TIM4 轮询收尾) */
    RF_SPI_BUSY = 0;

    RF_TX_BUSY = 1;
    RF_TX_DONE = 0;
    return 0;
}

/* ==================== TIM4 ISR 轮询 ====================
 * 由 timer.c 的 TIM4 中断每 83us 调用一次:
 *   平时只读 DIO0 电平, 无事件则立即返回 (开销极小);
 *   有事件才做 SPI 读 + 收包入缓冲 + 清忙。
 * 半双工: 处理完回 RXCONT (接收常态)。
 */
void rf_poll(void)
{
    uint8_t flags;

    if (!RF_ENABLED || RF_SPI_BUSY) return;   /* 未初始化或主循环在用 SPI */
    if (!(GPIOH->IDR & 0x20)) return;         /* DIO0(PH5) 低: 无事件 */

    flags = rf_read_reg(REG_IRQ_FLAGS);

    if (flags & IRQ_TX_DONE) {                /* 发送完成 */
        rf_write_reg(REG_IRQ_FLAGS, 0xFF);
        RF_TX_BUSY = 0;
        RF_TX_DONE = 1;
    }
    if (flags & IRQ_RX_DONE) {                /* 收到一包 */
        uint8_t n, i;
        if (!(flags & IRQ_PAYLOAD_CRC_ERR)) {
            n = rf_read_reg(REG_RX_NB_BYTES);
            if (n > RF_RX_MAX) n = RF_RX_MAX;
            rf_write_reg(REG_FIFO_ADDR_PTR, rf_read_reg(REG_FIFO_RX_BASE));
            for (i = 0; i < n; i++)
                RF_RX_BUF[i] = rf_read_reg(REG_FIFO);
            RF_RX_LEN  = n;
            RF_RX_FLAG = 1;
            rf_app_rx_isr(RF_RX_BUF, n);      /* ISR 内解包, 更新输出内存 */
        }
        rf_write_reg(REG_IRQ_FLAGS, 0xFF);
    }

    rf_set_opmode(OPMODE_RXCONT);             /* 回接收常态 */
}

/* ==================== 接收 ==================== */
uint8_t rf_receive(uint8_t *buf, uint8_t *len, uint16_t timeout_ms)
{
    uint16_t t0 = rtc_get_ms();
    uint8_t n, i;

    rf_set_opmode(OPMODE_STDBY);
    rf_write_reg(REG_IRQ_FLAGS, 0xFF);
    rf_write_reg(REG_FIFO_RX_BASE, 0x00);
    rf_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    rf_set_opmode(OPMODE_RXCONT);

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
