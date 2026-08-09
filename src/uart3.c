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
#include <Arduino.h>
#include <stm8s.h>

/* ==================== 常量 ==================== */
#define UART3_BAUD       115200UL
#define UART3_TIMEOUT_MS 5        /* 帧字节间隔超时(ms), 超时回 IDLE */
#define UART3_HEAD_WRITE 0x37     /* 写入帧定位头 */
#define UART3_HEAD_READ  0x36     /* 读取帧定位头 */
#define UART3_MCU_ID_BASE 0x48CD  /* STM8S208 唯一ID基址 (RM0016), 12字节 */

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

/* ==================== 全局变量 (Page0) ==================== */
static volatile uint8_t  UART3_STATE;    /* 状态机 */
static volatile uint8_t  UART3_HEAD;     /* 定位头 */
static volatile uint8_t  UART3_ADDR;     /* 地址码 */
static volatile uint8_t  UART3_DL;       /* 数据低字节 */
static volatile uint8_t  UART3_DH;       /* 数据高字节 */
static volatile uint8_t  UART3_CRC;      /* CRC 累积 */
static volatile uint8_t  UART3_READ;     /* 1=读取帧, 0=写入帧 */
static volatile uint8_t  UART3_RX_OK;    /* 完整帧标志 */
static volatile uint16_t UART3_LAST_MS;  /* 最近收字节时刻 (超时用) */

/* 发射流程任务表 (32任务) */
static uint8_t  UART3_TX_CONTENT[32];
static uint8_t  UART3_TX_ENA[32];
static uint16_t UART3_TX_PERIOD_L[32];
static uint16_t UART3_TX_PERIOD_H[32];
static uint16_t UART3_TX_LAST[32];   /* 各任务上次发射时刻(ms) */

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
    }
    if (addr >= 0x40 && addr <= 0x7F)   /* 射频设置: SX1278 寄存器 */
        return rf_read_reg(addr & 0x3F);
    if (addr >= 0x80)                   /* 发射流程设置 */
        return uart3_tx_read(addr);
    return 0;
}

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
    }
    if (addr >= 0x40 && addr <= 0x7F) {
        rf_write_reg(addr & 0x3F, (uint8_t)val);
        return rf_read_reg(addr & 0x3F);
    }
    if (addr >= 0x80)
        return uart3_tx_write(addr, val);
    return 0;   /* 只读地址写入返回 0 */
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
    while (!(UART3->SR & 0x80));  /* 等 TXE */
    UART3->DR = head;
    while (!(UART3->SR & 0x80));
    UART3->DR = addr;
    while (!(UART3->SR & 0x80));
    UART3->DR = (uint8_t)(val & 0xFF);
    while (!(UART3->SR & 0x80));
    UART3->DR = (uint8_t)(val >> 8);
    while (!(UART3->SR & 0x80));
    UART3->DR = crc;
    while (!(UART3->SR & 0x80));
    UART3->DR = tail;
}

/* ==================== 初始化 ==================== */
void uart3_init(void)
{
    GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST); /* TX 推挽 */
    GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);      /* RX 浮空 */
    UART3_Init(UART3_BAUD, UART3_WORDLENGTH_8D, UART3_STOPBITS_1,
               UART3_PARITY_NO, UART3_MODE_TXRX_ENABLE);

    /* 使能 RXNE 中断 (接收状态机) */
    UART3_ITConfig(UART3_IT_RXNE, ENABLE);

    /* 状态机复位 */
    UART3_STATE = ST_IDLE;
    UART3_RX_OK = 0;
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

/* ==================== 发射流程调度 ====================
 * 遍历 32 任务: ENA.bit0=1 且到周期 -> rf_send 发送"帧内容指示字段"
 * 周期 = (H<<16|L) * 128us; 调度用 RTC 毫秒(1ms 精度)
 * 注: 当前发送内容为内容指示字段 1 字节, 待帧打包模块扩展完整帧
 */
void uart3_tx_run(void)
{
    uint8_t i;
    uint16_t now = rtc_get_ms();

    for (i = 0; i < 32; i++) {
        if (UART3_TX_ENA[i] & 0x01) {
            uint32_t period = ((uint32_t)UART3_TX_PERIOD_H[i] << 16)
                            | UART3_TX_PERIOD_L[i];
            uint32_t ms = (uint32_t)(((uint64_t)period * 128) / 1000);
            if (ms < 1) ms = 1;
            if ((uint16_t)(now - UART3_TX_LAST[i]) >= ms) {
                UART3_TX_LAST[i] = now;
                rf_send(&UART3_TX_CONTENT[i], 1);
            }
        }
    }
}

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

    if (UART3_READ)
        val = uart3_read_addr(addr);
    else
        val = uart3_write_addr(addr, val);

    uart3_reply(head, addr, val);
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
