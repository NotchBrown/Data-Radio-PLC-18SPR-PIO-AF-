/*
 * uart1.c - UART1 RS-485 透传驱动
 *
 * 硬件: ISO3082DWR (半双工 RS-485 隔离收发)
 *   PA5 = UART1_TX (D), PA4 = UART1_RX (R), PA3 = UART1_CTRL (DE+RE#)
 *   CTRL 高 = DE 使能(发送), 低 = RE# 使能(接收)
 *
 * 收发模型 (doc/upperpc.md 0x1A~0x1D 配置):
 *   上行: UART1 RX 中断收字节 -> 缓冲 -> 主循环 uart1_poll 组 485 帧发 RF
 *         (触发: 缓冲满 UART1_BUF_MAX 或距上次收字节超 UART1_TIMEOUT ms)
 *   下行: 收到 RF 485 帧 -> uart1_send 经 UART1 发出 (CTRL 方向控制)
 *
 * 注意: STM8S UART 无硬件 FIFO, RX 中断必须尽快读 DR, 否则下一字节覆盖
 *       (9600 波特率约 1.04ms/字节, TIM4 6kHz ISR 每 167us, 有足够余量)
 */
#include "uart1.h"
#include "uart3.h"
#include "rf.h"
#include "timer.h"
#include "dbg.h"
#include <Arduino.h>
#include <stm8s.h>

/* CTRL = PA3 (bit2), 高=发送(DE), 低=接收(RE#) */
#define UART1_CTRL_TX()  (GPIOA->ODR |= 0x08)
#define UART1_CTRL_RX()  (GPIOA->ODR &= (uint8_t)~0x08)

/* 485 接收缓冲 (ISR 写入, 主循环取出) */
static volatile uint8_t  UART1_RX_BUF[UART1_BUF_SIZE];
static volatile uint8_t  UART1_RX_LEN = 0;     /* 缓冲中字节数 */
static volatile uint8_t  UART1_RX_FULL = 0;    /* 缓冲满标志 */
static volatile uint16_t UART1_RX_LAST_MS = 0; /* 上次收字节时刻 (RTC_MS) */
static volatile uint8_t  UART1_ENABLED = 0;    /* 485 透传运行中 */
/* 上次发 485 时刻 (TICK_MS; 供 rf_app 判断 485 空闲超时, 三条件条件2) */
volatile uint16_t UART1_LAST_TX_MS = 0;

/* ==================== 初始化 ==================== */
void uart1_init(void)
{
    GPIO_Init(GPIOA, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST); /* TX 推挽 */
    GPIO_Init(GPIOA, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);      /* RX 浮空 */
    GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST); /* CTRL 推挽 */
    UART1_CTRL_RX();   /* 默认接收 */

    UART1_Init(UART1_BAUD, UART1_WORDLENGTH_8D, UART1_STOPBITS_1,
               UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE,
               UART1_MODE_TXRX_ENABLE);
    UART1_Cmd(ENABLE);
    /* RXNE 中断由 uart1_enable 控制 */

    UART1_ENABLED = 0;
    UART1_RX_LEN = 0;
    UART1_RX_FULL = 0;
    UART1_RX_LAST_MS = 0;
}

/* ==================== 使能/禁用 ==================== */
void uart1_enable(uint8_t on)
{
    UART1_ENABLED = on;
    if (on) {
        UART1_RX_LEN = 0;
        UART1_RX_FULL = 0;
        UART1_CTRL_RX();
        UART1_ITConfig(UART1_IT_RXNE, ENABLE);
        UART1_Cmd(ENABLE);
    } else {
        UART1_ITConfig(UART1_IT_RXNE, DISABLE);
        UART1_Cmd(DISABLE);
    }
}

/* ==================== UART1 接收中断 (向量18) ====================
 * 短小: 读 DR -> 存缓冲 -> 更新计数/时刻; 组帧/发送放主循环
 * (STM8S UART 无 FIFO, 必须尽快读 DR 否则下一字节覆盖)
 */
void UART1_RX_IRQHandler(void) __interrupt(ITC_IRQ_UART1_RX)
{
    uint8_t b = UART1_ReceiveData8();   /* 读 DR 清 RXNE */

    if (!UART1_ENABLED) return;

    if (UART1_RX_LEN < UART1_BUF_SIZE) {
        UART1_RX_BUF[UART1_RX_LEN++] = b;
        if (UART1_RX_LEN >= UART1_BUF_MAX) UART1_RX_FULL = 1;
    }
    UART1_RX_LAST_MS = rtc_get_ms();
}

/* ==================== UART1 发送中断 (向量17) ====================
 * 未使用 (485 发送为轮询 + CTRL 方向控制), 空实现满足中断向量表
 */
void UART1_TX_IRQHandler(void) __interrupt(ITC_IRQ_UART1_TX)
{
}

/* ==================== 485 上行: 由 rf_app 主从调度发 (三条件) ====================
 * 提供: 是否有待发 / 缓冲是否满 / 取走数据; 组帧+RF发送由 rf_app 统一调度 */
uint8_t uart1_has_frame(void)
{
    uint16_t idle;
    if (!UART1_ENABLED) return 0;
    if (UART1_RX_LEN == 0) return 0;
    if (UART1_RX_FULL) return 1;                       /* 缓冲满 */
    idle = (uint16_t)(rtc_get_ms() - UART1_RX_LAST_MS);
    if (idle >= UART1_TIMEOUT) return 1;               /* 组帧超时 */
    return 0;
}

uint8_t uart1_is_full(void)
{
    return (UART1_ENABLED && UART1_RX_FULL) ? 1 : 0;   /* 条件1: 缓冲满优先发 */
}

uint8_t uart1_take_frame(uint8_t *buf)
{
    uint8_t i, len;
    if (!UART1_ENABLED) return 0;
    if (UART1_RX_LEN == 0) return 0;
    __critical {
        len = UART1_RX_LEN;
        UART1_RX_LEN = 0;
        UART1_RX_FULL = 0;
        for (i = 0; i < len; i++) buf[i] = UART1_RX_BUF[i];
    }
    return len;
}

/* ==================== 485 下行: 数据经 UART1 发出 ====================
 * 半双工: CTRL 置高(DE) -> 逐字节发 -> 等 TC -> CTRL 置低(RE#) 回接收
 */
void uart1_send(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint16_t guard = 0;
    if (!UART1_ENABLED || len == 0) return;

    UART1_CTRL_TX();
    for (i = 0; i < len; i++) {
        guard = 0;
        while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET) {   /* 等 TXE */
            if (++guard > 2000) { UART1_CTRL_RX(); return; }
        }
        UART1_SendData8(data[i]);
    }
    guard = 0;
    while (UART1_GetFlagStatus(UART1_FLAG_TC) == RESET) {   /* 等发送完成 */
        if (++guard > 2000) break;
    }
    UART1_CTRL_RX();
}

/* ==================== 485 接收缓冲长度 (快照/诊断 0x2E) ==================== */
uint8_t uart1_get_len(void)
{
    return UART1_RX_LEN;
}
