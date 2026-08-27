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
 *       (9600 波特率约 1.04ms/字节, TIM4 12kHz ISR 每 83us, 有足够余量)
 */
#include "uart1.h"
#include "uart3.h"
#include "rf.h"
#include "timer.h"
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

/* ==================== 主循环: 485 上行透传 ====================
 * 触发条件: 缓冲有数据 且 (缓冲满 或 距上次收字节 > 组帧超时)
 * 组 485 帧 (doc/frame.md): [地址][0x37][1 L][数据×L] -> rf_tx_start
 */
void uart1_poll(void)
{
    uint8_t fbuf[UART1_BUF_SIZE + 3];
    uint8_t i, len, bmax;
    uint16_t idle;

    if (!UART1_ENABLED) return;
    if (UART1_RX_LEN == 0) return;

    bmax = UART1_BUF_MAX;
    if (bmax == 0 || bmax > UART1_BUF_SIZE) bmax = 64;

    idle = (uint16_t)(rtc_get_ms() - UART1_RX_LAST_MS);
    if (!UART1_RX_FULL && idle < UART1_TIMEOUT) return;   /* 未满且未超时 */

    /* 原子取走缓冲 */
    __critical {
        len = UART1_RX_LEN;
        UART1_RX_LEN = 0;
        UART1_RX_FULL = 0;
        for (i = 0; i < len; i++) fbuf[i] = UART1_RX_BUF[i];
    }

    /* 组 485 帧: 数据后移 3 字节, 前插 [地址][0x37][1 L] */
    for (i = len; i > 0; i--) fbuf[i + 2] = fbuf[i - 1];
    fbuf[0] = UART3_PEER_ADDR;
    fbuf[1] = 0x37;                          /* 定位头|内容指示=1 */
    fbuf[2] = (uint8_t)(0x80 | (len & 0x7F));
    rf_tx_start(fbuf, (uint8_t)(len + 3));
}

/* ==================== 485 下行: 数据经 UART1 发出 ====================
 * 半双工: CTRL 置高(DE) -> 逐字节发 -> 等 TC -> CTRL 置低(RE#) 回接收
 */
void uart1_send(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    if (!UART1_ENABLED || len == 0) return;

    UART1_CTRL_TX();
    for (i = 0; i < len; i++)
        UART1_SendData8(data[i]);
    while (UART1_GetFlagStatus(UART1_FLAG_TC) == RESET);   /* 等发送完成 */
    UART1_CTRL_RX();
}
