/*
 * rf_app.c - RF 应用层实现
 *
 * 帧协议 (doc/frame.md): [定位头(7bit)+指示字(1bit)][地址][内容指示][数据...]
 * 4 模式 (doc/upperpc.md 0x18):
 *   1 同步发送(单工): 发送表[0] 内容指示 + 周期 采集成帧发送
 *   2 异步接收(单工): 静默接收, 解帧更新输出
 *   3 异步收发(半双工): 收到一帧应答一帧
 *   4 定时发送+异步接收: 遍历发送表周期发送
 * 接收: rf.c TIM4 轮询收原始帧 -> rf_app_rx_isr (ISR) 解包更新输出内存,
 *       主循环 rf_app_run 把输出内存同步到硬件(GPIO/SPI)。
 */
#include "rf_app.h"
#include "rf.h"
#include "uart3.h"
#include "dio.h"
#include "adc.h"
#include "dac.h"
#include "compress.h"
#include "timer.h"
#include <Arduino.h>
#include <stm8s.h>

/* ==================== 帧协议常量 (doc/frame.md) ==================== */
#define FRAME_HEAD       0b0011011   /* 定位头 7bit */
#define FRAME_IND_TELE   0           /* 负荷指示字: 遥测 */
#define FRAME_IND_485    1           /* 负荷指示字: RS-485 透传(暂未启用) */
#define FRAME_ADDR_BCAST 0xFF        /* 广播地址 */

/* 内容指示位 (bit7..0: DH DL A3 A2 A1 A0 字长 压缩) */
#define CI_DH       0x80
#define CI_DL       0x40
#define CI_A3       0x20
#define CI_A2       0x10
#define CI_A1       0x08
#define CI_A0       0x04
#define CI_WORDLEN  0x02
#define CI_COMPRESS 0x01

/* 收发模式 (doc/upperpc.md 0x18) */
#define RF_MODE_SYNC_TX    1   /* 同步发送(单工) */
#define RF_MODE_ASYNC_RX   2   /* 异步接收(单工) */
#define RF_MODE_ASYNC_TXRX 3   /* 异步收发(半双工) */
#define RF_MODE_TIMED_TX   4   /* 定时发送+异步接收 */

/* SYSTEM_LED = PH1 (高电平点亮) */
#define RF_LED_SYS_ON()  (GPIOH->ODR |= 0x02)
#define RF_LED_SYS_OFF() (GPIOH->ODR &= (uint8_t)~0x02)

/* 模式3 主从: 0=从机 1=主机 (doc/upperpc.md 0x19) */
#define RF_ROLE_SLAVE  0
#define RF_ROLE_MASTER 1
/* 主机应答模式无收包超时: 超过则回到主动轮询 (ms) */
#define RF_M3_TIMEOUT_MS 2000

/* ==================== 全局状态 ==================== */
volatile uint8_t RF_APP_OVERFLOW = 0;   /* 任务堆叠计数 (非0 时 SYSTEM_LED 亮) */
static volatile uint8_t RF_OUT_UPDATED = 0;  /* ISR 解包已更新输出内存 */
static volatile uint8_t RF_ACK_PENDING = 0;  /* 异步收发: 收到帧待应答 */

/* 模式3 主机状态机 */
static uint8_t  RF_M3_STATE;        /* 0=主动轮询 1=应答模式 */
static uint16_t RF_M3_LAST_TX_MS;   /* 主机最近主动发帧时刻 (TICK_MS) */
static uint16_t RF_M3_LAST_RX_MS;   /* 主机最近收到帧时刻 (TICK_MS) */

/* 各发送任务上次发射时刻(ms): 模式1 用 [0], 模式4 用全表 */
static uint16_t RF_TX_LAST[32];

/* 位流累加器: 打包(发送,主循环) / 解包(接收,ISR) 相互独立 */
static uint32_t RF_PK_ACC;   static uint8_t RF_PK_BITS;
static uint32_t RF_UNPK_ACC; static uint8_t RF_UNPK_BITS;

/* ==================== 位流打包 (MSB 优先) ==================== */
static void rf_bitw(uint16_t v, uint8_t n, uint8_t *out, uint8_t *idx)
{
    while (n--) {
        RF_PK_ACC = (RF_PK_ACC << 1) | ((v >> n) & 1);
        if (++RF_PK_BITS == 8) {
            out[(*idx)++] = (uint8_t)RF_PK_ACC;
            RF_PK_ACC = 0; RF_PK_BITS = 0;
        }
    }
}

/* ==================== 位流解包 (MSB 优先, 读 10bit) ==================== */
static uint16_t rf_bitr(const uint8_t *buf, uint8_t len, uint8_t *idx)
{
    uint16_t v = 0;
    uint8_t i;
    for (i = 0; i < 10; i++) {
        if (RF_UNPK_BITS == 0) {
            if (*idx >= len) return 0;
            RF_UNPK_ACC = buf[(*idx)++];
            RF_UNPK_BITS = 8;
        }
        v = (uint16_t)((v << 1) | ((RF_UNPK_ACC >> (RF_UNPK_BITS - 1)) & 1));
        RF_UNPK_BITS--;
    }
    return v;
}

/* ==================== 采集并打包模拟量 ====================
 * (字长0,压缩0)=8bit截断(高8位); (字长0,压缩1)=A-law; (字长1,压缩0)=10bit位流
 */
static void rf_pack_analog(uint8_t ci, uint8_t *out, uint8_t *idx)
{
    uint8_t word = (ci & CI_WORDLEN) ? 1 : 0;
    uint8_t comp = (ci & CI_COMPRESS) ? 1 : 0;
    uint16_t a[4];
    uint8_t n = 0, i;

    if (ci & CI_A3) { read_adc_ai3(); a[n++] = (uint16_t)((PORT_AI3_H << 2) | (PORT_AI3_L >> 6)); }
    if (ci & CI_A2) { read_adc_ai2(); a[n++] = (uint16_t)((PORT_AI2_H << 2) | (PORT_AI2_L >> 6)); }
    if (ci & CI_A1) { read_adc_ai1(); a[n++] = (uint16_t)((PORT_AI1_H << 2) | (PORT_AI1_L >> 6)); }
    if (ci & CI_A0) { read_adc_ai0(); a[n++] = (uint16_t)((PORT_AI0_H << 2) | (PORT_AI0_L >> 6)); }

    if (!word) {
        for (i = 0; i < n; i++) {
            if (comp) out[(*idx)++] = a13_compress(a[i]);    /* A-law 8bit */
            else      out[(*idx)++] = (uint8_t)(a[i] >> 2);  /* 截断: 高8位 */
        }
    } else {
        /* 10bit 高端对齐位流 (N 路 -> 16/24/32/40 bit, 低位补 0) */
        RF_PK_ACC = 0; RF_PK_BITS = 0;
        for (i = 0; i < n; i++)
            rf_bitw(a[i], 10, out, idx);
        if (RF_PK_BITS) {
            out[(*idx)++] = (uint8_t)(RF_PK_ACC << (8 - RF_PK_BITS));
            RF_PK_ACC = 0; RF_PK_BITS = 0;
        }
    }
}

/* ==================== 成帧 (遥测) ==================== */
static uint8_t rf_build_frame(uint8_t ci, uint8_t dst, uint8_t *frame)
{
    uint8_t idx = 3;
    frame[0] = (uint8_t)((FRAME_HEAD << 1) | FRAME_IND_TELE);
    frame[1] = dst;
    frame[2] = ci;
    if (ci & CI_DH) { dio_read_di_high(); frame[idx++] = PORT_DI_H; }
    if (ci & CI_DL) { dio_read_di_low();  frame[idx++] = PORT_DI_L; }
    rf_pack_analog(ci, frame, &idx);
    return idx;
}

/* ==================== 发送一帧 (内容指示驱动) ====================
 * 提交非阻塞发送; 忙(上一帧未发完) -> 任务堆叠: 计数 + 亮 SYSTEM_LED
 */
static void rf_send_ci(uint8_t ci)
{
    uint8_t frame[16];
    uint8_t len = rf_build_frame(ci, UART3_PEER_ADDR, frame);

    if (rf_tx_start(frame, len)) {
        RF_APP_OVERFLOW++;
        RF_LED_SYS_ON();
    }
}

/* ==================== ISR 解包 (rf.c TIM4 轮询调用) ====================
 * 校验定位头/指示字/地址 -> 解析内容指示 -> 更新 PORT_DO_* 与 AO_* 内存 -> 置更新标志
 * 只写内存(单字节原子), 硬件同步(GPIO/SPI)由主循环 rf_app_run 完成
 */
void rf_app_rx_isr(const uint8_t *buf, uint8_t len)
{
    uint8_t ci, idx = 3;
    uint8_t word, comp, n = 0, i;
    uint16_t a[4];
    uint8_t tgt[4];

    if (len < 3) return;
    if ((buf[0] >> 1) != FRAME_HEAD) return;          /* 定位头 */
    if ((buf[0] & 0x01) != FRAME_IND_TELE) return;    /* 暂只支持遥测(485 透传未启用) */
    if (buf[1] != UART3_SELF_ADDR && buf[1] != FRAME_ADDR_BCAST) return; /* 地址过滤 */

    ci = buf[2];
    word = (ci & CI_WORDLEN) ? 1 : 0;
    comp = (ci & CI_COMPRESS) ? 1 : 0;

    /* 数字量 DH/DL -> DO 内存 */
    if (ci & CI_DH) { if (idx >= len) return; PORT_DO_H = buf[idx++]; }
    if (ci & CI_DL) { if (idx >= len) return; PORT_DO_L = buf[idx++]; }

    /* 模拟量 (按位序 A3..A0 收集目标通道) */
    if (ci & CI_A3) tgt[n++] = 3;
    if (ci & CI_A2) tgt[n++] = 2;
    if (ci & CI_A1) tgt[n++] = 1;
    if (ci & CI_A0) tgt[n++] = 0;

    if (!word) {
        for (i = 0; i < n; i++) {
            if (idx >= len) return;
            a[i] = comp ? a13_decompress(buf[idx++])
                        : ((uint16_t)buf[idx++] << 2);   /* 8bit 截断恢复 */
        }
    } else {
        RF_UNPK_ACC = 0; RF_UNPK_BITS = 0;
        for (i = 0; i < n; i++)
            a[i] = rf_bitr(buf, len, &idx);              /* 10bit 位流 */
    }

    /* 写 DAC 输出内存 */
    for (i = 0; i < n; i++) {
        uint16_t v = a[i] & 0x3FF;
        switch (tgt[i]) {
        case 0: PORT_DO0_H = (uint8_t)(v >> 2); PORT_DO0_L = (uint8_t)((v << 6) & 0xFF); break;
        case 1: PORT_DO1_H = (uint8_t)(v >> 2); PORT_DO1_L = (uint8_t)((v << 6) & 0xFF); break;
        case 2: PORT_DO2_H = (uint8_t)(v >> 2); PORT_DO2_L = (uint8_t)((v << 6) & 0xFF); break;
        default:PORT_DO3_H = (uint8_t)(v >> 2); PORT_DO3_L = (uint8_t)((v << 6) & 0xFF); break;
        }
    }

    RF_OUT_UPDATED = 1;
    if (UART3_RF_MODE == RF_MODE_ASYNC_TXRX) {
        RF_ACK_PENDING = 1;   /* 异步收发: 请求应答 */
        if (UART3_RF_ROLE == RF_ROLE_MASTER) {
            RF_M3_STATE     = 1;            /* 主机收到第一帧 -> 进入应答模式 */
            RF_M3_LAST_RX_MS = TICK_MS;
        }
    }
}

/* ==================== 发送表周期换算 (128us -> ms) ==================== */
static uint16_t rf_period_ms(uint16_t pl, uint16_t ph)
{
    uint32_t p = ((uint32_t)ph << 16) | pl;
    uint32_t ms = (uint32_t)(((uint64_t)p * 128) / 1000);
    if (ms < 1) ms = 1;
    if (ms > 60000) ms = 60000;   /* 上限保护 */
    return (uint16_t)ms;
}

/* ==================== 主循环: 4 模式收发状态机 ==================== */
void rf_app_run(void)
{
    uint8_t mode = UART3_RF_MODE;
    uint16_t now = rtc_get_ms();
    uint8_t i;

    /* 1. 发送完成收尾: 清标志, 消一次堆叠灯 */
    if (RF_TX_DONE) {
        RF_TX_DONE = 0;
        if (RF_APP_OVERFLOW) RF_APP_OVERFLOW--;
        if (!RF_APP_OVERFLOW) RF_LED_SYS_OFF();
    }

    /* 2. 接收应用: ISR 已解包更新内存, 这里同步硬件 */
    if (RF_OUT_UPDATED) {
        RF_OUT_UPDATED = 0;
        dio_write_do();    /* PORT_DO_L/H -> GPIO */
        write_dac_all();   /* PORT_DO0~3_H/L -> AD5314 (SPI) */
    }

    switch (mode) {
    case RF_MODE_SYNC_TX:            /* 1 同步发送(单工): 发送表[0] 内容+周期 */
        if (UART3_TX_CONTENT[0] && !RF_TX_BUSY &&
            (uint16_t)(now - RF_TX_LAST[0]) >= rf_period_ms(UART3_TX_PERIOD_L[0],
                                                            UART3_TX_PERIOD_H[0])) {
            RF_TX_LAST[0] = now;
            rf_send_ci(UART3_TX_CONTENT[0]);
        }
        break;

    case RF_MODE_ASYNC_RX:           /* 2 异步接收(单工): 静默接收 */
        break;

    case RF_MODE_ASYNC_TXRX:         /* 3 异步收发(半双工): 主从应答 */
        if (UART3_RF_ROLE == RF_ROLE_SLAVE) {
            /* 从机: 收到一帧应答一帧 */
            if (RF_ACK_PENDING && !RF_TX_BUSY) {
                RF_ACK_PENDING = 0;
                rf_send_ci(UART3_TX_CONTENT[0] ? UART3_TX_CONTENT[0]
                                               : (CI_DH | CI_DL));
            }
        } else {
            /* 主机: 先主动轮询; 收到第一帧后进入应答模式; 超时回到主动轮询 */
            if (RF_M3_STATE == 0) {
                if (!RF_TX_BUSY &&
                    (uint16_t)(TICK_MS - RF_M3_LAST_TX_MS) >=
                        rf_period_ms(UART3_TX_PERIOD_L[0], UART3_TX_PERIOD_H[0])) {
                    RF_M3_LAST_TX_MS = TICK_MS;
                    rf_send_ci(UART3_TX_CONTENT[0] ? UART3_TX_CONTENT[0]
                                                   : (CI_DH | CI_DL));
                }
            } else {
                /* 应答模式: 收到即答 */
                if (RF_ACK_PENDING && !RF_TX_BUSY) {
                    RF_ACK_PENDING = 0;
                    rf_send_ci(UART3_TX_CONTENT[0] ? UART3_TX_CONTENT[0]
                                                   : (CI_DH | CI_DL));
                }
                /* 较长时间收不到 -> 重新主动发帧 */
                if ((uint16_t)(TICK_MS - RF_M3_LAST_RX_MS) >= RF_M3_TIMEOUT_MS) {
                    RF_M3_STATE = 0;
                    RF_M3_LAST_TX_MS = TICK_MS;
                }
            }
        }
        break;

    case RF_MODE_TIMED_TX:           /* 4 定时发送+异步接收: 遍历发送表 */
    default:
        for (i = 0; i < 32; i++) {
            if ((UART3_TX_ENA[i] & 0x01) && !RF_TX_BUSY &&
                (uint16_t)(now - RF_TX_LAST[i]) >= rf_period_ms(UART3_TX_PERIOD_L[i],
                                                                UART3_TX_PERIOD_H[i])) {
                RF_TX_LAST[i] = now;
                rf_send_ci(UART3_TX_CONTENT[i]);
            }
        }
        break;
    }
}

/* ==================== 初始化 ==================== */
void rf_app_init(void)
{
    uint8_t i;
    for (i = 0; i < 32; i++)
        RF_TX_LAST[i] = 0;
    RF_APP_OVERFLOW = 0;
    RF_OUT_UPDATED = 0;
    RF_ACK_PENDING = 0;
    RF_M3_STATE = 0;
    RF_M3_LAST_TX_MS = 0;
    RF_M3_LAST_RX_MS = 0;
    RF_PK_ACC = 0; RF_PK_BITS = 0;
    RF_UNPK_ACC = 0; RF_UNPK_BITS = 0;
}
