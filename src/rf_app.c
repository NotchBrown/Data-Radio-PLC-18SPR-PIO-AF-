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
#include "uart1.h"
#include "dbg.h"
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

/* RF 命令码 (content=1, bit7=0, 广播无地址, doc/frame.md) */
#define RF_CMD_LINK_TEST  0x01       /* 链路探测/重连测试 */

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

/* 模式3 停等协议 (第一步宏默认值, 后续加 EEPROM 配置):
 * 主机主动发起, 回包驱动刷新; 等回包超时重发; 连续失败回握手 */
#define RF_M3_RETRY_TIMEOUT_MS 500   /* 等回包超时(重发间隔) ms */
#define RF_M3_RETRY_MAX        3     /* 连续重传上限, 超限回握手 */

/* 链路监控 (模式3): 掉线发 RF 探测命令重连 + SYS LED 报警 */
#define RF_LINK_CORRECT_MS 3000   /* 3s 无有效帧: 发一次 RF_CMD_LINK_TEST 探测 */
#define RF_LINK_DEAD_MS    5000   /* 5s 无有效帧: 链路断开, SYS LED 常亮报警 */

/* 通联状态机 (所有发射模式前置): 未通联先握手, 通联成功才传数据 */
#define RF_LINK_TRY_PERIOD_MS 200    /* 握手探测命令周期 (ms) */
#define RF_LINK_LOST_MS       3000   /* 已通联后超时无帧 -> 掉线重握手 (ms) */

/* ==================== 全局状态 ==================== */
volatile uint8_t RF_APP_OVERFLOW = 0;   /* 任务堆叠计数 (非0 时 SYSTEM_LED 亮) */
/* 收发统计 + 频偏校正请求 (控制指令 0x21/0x22/0x26) */
volatile uint16_t RF_APP_RX_CNT     = 0;   /* 收到有效帧计数 */
volatile uint16_t RF_APP_TX_CNT     = 0;   /* 发送帧计数 (含握手/重传) */
volatile uint16_t RF_APP_CRC_CNT    = 0;   /* CRC 错帧计数 (rf.c 递增) */
volatile uint8_t  RF_APP_FE_REQUEST = 0;   /* 频偏校正请求 (uart3 0x26 触发) */
static volatile uint8_t RF_OUT_UPDATED = 0;  /* ISR 解包已更新输出内存 */
static volatile uint8_t RF_ACK_PENDING = 0;  /* 异步收发: 收到帧待应答 */

/* 模式3 主机状态机 */
static uint8_t  RF_M3_STATE;        /* 主机: 0=待发 1=等回包 */
static uint16_t RF_M3_LAST_TX_MS;   /* 主机最近发帧时刻 (TICK_MS) */
static uint16_t RF_M3_LAST_RX_MS;   /* 主机最近收到帧时刻 (TICK_MS) */
static uint8_t  RF_M3_RETRY_CNT;    /* 主机连续重传计数 */
static volatile uint8_t RF_M3_RX_EVT = 0; /* 收到有效帧事件 (统一异步主从, 主循环处理) */
static uint8_t UART3_SLAVE_RET_CI = (CI_DH | CI_DL);  /* 从站回传内容指示 (主站要求 CI₂, 默认 DI) */
static uint8_t RF_M3_LAST_CI1 = (CI_DH | CI_DL);      /* 主机最近发送 CI₁ (重传用) */
static uint8_t RF_M3_LAST_CI2 = (CI_DH | CI_DL);      /* 主机最近发送 CI₂ (重传用) */

/* 链路状态 (模式3 掉线探测重连 + 报警) */
static volatile uint16_t RF_LINK_LAST_RX_MS = 0;  /* 上次收到有效帧时刻 (TICK_MS) */
static volatile uint8_t  RF_LINK_PROBED = 0;      /* 本次掉线已发过探测命令 */
static volatile uint8_t  RF_LINK_ALARM = 0;       /* 链路断开报警 (SYS LED 亮) */

/* 通联状态 (所有发射模式前置): 0=未通联 1=已通联 */
volatile uint8_t  RF_M3_LINK = 0;                 /* 通联标志 (rf_app_rx_isr 置 1) */
static volatile uint16_t RF_LINK_TRY_MS = 0;      /* 上次握手时刻 (TICK_MS) */

/* ==================== SYS 灯 (PH1) 统一刷新 ====================
 * 亮 = 任务堆叠(RF_APP_OVERFLOW) OR 环形溢出(RF_RX_OVF) OR 链路断开报警(RF_LINK_ALARM)
 */
static void rf_led_refresh(void)
{
    uint8_t on = (RF_APP_OVERFLOW || RF_RX_OVF || RF_LINK_ALARM) ? 1 : 0;
    if (on) RF_LED_SYS_ON(); else RF_LED_SYS_OFF();
    {   /* 仅状态变化时打印一次, 便于诊断 SYS 灯 */
        static uint8_t sys_led_last = 0xFF;
        if (on != sys_led_last) {
            sys_led_last = on;
            DBG_STR(on ? "[D]SYSon" : "[D]SYSoff");
            DBG_STR(" ovf="); DBG_DEC(RF_APP_OVERFLOW);
            DBG_STR(" rxovf="); DBG_DEC(RF_RX_OVF);
            DBG_STR(" alarm="); DBG_DEC(RF_LINK_ALARM);
            DBG_NL();
        }
    }
}

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

/* ==================== 成帧 (遥测) ====================
 * 主站→从站: [地址][0x36][CI₁][CI₂][数据按CI₁] (两个内容指示字节)
 * 从站→主站: [地址][0x36][CI][数据按CI]       (一个内容指示字节)
 * (doc/frame.md) */
static uint8_t rf_build_master(uint8_t ci1, uint8_t ci2, uint8_t dst, uint8_t *frame)
{
    uint8_t idx = 4;
    frame[0] = dst;   /* 地址 (超帧地址段, 帧前面) */
    frame[1] = (uint8_t)((FRAME_HEAD << 1) | FRAME_IND_TELE);
    frame[2] = ci1;   /* CI₁: 主站→从站内容 */
    frame[3] = ci2;   /* CI₂: 要求从站回传内容 */
    if (ci1 & CI_DH) { dio_read_di_high(); frame[idx++] = PORT_DI_H; }
    if (ci1 & CI_DL) { dio_read_di_low();  frame[idx++] = PORT_DI_L; }
    rf_pack_analog(ci1, frame, &idx);
    return idx;
}

static uint8_t rf_build_slave(uint8_t ci, uint8_t dst, uint8_t *frame)
{
    uint8_t idx = 3;
    frame[0] = dst;   /* 地址 (超帧地址段, 帧前面) */
    frame[1] = (uint8_t)((FRAME_HEAD << 1) | FRAME_IND_TELE);
    frame[2] = ci;
    if (ci & CI_DH) { dio_read_di_high(); frame[idx++] = PORT_DI_H; }
    if (ci & CI_DL) { dio_read_di_low();  frame[idx++] = PORT_DI_L; }
    rf_pack_analog(ci, frame, &idx);
    return idx;
}

/* ==================== 发送 ====================
 * 提交非阻塞发送; 忙(上一帧未发完) -> 任务堆叠: 计数 + 亮 SYSTEM_LED */
static void rf_send_master(uint8_t ci1, uint8_t ci2)
{
    uint8_t frame[16];
    uint8_t len = rf_build_master(ci1, ci2, UART3_PEER_ADDR, frame);
    DBG_POS(4);   /* 位置码: 发送遥测 */
    RF_APP_TX_CNT++;   /* 发送计数 */
    if (rf_tx_start(frame, len)) {
        RF_APP_OVERFLOW++;
        DBG_STR("[D]txbusy ovf="); DBG_DEC(RF_APP_OVERFLOW); DBG_NL();
        rf_led_refresh();   /* 堆叠 -> SYS 灯亮 */
    }
}

static void rf_send_slave(uint8_t ci)
{
    uint8_t frame[16];
    uint8_t len = rf_build_slave(ci, UART3_PEER_ADDR, frame);
    DBG_POS(4);   /* 位置码: 发送遥测 */
    RF_APP_TX_CNT++;   /* 发送计数 */
    if (rf_tx_start(frame, len)) {
        RF_APP_OVERFLOW++;
        DBG_STR("[D]txbusy ovf="); DBG_DEC(RF_APP_OVERFLOW); DBG_NL();
        rf_led_refresh();
    }
}

/* ==================== 发送测试帧 (控制指令 0x20) ==================== */
void rf_app_test_tx(void)
{
    rf_send_master(UART3_TX_CONTENT[0], UART3_TX_CI2[0]);   /* 任务0: CI₁ + CI₂ */
}

/* ==================== 发送 RF 命令帧 (content=1) ====================
 * 净负荷 = [地址] + 帧; 命令帧 = [地址][0b00110111][0 CCCCCCC], 3 字节
 * (doc/frame.md: 地址在帧前面, 帧本身无地址)
 */
static void rf_send_cmd(uint8_t cmd)
{
    uint8_t frame[3];
    frame[0] = UART3_PEER_ADDR;   /* 地址 (超帧地址段, 帧前面) */
    frame[1] = (uint8_t)((FRAME_HEAD << 1) | FRAME_IND_485);  /* 0x37 */
    frame[2] = (uint8_t)(cmd & 0x7F);
    RF_APP_TX_CNT++;   /* 握手也计数 */
    if (rf_tx_start(frame, 3)) {
        RF_APP_OVERFLOW++;
        rf_led_refresh();
    }
}

/* ==================== ISR 解包 (rf.c TIM4 轮询调用) ====================
 * 校验地址/定位头/指示字 -> 解析内容指示 -> 更新 PORT_DO_* 与 AO_* 内存 -> 置更新标志
 * 只写内存(单字节原子), 硬件同步(GPIO/SPI)由主循环 rf_app_run 完成
 * 净负荷 = [地址] + 帧; 地址过滤 buf[0], 定位头/内容指示在 buf[1] (doc/frame.md)
 */
void rf_app_rx_isr(const uint8_t *buf, uint8_t len)
{
    uint8_t ci, idx;   /* idx 由角色决定 (主机收从站1CI=3 / 从机收主站2CI=4) */
    uint8_t word, comp, n = 0, i;
    uint16_t a[4];
    uint8_t tgt[4];

    if (len < 3) return;
    /* 地址过滤 (净负荷第一字节 = 超帧地址段) */
    if (buf[0] != UART3_SELF_ADDR && buf[0] != FRAME_ADDR_BCAST) return;
    if ((buf[1] >> 1) != FRAME_HEAD) return;          /* 定位头 */

    RF_M3_RX_EVT = 1;   /* 有效帧事件 (统一异步主从, 主循环处理) */

    if (buf[1] & 0x01) {                              /* 内容指示=1: 命令/485 帧 */
        if (buf[2] & 0x80) {                          /* bit7=1: RS-485 数据帧 */
            uint8_t l = buf[2] & 0x7F;
            if (l > 0 && (uint8_t)(3 + l) <= len)
                uart1_send(&buf[3], l);               /* 485 数据 -> UART1 发出 */
        } else {
            /* 命令帧: 命令码在 buf[2] 低7bit (doc/frame.md) */
            if ((buf[2] & 0x7F) == RF_CMD_LINK_TEST
                && UART3_RF_ROLE == RF_ROLE_SLAVE) {
                RF_ACK_PENDING = 1;                   /* 从机收到探测 -> 回 ACK */
            }
        }
        RF_M3_LINK = 1;                               /* 收到命令帧 = 对端在线 */
        RF_LINK_LAST_RX_MS = TICK_MS;                 /* 命令/485 帧也算链路活动 */
        RF_LINK_PROBED = 0;
        RF_LINK_ALARM = 0;
        return;                                       /* 命令/485 帧不更新输出 */
    }

    /* 遥测帧: 按角色解析
     * 主机收到从站回传: [0x36][CI][数据]            -> idx=3
     * 从机收到主站帧:   [0x36][CI₁][CI₂][数据按CI₁] -> idx=4, 记录 CI₂ */
    ci = buf[2];
    if (UART3_RF_ROLE == RF_ROLE_SLAVE) {
        UART3_SLAVE_RET_CI = buf[3];   /* 主站要求从站回传的内容指示 */
        idx = 4;
    } else {
        idx = 3;
    }
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

    RF_APP_RX_CNT++;        /* 统计: 收到有效帧 */
    if (!RF_M3_LINK) DBG_STR("[D]link_up\r\n");   /* 通联成功(0->1) */
    RF_M3_LINK = 1;                 /* 通联成功 (收到对端有效帧) */
    RF_LINK_LAST_RX_MS = TICK_MS;   /* 链路恢复 */
    RF_LINK_PROBED = 0;             /* 允许再次探测 */
    RF_LINK_ALARM = 0;              /* 灭链路报警 (rf_led_refresh 生效) */
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
void rf_app_poll(void)
{
    DBG_POS(2);   /* 位置码: rf_app_poll 入口 */

    /* 0. 发送超时兜底: RF_TX_BUSY 卡 500ms 未完成 -> 强制回接收 (防死锁) */
    if (RF_TX_BUSY && (uint16_t)(TICK_MS - RF_TX_START_MS) > 500) {
        DBG_POS(3);   /* 位置码: 发送超时兜底 */
        rf_abort_tx();
        RF_TX_DONE = 1;               /* 让下方收尾消一次溢出 */
        DBG_STR("[D]txabort\r\n");
    }

    /* 1. 发送完成收尾: 清标志, 消一次堆叠灯 */
    if (RF_TX_DONE) {
        DBG_POS(4);   /* 位置码: 发送收尾 */
        RF_TX_DONE = 0;
        if (RF_APP_OVERFLOW) RF_APP_OVERFLOW--;
        /* DBG_STR("[D]txdone ovf="); DBG_DEC(RF_APP_OVERFLOW); DBG_NL();  高频, 注释减少刷屏 */
        rf_led_refresh();   /* 堆叠/溢出/报警 统一刷新 */
    }

    /* 2. 接收解析: 主循环从环形缓冲取帧 -> 解包 (原 ISR 解包移这里, ISR 只快速搬帧) */
    {
        uint8_t rbuf[RF_RX_MAX], rlen;
        int8_t rr, rs;
        DBG_POS(5);   /* 位置码: 接收解析循环 */
        while (rf_rx_pop(rbuf, &rlen, &rr, &rs)) {
            RF_LAST_RSSI = rr;
            RF_LAST_SNR  = rs;
            rf_app_rx_isr(rbuf, rlen);   /* 解包更新输出内存 */
        }
    }
    /* 3. 接收应用: 解包已更新输出内存, 同步硬件 */
    if (RF_OUT_UPDATED) {
        DBG_POS(6);   /* 位置码: 输出同步(GPIO/DAC) */
        RF_OUT_UPDATED = 0;
        dio_write_do();    /* PORT_DO_L/H -> GPIO */
        write_dac_all();   /* PORT_DO0~3_H/L -> AD5314 (SPI) */
    }
    /* SYS 灯统一刷新: 环形溢出(rf_rx_pop 清 OVF) / 链路报警(模式3 置, 收帧清) */
    DBG_POS(7);   /* 位置码: SYS 灯刷新 */
    rf_led_refresh();

    /* 频偏校正执行 (控制指令 0x26 或模式3 自动触发; 仅校正开关打开才允许触发) */
    if (RF_APP_FE_REQUEST) {
        DBG_POS(8);   /* 位置码: 频偏校正 */
        RF_APP_FE_REQUEST = 0;
        {
            int32_t hz, nv;
            if (rf_freq_correct_measure(&hz)
                && hz > -200000 && hz < 200000) {
                /* 累加校正值: 本机频率向对端实际频率靠拢 (FEI>0 -> 频率偏高) */
                nv = (int32_t)UART3_FE_VALUE + hz;
                if (nv >  32767) nv =  32767;
                if (nv < -32767) nv = -32767;
                UART3_FE_VALUE = (int16_t)nv;
                rf_set_freq_offset(UART3_FE_VALUE);
                UART3_FE_STATUS = 1;      /* 校正成功 */
            } else {
                UART3_FE_STATUS = 2;      /* 失败: 无有效帧 / FEI 超范围 */
            }
        }
    }
}

/* ==================== 主循环 (mode3 远程发射) ====================
 * 公共维护 + 按 UART3_RF_MODE 的 4 模式收发调度
 */
void rf_app_run(void)
{
    uint8_t mode = UART3_RF_MODE;
    uint16_t now = rtc_get_ms();
    uint8_t i;

    DBG_POS(1);   /* 位置码: rf_app_run 入口 */

#ifdef RF_DEBUG
    {   /* 低频统计(每2s一行): 时间戳 + 收帧率 + RF 发送占用率(性能剩余)
         * t=时间戳ms rx=累计收帧 drx=窗口增量 rxrate=帧/秒 busy=发送占用ms */
        static uint16_t st_last = 0;
        static uint16_t st_rx_last = 0;
        static uint16_t st_tx_last = 0;
        static uint16_t st_busy = 0;
        static uint16_t st_tick_prev = 0;
        uint16_t dtick = (uint16_t)(TICK_MS - st_tick_prev);
        st_tick_prev = TICK_MS;
        if (RF_TX_BUSY) st_busy = (uint16_t)(st_busy + dtick);
        if ((uint16_t)(TICK_MS - st_last) >= 2000) {
            uint16_t dt = (uint16_t)(TICK_MS - st_last);
            uint16_t drx = (uint16_t)(RF_APP_RX_CNT - st_rx_last);
            uint16_t dtx = (uint16_t)(RF_APP_TX_CNT - st_tx_last);
            st_last = TICK_MS;
            st_rx_last = RF_APP_RX_CNT;
            st_tx_last = RF_APP_TX_CNT;
            DBG_STR("[D]st t="); DBG_DEC(TICK_MS);
            DBG_STR(" rx="); DBG_DEC(RF_APP_RX_CNT);
            DBG_STR(" tx="); DBG_DEC(RF_APP_TX_CNT);
            DBG_STR(" dtx="); DBG_DEC(dtx);
            DBG_STR(" drx="); DBG_DEC(drx);
            DBG_STR(" busy="); DBG_DEC(st_busy);
            DBG_STR("ms link="); DBG_DEC(RF_M3_LINK);
            DBG_NL();
            st_busy = 0;
        }
    }
#endif

    rf_app_poll();   /* 公共维护: 接收解析/发送收尾/溢出灯/频偏校正 */

    {   /* 进模式3 首次运行: 打印关键状态 (诊断配置是否生效/为何不发握手) */
        static uint8_t run_once = 0;
        if (!run_once) {
            run_once = 1;
            DBG_STR("[D]run mode="); DBG_DEC(UART3_RF_MODE);
            DBG_STR(" role="); DBG_DEC(UART3_RF_ROLE);
            DBG_STR(" tx0="); DBG_HEX8(UART3_TX_CONTENT[0]);
            DBG_STR(" link="); DBG_DEC(RF_M3_LINK);
            DBG_STR(" txbusy="); DBG_DEC(RF_TX_BUSY);
            DBG_STR(" tick="); DBG_DEC(TICK_MS);
            DBG_STR(" try="); DBG_DEC(RF_LINK_TRY_MS);
            DBG_NL();
        }
    }

    /* ===== 通联状态机 (所有发射模式前置): 先尝试通联, 成功才传数据 ===== */
    DBG_POS(9);   /* 位置码: 通联状态机 */
    if (!RF_M3_LINK) {
        /* 未通联: 只有主机主动发握手探测, 从机被动应答(不主动发) */
        if (UART3_RF_ROLE == RF_ROLE_MASTER &&
            !RF_TX_BUSY &&
            (uint16_t)(TICK_MS - RF_LINK_TRY_MS) >= RF_LINK_TRY_PERIOD_MS) {
            RF_LINK_TRY_MS = TICK_MS;
            rf_send_cmd(RF_CMD_LINK_TEST);   /* 主机发握手探测 */
            DBG_STR("[D]link_try\r\n");
        }
        /* 应答收到的帧 (从机收到主机握手/数据时回一帧) */
        if (RF_ACK_PENDING && !RF_TX_BUSY) {
            RF_ACK_PENDING = 0;
            rf_send_slave(UART3_SLAVE_RET_CI);
        }
        return;   /* 未通联不发数据 */
    }

    DBG_POS(10);   /* 位置码: 已通联 */
    if (UART3_RF_ROLE == RF_ROLE_SLAVE) {
        /* 从机: 被动 - 收到主站帧回传从机数据 (按主站要求的 CI₂) */
        if (RF_M3_RX_EVT && !RF_TX_BUSY) {
            RF_M3_RX_EVT = 0;
            rf_send_slave(UART3_SLAVE_RET_CI);
        }
    } else {
        /* 主机: 主动 - 按发射表周期发 (每 enabled 任务发 CI₁+CI₂), 超时重传, 连续失败回握手 */
        if (RF_M3_STATE == 0) {
            /* 待发: 找第一个到期的 enabled 任务发一帧 */
            for (i = 0; i < 32; i++) {
                if ((UART3_TX_ENA[i] & 0x01) && !RF_TX_BUSY &&
                    (uint16_t)(now - RF_TX_LAST[i]) >=
                        rf_period_ms(UART3_TX_PERIOD_L[i], UART3_TX_PERIOD_H[i])) {
                    RF_TX_LAST[i] = now;
                    RF_M3_STATE = 1;
                    RF_M3_LAST_TX_MS = TICK_MS;
                    RF_M3_RETRY_CNT = 0;
                    RF_M3_LAST_CI1 = UART3_TX_CONTENT[i];
                    RF_M3_LAST_CI2 = UART3_TX_CI2[i];
                    rf_send_master(RF_M3_LAST_CI1, RF_M3_LAST_CI2);
                    break;   /* 每次只发一帧 */
                }
            }
        } else {
            /* 等回包 */
            if (RF_M3_RX_EVT) {
                RF_M3_RX_EVT = 0;
                RF_M3_RETRY_CNT = 0;
                RF_M3_STATE = 0;   /* 收到回包 -> 下周期再发 */
            } else if (!RF_TX_BUSY &&
                       (uint16_t)(TICK_MS - RF_M3_LAST_TX_MS) >= RF_M3_RETRY_TIMEOUT_MS) {
                /* 超时无回包 -> 重发同一帧 */
                RF_M3_RETRY_CNT++;
                if (RF_M3_RETRY_CNT >= RF_M3_RETRY_MAX) {
                    RF_M3_LINK = 0;
                    RF_LINK_TRY_MS = TICK_MS;
                    RF_LINK_ALARM = 1;
                    RF_M3_STATE = 0;
                    DBG_STR("[D]m3_lost\r\n");
                    rf_led_refresh();
                } else {
                    RF_M3_LAST_TX_MS = TICK_MS;
                    rf_send_master(RF_M3_LAST_CI1, RF_M3_LAST_CI2);
                }
            }
        }
    }

    /* 掉线检测: 已通联后超时无帧 -> 回未通联重新握手 */
    if ((uint16_t)(TICK_MS - RF_LINK_LAST_RX_MS) >= RF_LINK_LOST_MS) {
        RF_M3_LINK = 0;
        RF_LINK_TRY_MS = TICK_MS;
        RF_LINK_ALARM = 1;
        DBG_STR("[D]link_lost\r\n");
        rf_led_refresh();
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
    RF_M3_LAST_RX_MS = 0;    RF_M3_RETRY_CNT = 0;
    RF_M3_RX_EVT = 0;    RF_M3_LAST_CI1 = (CI_DH | CI_DL);    RF_M3_LAST_CI2 = (CI_DH | CI_DL);
    RF_LINK_LAST_RX_MS = TICK_MS;   /* 上电: 链路视为正常 */
    RF_LINK_PROBED = 0;
    RF_LINK_ALARM = 0;
    RF_M3_LINK = 0;                 /* 未通联: 需先握手 */
    RF_LINK_TRY_MS = TICK_MS;
    RF_PK_ACC = 0; RF_PK_BITS = 0;
    RF_UNPK_ACC = 0; RF_UNPK_BITS = 0;
}
