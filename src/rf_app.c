/*
 * rf_app.c - RF 应用层实现
 *
 * 帧协议 (doc/frame.md): [定位头(7bit)+指示字(1bit)][地址][内容指示][数据...]
 * 统一异步主从 (doc/upperpc.md 0x19 主从位, 与拨码无关):
 *   主机: 按发射表周期发帧(CI₁ 本机内容 + CI₂ 要求从站回传内容), 等回包超时重传, 连续失败回握手
 *   从机: 收到主站帧, 按主站要求的 CI₂ 回传本机数据(1 个 CI)
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

/* SYSTEM_LED = PH1 (高电平点亮); RUN_LED = PA6 (模式3 正常收发活动闪烁) */
#define RF_LED_SYS_ON()  (GPIOH->ODR |= 0x02)
#define RF_LED_SYS_OFF() (GPIOH->ODR &= (uint8_t)~0x02)
#define RF_LED_RUN_ON()  (GPIOA->ODR |= 0x40)
#define RF_LED_RUN_OFF() (GPIOA->ODR &= (uint8_t)~0x40)

/* 模式3 主从: 0=从机 1=主机 (doc/upperpc.md 0x19) */
#define RF_ROLE_SLAVE  0
#define RF_ROLE_MASTER 1
/* 主机应答模式无收包超时: 超过则回到主动轮询 (ms) */
#define RF_M3_TIMEOUT_MS 2000

/* 模式3 停等协议 (第一步宏默认值, 后续加 EEPROM 配置):
 * 主机主动发起, 回包驱动刷新; 等回包超时重发; 连续失败回握手 */
#define RF_M3_RETRY_MAX        3     /* 连续重传上限, 超限回握手 */
#define RF_M3_485_PERIOD       500   /* 485 空闲超时: 超时发保活命令占用轮次(条件2) ms */

/* 链路监控 (模式3): 掉线发 RF 探测命令重连 + SYS LED 报警 */
#define RF_LINK_CORRECT_MS 3000   /* 3s 无有效帧: 发一次 RF_CMD_LINK_TEST 探测 */
#define RF_LINK_DEAD_MS    5000   /* 5s 无有效帧: 链路断开, SYS LED 常亮报警 */

/* 通联状态机 (所有发射模式前置): 未通联先握手, 通联成功才传数据 */
/* ---- 以下 4 个超时为运行时值 (高速默认; 长距离模式由 rf_set_timeouts 放大) ---- */
volatile uint16_t RF_M3_RETRY_TIMEOUT_MS = 500;  /* 等回包超时(重发间隔) ms */
volatile uint16_t RF_LINK_TRY_PERIOD_MS  = 200;  /* 握手探测命令周期 (ms) */
volatile uint16_t RF_LINK_LOST_MS        = 3000; /* 已通联后超时无帧 -> 掉线重握手 (ms) */
volatile uint16_t RF_TX_WATCH_MS         = 500;  /* 发送卡死 Watchdog (ms) */

/* ==================== 全局状态 ==================== */
volatile uint8_t RF_APP_OVERFLOW = 0;   /* 任务堆叠计数 (非0 时 SYSTEM_LED 亮) */
/* 收发统计 + 频偏校正请求 (控制指令 0x21/0x22/0x26) */
volatile uint16_t RF_APP_RX_CNT     = 0;   /* 收到有效帧计数 */
volatile uint16_t RF_APP_TX_CNT     = 0;   /* 发送帧计数 (含握手/重传) */
volatile uint16_t RF_APP_CRC_CNT    = 0;   /* CRC 错帧计数 (rf.c 递增) */
volatile uint8_t  RF_APP_FE_REQUEST = 0;   /* 频偏校正请求 (uart3 0x26 触发) */
static volatile uint8_t RF_OUT_UPDATED = 0;  /* ISR 解包已更新输出内存 */
static volatile uint8_t RF_ACK_PENDING = 0;  /* 异步收发: 收到帧待应答 */
static volatile uint8_t RF_RUN_ACT = 0;      /* 本周期有 RF 收发活动 (模式3 RUN 灯用) */
static uint8_t RF_RUN_EVER = 0;              /* 模式3 是否已发生 RF 活动 (RUN 灯: 活动前常亮) */

/* 模式3 主机状态机 */
static uint8_t  RF_M3_STATE;        /* 主机: 0=待发 1=等回包 */
static uint16_t RF_M3_LAST_TX_MS;   /* 主机最近发帧时刻 (TICK_MS) */
static uint16_t RF_M3_LAST_RX_MS;   /* 主机最近收到帧时刻 (TICK_MS) */
static uint8_t  RF_M3_RETRY_CNT;    /* 主机连续重传计数 */
static volatile uint8_t RF_M3_RX_EVT = 0; /* 收到有效帧事件 (统一异步主从, 主循环处理) */
static volatile uint8_t RF_PULL_485_REQ = 0; /* 从机: 收到主站拉取485请求(有485则回485帧) */
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

/* RUN LED: 有 RF 活动(收发一帧)翻转一次, 无活动保持 (模式3 活动指示) */
static void rf_led_run_toggle(void)
{
    static uint8_t run_led_on = 0;
    run_led_on ^= 1;
    if (run_led_on) RF_LED_RUN_ON(); else RF_LED_RUN_OFF();
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

/* ==================== 发送 485 帧 (主机→从机; 从机收到后经其 UART1 发出) ==================== */
static void rf_send_485(void)
{
    uint8_t d[UART1_BUF_SIZE + 3];
    uint8_t dlen = uart1_take_frame(d + 3);
    if (dlen == 0) return;
    d[0] = UART3_PEER_ADDR;
    d[1] = 0x37;                        /* 定位头|内容指示=1 */
    d[2] = (uint8_t)(0x80 | (dlen & 0x7F));
    RF_APP_TX_CNT++;
    DBG_STR("[D]485tx len="); DBG_DEC(dlen); DBG_NL();
    if (rf_tx_start(d, (uint8_t)(dlen + 3))) {
        RF_APP_OVERFLOW++;
        rf_led_refresh();
    }
    UART1_LAST_TX_MS = TICK_MS;
}

/* ==================== 发送 485 拉取帧 (长度0, 无数据) ====================
 * 主机条件2(485定时到)本机无485数据时发送, 作为"拉取从机485"信号;
 * 从机收到长度0帧回传本机485(无则回长度0), 实现从机→主机上行。
 */
static void rf_send_485_poll(void)
{
    uint8_t d[3];
    d[0] = UART3_PEER_ADDR;
    d[1] = 0x37;                        /* 定位头|内容指示=1 */
    d[2] = 0x80;                        /* bit7=1: 485 帧, 长度=0 */
    RF_APP_TX_CNT++;
    if (rf_tx_start(d, 3)) {
        RF_APP_OVERFLOW++;
        rf_led_refresh();
    }
    UART1_LAST_TX_MS = TICK_MS;
}

/* ==================== 往返测试 (控制指令 0x20) ====================
 * 非阻塞: 触发即发任务0帧, 主循环正常调度; 收到从站回包时在 rf_app_poll
 * 里记录真实往返耗时到 RF_RTT_MS (超时 0xFFFF)。避免阻塞 UART3 帧处理。*/
volatile uint16_t RF_RTT_MS = 0xFFFF;   /* 真实回路往返 ms; 0xFFFF=未测/超时 */
void rf_app_test_tx(void)
{
    uint16_t t0;
    if (UART3_RF_ROLE != RF_ROLE_MASTER) { RF_RTT_MS = 0xFFFF; return; }
    RF_M3_RX_EVT = 0;                   /* 清陈旧回包 */
    RF_RTT_MS = 0xFFFF;                 /* 先置未测 */
    rf_send_master(UART3_TX_CONTENT[0], UART3_TX_CI2[0]);
    t0 = TICK_MS;                       /* 发任务0 帧起点 (TICK_MS 全局时间戳) */
    for (;;) {
        rf_app_poll();                  /* 维持收发/解包 (TIM4 照常收包) */
        if (RF_M3_RX_EVT) {             /* 收到从站回包 */
            RF_M3_RX_EVT = 0;
            RF_RTT_MS = (uint16_t)(TICK_MS - t0);
            return;
        }
        if ((uint16_t)(TICK_MS - t0) > RF_M3_RETRY_TIMEOUT_MS) {
            RF_RTT_MS = 0xFFFF;         /* 超时无回包 */
            return;
        }
    }
}

/* ==================== 内部状态只读 (快照/诊断 0x2B~0x2C) ==================== */
uint8_t rf_app_get_link(void)  { return RF_M3_LINK; }
uint8_t rf_app_get_state(void) { return RF_M3_STATE; }

/* 复位 RUN 灯闪烁状态: 每次重新进入模式3时调用 (从"先常亮"重新开始) */
void rf_app_run_reset(void)
{
    RF_RUN_EVER = 0;
    RF_RUN_ACT = 0;
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

    if (buf[1] & 0x01) {                              /* 内容指示=1: 命令/485 帧 */
        if (buf[2] & 0x80) {                          /* bit7=1: RS-485 数据帧 */
            uint8_t l = buf[2] & 0x7F;
            if (l > 0 && (uint8_t)(3 + l) <= len)
                uart1_send(&buf[3], l);               /* 485 数据 -> UART1 发出 */
            else if (l == 0 && UART3_RF_ROLE == RF_ROLE_SLAVE)
                RF_PULL_485_REQ = 1;                  /* 长度0=拉取: 从机待回传485 */
        } else {
            /* 命令帧: 从机收到任何命令帧都回 ACK (握手/链路保持) */
            if (UART3_RF_ROLE == RF_ROLE_SLAVE)
                RF_ACK_PENDING = 1;
        }
        RF_M3_LINK = 1;                               /* 收到命令帧 = 对端在线 */
        RF_LINK_LAST_RX_MS = TICK_MS;                 /* 命令/485 帧也算链路活动 */
        RF_LINK_PROBED = 0;
        RF_LINK_ALARM = 0;
        return;                                       /* 命令/485 帧不更新输出 */
    }

    RF_M3_RX_EVT = 1;   /* 仅遥测帧触发回包事件 (485/命令帧不误判主机状态机) */

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
}

/* ==================== 发送表周期换算 (TIM4 6kHz tick -> ms) ====================
 * 周期值单位 = TIM4 节拍(166.7us, 6kHz); 6 tick = 1ms, 故 ms = p/6 */
static uint16_t rf_period_ms(uint16_t pl, uint16_t ph)
{
    uint32_t p = ((uint32_t)ph << 16) | pl;
    uint32_t ms = (uint32_t)(p / 6);   /* 6 tick = 1ms (TIM4 6kHz) */
    if (ms < 1) ms = 1;
    if (ms > 60000) ms = 60000;   /* 上限保护 */
    return (uint16_t)ms;
}

/* ==================== 主循环: 4 模式收发状态机 ==================== */
void rf_app_poll(void)
{
    DBG_POS(2);   /* 位置码: rf_app_poll 入口 */

    /* 0. 发送超时兜底: RF_TX_BUSY 卡 500ms 未完成 -> 强制回接收 (防死锁) */
    if (RF_TX_BUSY && (uint16_t)(TICK_MS - RF_TX_START_MS) > RF_TX_WATCH_MS) {
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
        RF_RUN_ACT = 1;             /* RUN: 有发送活动 */
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
            RF_RUN_ACT = 1;              /* RUN: 有接收活动 */
        }
    }
    /* 从机回传 (联通测试/通用): 收到主站遥测帧即回传一帧 (UART3_SLAVE_RET_CI)
     * 模式3 中 poll 先于此清 RF_M3_RX_EVT 并回传, run 内从机回传不会重复 */
    if (UART3_RF_ROLE == RF_ROLE_SLAVE && RF_M3_RX_EVT && !RF_TX_BUSY) {
        RF_M3_RX_EVT = 0;
        rf_send_slave(UART3_SLAVE_RET_CI);   /* 回传主站要求的内容 */
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
 * 公共维护 + 统一异步主从收发调度
 */
void rf_app_run(void)
{
    uint16_t now = TICK_MS;   /* 全局 1ms 时间戳 (uint16 回绕, 差值正确; 勿用 rtc_get_ms=0..999) */
    uint8_t i;

    DBG_POS(1);   /* 位置码: rf_app_run 入口 */

    /* RUN 灯 (仅模式3): 第一次 RF 活动前常亮, 活动后随收发翻转闪烁 */
    {
        uint8_t act = RF_RUN_ACT;
        RF_RUN_ACT = 0;                 /* 清本周期活动标志 */
        if (!RF_RUN_EVER) {
            RF_LED_RUN_ON();            /* 活动前: 常亮 */
            if (act) RF_RUN_EVER = 1;   /* 第一次 RF 活动: 转闪烁 */
        } else if (act) {
            rf_led_run_toggle();        /* 活动后: 随收发翻转 */
        }
    }


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
        /* 从机: 被动 - 收到主站任何帧(遥测/命令)都回传 (按主站要求的 CI₂) */
        if ((RF_M3_RX_EVT || RF_ACK_PENDING || RF_PULL_485_REQ) && !RF_TX_BUSY) {
            RF_M3_RX_EVT = 0;
            RF_ACK_PENDING = 0;
            if (RF_PULL_485_REQ) {
                RF_PULL_485_REQ = 0;
                if (uart1_has_frame())
                    rf_send_485();                 /* 拉取+有485 -> 回485数据帧 */
                else
                    rf_send_485_poll();            /* 拉取+无485 -> 回485长度0帧(表示无数据) */
            } else {
                rf_send_slave(UART3_SLAVE_RET_CI); /* 响应遥测/命令 -> 回遥测ACK */
            }
        }
    } else {
        /* 主机: 主动 - 按发射表周期发 (每 enabled 任务发 CI₁+CI₂), 超时重传, 连续失败回握手 */
        if (RF_M3_STATE == 0) {
            /* 待发: 三条件调度决定本轮发什么 (485优先 / 遥测 / 485空闲 / 保活) */
            uint8_t have485 = uart1_has_frame();
            uint8_t task = 0xFF, ci1 = 0, ci2 = 0, sent = 0;
            /* 找发射表到期任务 (遥测) */
            if (!RF_TX_BUSY) {
                for (i = 0; i < 32; i++) {
                    if ((UART3_TX_ENA[i] & 0x01) &&
                        (uint16_t)(now - RF_TX_LAST[i]) >=
                            rf_period_ms(UART3_TX_PERIOD_L[i], UART3_TX_PERIOD_H[i])) {
                        task = i; ci1 = UART3_TX_CONTENT[i]; ci2 = UART3_TX_CI2[i];
                        break;
                    }
                }
                /* 条件1: 485 缓冲满 -> 优先发 485 (抢占遥测) */
                if (have485 && uart1_is_full()) {
                    rf_send_485(); sent = 1;
                } else if (task != 0xFF) {
                    RF_TX_LAST[task] = now;             /* 发遥测 */
                    RF_M3_LAST_CI1 = ci1; RF_M3_LAST_CI2 = ci2;
                    rf_send_master(ci1, ci2); sent = 1;
                } else if (have485) {
                    rf_send_485(); sent = 1;            /* 条件3: 无遥测到期+有485 -> 发485 */
                } else if ((uint16_t)(now - UART1_LAST_TX_MS) >= RF_M3_485_PERIOD) {
                    /* 条件2: 485 定时到 */
                    if (have485) {
                        rf_send_485(); sent = 1;              /* 有485: 发485 */
                    } else {
                        rf_send_485_poll();                   /* 无485: 发485长度0帧拉取从机(不进state=1) */
                    }
                    UART1_LAST_TX_MS = now;
                }
                if (sent) {
                    RF_M3_STATE = 1;                    /* 进等回包 */
                    RF_M3_LAST_TX_MS = TICK_MS;
                    RF_M3_RETRY_CNT = 0;
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
                /* 超时无回包 -> 断连重试: SYS 灯亮 (恢复断链后 rx_isr 清 alarm -> 灭) */
                RF_M3_RETRY_CNT++;
                RF_LINK_ALARM = 1;
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

/* ==================== 超时设置 (高速/长距离) ====================
 * 长距离最坏情况 = 完整超帧 (前导+同步+长度+地址+载荷+CRC):
 * LoRa SF12/BW125/CR4/8 下单帧 ~0.93s, 一次收发往返 ~1.85s。
 * 长距离模式各超时取往返 2 倍以上, 且掉线阈值 > 任务周期(5000ms 演示), 防误判掉线。
 * 高速(默认)保持原值, FSK/快链不受影响。 */
void rf_set_timeouts(uint8_t long_range)
{
    if (long_range) {
        RF_M3_RETRY_TIMEOUT_MS = 4000;  /* 等回包: > 一次往返 ~1.85s */
        RF_LINK_LOST_MS        = 8000;  /* 掉线检测: > 任务周期 5000ms, 防误判 */
        RF_LINK_TRY_PERIOD_MS  = 2000;  /* 握手探测: 探测+应答 ~1.85s */
        RF_TX_WATCH_MS         = 3000;  /* TX 兜底: > 最慢单帧(SF12/BW125 ~0.93s+前导), 防误杀 */
    } else {
        RF_M3_RETRY_TIMEOUT_MS = 500;
        RF_LINK_LOST_MS        = 3000;
        RF_LINK_TRY_PERIOD_MS  = 200;
        RF_TX_WATCH_MS         = 500;
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
    RF_PULL_485_REQ = 0;
    RF_LINK_LAST_RX_MS = TICK_MS;   /* 上电: 链路视为正常 */
    RF_LINK_PROBED = 0;
    RF_LINK_ALARM = 0;
    RF_M3_LINK = 0;                 /* 未通联: 需先握手 */
    RF_LINK_TRY_MS = TICK_MS;
    RF_PK_ACC = 0; RF_PK_BITS = 0;
    RF_UNPK_ACC = 0; RF_UNPK_BITS = 0;
}
