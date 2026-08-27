/*
 * main.c - RF 调试器 (test/rf_debug 独立工程)
 * 通过 UART3(115200) 命令行调试 RA-01 SX1278 裸 RF 链路
 *
 * 烧录: cd test/rf_debug && pio run -t upload
 * 使用: pio device monitor (或任意串口工具 115200), 输入 help 看命令
 *
 * 典型排查流程:
 *   1. ver    -> 确认 SX1278 响应 (0x12=SX1276/78), 验证 SPI 通
 *   2. reg 1D / 1E / 39 -> 确认调制/同步字配置
 *   3. init   -> 复位+默认参数
 *   4. 一块板 tx 010203, 另一块板 rx -> 看是否收到 + RSSI/SNR
 *   5. 若收不到: 对比两板 freq/sf/bw/cr/同步字, 检查 RSSI(接收时)是否接近环境底噪
 */
#include <Arduino.h>
#include <stm8s.h>
#include <stdio.h>
#include <string.h>
#include "spi.h"
#include "rf_dbg.h"

/* ==================== 时钟: HSE 24MHz (同主固件) ==================== */
static void clock_init(void)
{
    uint16_t guard = 0;
    CLK_HSECmd(ENABLE);
    while (CLK_GetFlagStatus(CLK_FLAG_HSERDY) == RESET) {
        if (++guard > 100000) return;   /* 超时: 保持 HSI */
    }
    CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE,
                          DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);
    CLK_HSICmd(DISABLE);
}

/* ==================== UART3 (115200/8N1) + printf ==================== */
void uart3_init(void)
{
    GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST); /* TX PD5 */
    GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);      /* RX PD6 */
    UART3_Init(115200, UART3_WORDLENGTH_8D, UART3_STOPBITS_1,
               UART3_PARITY_NO, UART3_MODE_TXRX_ENABLE);
    UART3_Cmd(ENABLE);
}

/* printf 重定向到 UART3 */
int putchar(int c)
{
    while (!(UART3->SR & 0x80));   /* 等 TXE */
    UART3->DR = (uint8_t)c;
    return c;
}

/* ==================== 命令表 ==================== */
static void cmd_help(void)
{
    printf("\n=== RA-01 SX1278 调试命令 ===\r\n");
    printf("  ver                 读芯片版本 (0x12=SX1276/78)\r\n");
    printf("  reg XX              读寄存器 0xXX\r\n");
    printf("  wreg XX YY          写寄存器 0xXX = 0xYY\r\n");
    printf("  init                复位 + LoRa 初始化 (470M/SF7/BW125/CR4/5/13dBm)\r\n");
    printf("  freq <hz>           设载波频率, 如 freq 470000000\r\n");
    printf("  sf <6-12>           扩频因子\r\n");
    printf("  bw <125|250|500>    带宽 kHz\r\n");
    printf("  cr <5-8>            编码率 (4/5..4/8)\r\n");
    printf("  pwr <2-17>          发射功率 dBm\r\n");
    printf("  tx <hex...>         发送一帧, 如 tx 00 11 22\r\n");
    printf("  rx [timeout]        接收一包 (默认 2000ms), 打印 hex+RSSI+SNR\r\n");
    printf("  rssi                接收模式下读当前 RSSI\r\n");
    printf("  dump                打印全部 LoRa 关键寄存器\r\n");
    printf("  carr [ms]           连续载波发射(诊断信号到达, 默认2000ms)\r\n");
    printf("  txc [ms]            连续发射真实LoRa帧(诊断解调, 默认3000ms)\r\n");
    printf("  txl [n]             循环发标准单帧(每帧含前导码, 默认20次)\r\n");
    printf("  fe                  进RX收一帧读FrequencyError(需对端发帧)\r\n");
    printf("  tune [base] [r]     自动扫频精调找对端频率(需对端txc连发)\r\n");
    printf("  help                本帮助\r\n");
}

/* 一次性打印所有 LoRa 关键寄存器 (对比官方驱动配置链用) */
static void cmd_dump(void)
{
    /* SDCC printf 限制参数数量, 每条不超过 6 个 */
    printf("OPMODE=0x%02X FRF=0x%02X%02X%02X PA=0x%02X LNA=0x%02X\r\n",
           rf_dbg_read_reg(0x01),
           rf_dbg_read_reg(0x06), rf_dbg_read_reg(0x07), rf_dbg_read_reg(0x08),
           rf_dbg_read_reg(0x09), rf_dbg_read_reg(0x0C));
    printf("OCP=0x%02X FIFO=0x%02X%02X%02X IRQMASK=0x%02X IRQ=0x%02X\r\n",
           rf_dbg_read_reg(0x0B),
           rf_dbg_read_reg(0x0D), rf_dbg_read_reg(0x0E), rf_dbg_read_reg(0x0F),
           rf_dbg_read_reg(0x11), rf_dbg_read_reg(0x12));
    printf("MCFG1=0x%02X MCFG2=0x%02X SYMB=0x%02X PRE=0x%02X%02X MCFG3=0x%02X\r\n",
           rf_dbg_read_reg(0x1D), rf_dbg_read_reg(0x1E), rf_dbg_read_reg(0x1F),
           rf_dbg_read_reg(0x20), rf_dbg_read_reg(0x21), rf_dbg_read_reg(0x26));
    printf("PAYLEN=0x%02X MAXPAY=0x%02X HOP=0x%02X RSSI=0x%02X\r\n",
           rf_dbg_read_reg(0x22), rf_dbg_read_reg(0x23), rf_dbg_read_reg(0x24),
           rf_dbg_read_reg(0x1B));
    printf("DETOPT=0x%02X INVIRQ=0x%02X DETTH=0x%02X SYNC=0x%02X IMGCL=0x%02X\r\n",
           rf_dbg_read_reg(0x31), rf_dbg_read_reg(0x33), rf_dbg_read_reg(0x37),
           rf_dbg_read_reg(0x39), rf_dbg_read_reg(0x3B));
    printf("DIOMAP1=0x%02X DIOMAP2=0x%02X PADAC=0x%02X VER=0x%02X\r\n",
           rf_dbg_read_reg(0x40), rf_dbg_read_reg(0x41), rf_dbg_read_reg(0x4D),
           rf_dbg_read_reg(0x42));
}

static uint8_t hexv(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    return 0xFF;
}

/* 手动解析 (SDCC 无 sscanf) */
static unsigned int parse_hex(const char *s)
{
    unsigned int v = 0;
    while (*s) {
        uint8_t d = hexv(*s);
        if (d == 0xFF) break;
        v = (unsigned int)((v << 4) | d);
        s++;
    }
    return v;
}

static unsigned long parse_dec(const char *s)
{
    unsigned long v = 0;
    while (*s >= '0' && *s <= '9')
        v = v * 10 + (unsigned long)(*s++ - '0');
    return v;
}

/* 连续载波发射 (诊断用): 设 TX 连续模式, 持续发射, 用于确认信号是否到达接收端 */
static void cmd_carr(char *s)
{
    unsigned long ms = parse_dec(s);
    unsigned long i;
    if (ms < 100) ms = 2000;
    if (ms > 10000) ms = 10000;

    rf_dbg_write_reg(RF_REG_OPMODE, 0x81);   /* STDBY (LongRange 0x80 + STDBY) */
    /* TX 连续模式: ModemConfig2 bit3(TxContinuousMode)=1, 0x74 -> 0x7C */
    rf_dbg_write_reg(RF_REG_MODEM_CONFIG_2,
                     (rf_dbg_read_reg(RF_REG_MODEM_CONFIG_2) & 0xF7) | 0x08);
    rf_dbg_write_reg(RF_REG_OPMODE, 0x83);   /* TX (LongRange 0x80 + TX) */
    printf("连续载波发射 %lu ms ...\r\n", ms);
    for (i = 0; i < ms; i++) delay(1);
    /* 恢复正常单帧 TX 模式 */
    rf_dbg_write_reg(RF_REG_MODEM_CONFIG_2,
                     (rf_dbg_read_reg(RF_REG_MODEM_CONFIG_2) & 0xF7));
    rf_dbg_write_reg(RF_REG_OPMODE, 0x81);   /* STDBY */
    printf("[完成]\r\n");
}

/* 连续发射真实 LoRa 帧 (诊断用): TX 连续模式下持续重发固定帧,
 * 给接收端充足时间解调, 区分"单帧TX问题"vs"RX解调问题" */
static void cmd_txc(char *s)
{
    unsigned long ms = parse_dec(s);
    unsigned long i;
    uint8_t i2;
    if (ms < 100) ms = 3000;
    if (ms > 10000) ms = 10000;

    rf_dbg_write_reg(RF_REG_OPMODE, 0x81);   /* STDBY */
    rf_dbg_write_reg(RF_REG_IRQ_FLAGS, 0xFF);
    rf_dbg_write_reg(RF_REG_FIFO_TX_BASE, 0x80);
    rf_dbg_write_reg(RF_REG_FIFO_ADDR_PTR, 0x80);
    for (i2 = 0; i2 < 4; i2++)              /* 写入固定帧 0xAA 0x55 0x01 0x02 */
        rf_dbg_write_reg(RF_REG_FIFO, (i2==0)?0xAA:(i2==1)?0x55:(i2==2)?0x01:0x02);
    rf_dbg_write_reg(RF_REG_PAYLOAD_LENGTH, 4);
    /* TX 连续模式: 持续重发该帧 */
    rf_dbg_write_reg(RF_REG_MODEM_CONFIG_2,
                     (rf_dbg_read_reg(RF_REG_MODEM_CONFIG_2) & 0xF7) | 0x08);
    rf_dbg_write_reg(RF_REG_OPMODE, 0x83);   /* TX */
    printf("连续发帧 %lu ms (AA 55 01 02) ...\r\n", ms);
    for (i = 0; i < ms; i++) delay(1);
    rf_dbg_write_reg(RF_REG_MODEM_CONFIG_2,
                     (rf_dbg_read_reg(RF_REG_MODEM_CONFIG_2) & 0xF7));
    rf_dbg_write_reg(RF_REG_OPMODE, 0x81);   /* STDBY */
    printf("[完成]\r\n");
}

/* 循环发送标准单帧 (每帧含前导码+header, 供接收端正常解调)。
 * 与 txc(TX连续模式, 只发一次前导码)不同, txl 反复调用标准 rf_dbg_tx,
 * 每帧都是完整 LoRa 帧 -> 接收端每次都能同步。 */
static void cmd_txl(char *s)
{
    unsigned long n = parse_dec(s);
    unsigned long i;
    uint8_t frm[4] = {0xAA, 0x55, 0x01, 0x02};
    if (n < 1) n = 20;
    if (n > 200) n = 200;
    printf("循环发标准帧 %lu 次 (AA 55 01 02)...\r\n", n);
    for (i = 0; i < n; i++) {
        rf_dbg_tx(frm, 4);
        delay(50);           /* 帧间隔 */
    }
    printf("[完成]\r\n");
    rf_dbg_rx_start();
}

/* 读 FrequencyError: 进 RX 等一帧(需对端发帧), 收到后打印频偏 */
static void cmd_fe(char *s)
{
    uint8_t buf[64], len;
    int32_t fe;
    printf("进 RX 等一帧读 FrequencyError (需对端发帧)...\r\n");
    if (rf_dbg_rx(buf, &len, 2000, NULL, NULL) == 1) {
        fe = rf_dbg_read_fe();
        printf("收到 %uB, FrequencyError=%ld Hz (正=对端频率比本机高)\r\n", len, (long)fe);
    } else {
        printf("超时/无包\r\n");
    }
    rf_dbg_rx_start();   /* 回到连续接收 */
}

/* 自动扫频精调: 在 base±range kHz 内扫, 找能锁定对端的频率并校正 */
static void cmd_tune(char *s)
{
    unsigned long base = 470000000UL;
    long range = 400000;          /* ±400 kHz 粗扫范围 */
    uint32_t tuned;
    /* 支持: tune [base_hz] [range_hz] */
    char *p = s;
    if (*p) {
        base = parse_dec(p);
        while (*p >= '0' && *p <= '9') p++;
        while (*p == ' ' || *p == '\t') p++;
        if (*p) range = (long)parse_dec(p);
    }
    printf("扫频精调: base=%lu Hz range=%+ld Hz (需对端持续发帧 txc)...\r\n",
           base, range);
    tuned = rf_dbg_tune((uint32_t)base, (int32_t)range);
    if (tuned) {
        printf("校正成功! 本机频率设为 %lu Hz\r\n", tuned);
    } else {
        printf("扫描无果: 未找到能锁定对端的频率\r\n");
    }
    rf_dbg_rx_start();
}

static void cmd_tx(char *s)
{
    uint8_t buf[64], n = 0;
    char *p = s;

    while (*p && n < sizeof(buf)) {
        uint8_t hi, lo;
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        hi = hexv(p[0]); lo = hexv(p[1]);
        if (hi == 0xFF || lo == 0xFF) break;   /* 需两位 hex */
        buf[n++] = (uint8_t)((hi << 4) | lo);
        p += 2;
    }
    if (!n) { printf("用法: tx AA BB CC ...\r\n"); return; }

    {
        uint8_t i;
        printf("发送 %uB:", n);
        for (i = 0; i < n; i++) printf(" %02X", buf[i]);
        printf("\r\n");
    }
    rf_dbg_tx(buf, n);   /* 发完自动回 RXCONT */
    printf("[TxDone] 等 ACK 1s...\r\n");
    {
        uint8_t abuf[8], alen, ar;
        int8_t arssi, asnr;
        unsigned long t;
        for (t = 0; t < 100; t++) {
            ar = rf_dbg_rx_check(abuf, &alen, &arssi, &asnr);
            if (ar == 2) { printf("[ACK OK] rssi=%ddBm\r\n", arssi); return; }
            if (ar == 1) {
                uint8_t i;
                printf("[等ACK时收到数据] %uB:", alen);
                for (i = 0; i < alen; i++) printf(" %02X", abuf[i]);
                printf("\r\n");
            }
            delay(10);
        }
        printf("[无ACK]\r\n");
    }
}

static void cmd_scan(char *s)
{
    unsigned long ms = parse_dec(s);
    unsigned long i;
    if (ms < 100) ms = 3000;
    if (ms > 10000) ms = 10000;
    printf("连续接收 %lums, 每200ms打印 RSSI/事件...\r\n", ms);
    rf_dbg_rx_start();
    for (i = 0; i < ms; i += 200) {
        uint8_t f = rf_dbg_irq();
        /* 打印任何解调事件标志: 0x80=RxTimeout 0x40=RxDone 0x20=CRC错
           0x10=ValidHeader 0x08=TxDone */
        if (f)
            printf("  [irq=0x%02X]", f);
        printf("  t=%4lu  rssi=%ddBm\r\n", i, rf_dbg_cur_rssi());
        delay(200);
    }
    rf_dbg_rx_stop();
    printf("扫描结束\r\n");
}

static void cmd_txn(char *s)
{
    unsigned long cnt = parse_dec(s);
    uint8_t buf[64], n = 0, i;
    char *p = s;

    while (*p >= '0' && *p <= '9') p++;
    while (*p == ' ' || *p == '\t') p++;
    while (*p && n < sizeof(buf)) {
        uint8_t hi, lo;
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        hi = hexv(p[0]); lo = hexv(p[1]);
        if (hi == 0xFF || lo == 0xFF) break;
        buf[n++] = (uint8_t)((hi << 4) | lo);
        p += 2;
    }
    if (cnt == 0 || cnt > 100) cnt = 10;
    if (!n) { printf("用法: txn <n> <hex...>\r\n"); return; }

    printf("连续发 %lu 帧: ", cnt);
    for (i = 0; i < n; i++) printf("%02X ", buf[i]);
    printf("\r\n");
    for (i = 0; i < (uint8_t)cnt; i++) {
        rf_dbg_tx(buf, n);
        delay(300);
    }
    printf("[完成]\r\n");
}

static void cmd_rx(char *s)
{
    uint16_t timeout = 2000;
    uint8_t buf[64], len = 0, i, r;
    int8_t rssi, snr;
    unsigned long t;

    t = parse_dec(s);
    if (t != 0)
        timeout = (uint16_t)t;

    printf("等待接收 %ums ...\r\n", timeout);
    r = rf_dbg_rx(buf, &len, timeout, &rssi, &snr);
    if (r == 1) {
        printf("收到 %uB:", len);
        for (i = 0; i < len; i++) printf(" %02X", buf[i]);
        printf(" | RSSI=%ddBm SNR=%ddB\r\n", rssi, snr);
    } else if (r == 2) {
        printf("收到 %uB 但 CRC 错 | RSSI=%ddBm SNR=%ddB\r\n", len, rssi, snr);
    } else {
        printf("超时/无包\r\n");
    }
}

static void cmd_dispatch(char *s)
{
    char tmp[64];
    char *tok;

    strncpy(tmp, s, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    tok = strtok(tmp, " \t");
    if (!tok) return;

    if (!strcmp(tok, "help")) { cmd_help(); return; }

    if (!strcmp(tok, "ver")) {
        printf("Version=0x%02X\r\n", rf_dbg_read_reg(RF_REG_VERSION));
    } else if (!strcmp(tok, "reg")) {
        tok = strtok(0, " \t");
        if (tok) {
            unsigned int a = parse_hex(tok);
            printf("reg[0x%02X]=0x%02X\r\n", a, rf_dbg_read_reg((uint8_t)a));
        } else printf("用法: reg XX\r\n");
    } else if (!strcmp(tok, "wreg")) {
        tok = strtok(0, " \t");
        if (tok) {
            unsigned int a = parse_hex(tok);
            tok = strtok(0, " \t");
            if (tok) {
                unsigned int b = parse_hex(tok);
                rf_dbg_write_reg((uint8_t)a, (uint8_t)b);
                printf("reg[0x%02X]=0x%02X (读回 0x%02X)\r\n",
                       a, b, rf_dbg_read_reg((uint8_t)a));
            } else printf("用法: wreg XX YY\r\n");
        } else printf("用法: wreg XX YY\r\n");
    } else if (!strcmp(tok, "init")) {
        uint8_t ok = rf_dbg_init();
        printf("RF 初始化 %s | Version=0x%02X | OPMODE=0x%02X (期望 0x86)\r\n",
               ok ? "完成" : "失败",
               rf_dbg_read_reg(RF_REG_VERSION),
               rf_dbg_read_reg(RF_REG_OPMODE));
    } else if (!strcmp(tok, "freq")) {
        tok = strtok(0, " \t");
        if (tok) {
            unsigned long f = parse_dec(tok);
            rf_dbg_set_freq(f);
            printf("载波频率=%lu Hz\r\n", f);
        } else printf("用法: freq <hz>\r\n");
    } else if (!strcmp(tok, "sf")) {
        tok = strtok(0, " \t");
        if (tok) {
            unsigned long a = parse_dec(tok);
            if (a >= 6 && a <= 12) { rf_dbg_set_modem((uint8_t)a, 0, 0); printf("SF=%lu\r\n", a); }
            else printf("SF 取 6~12\r\n");
        } else printf("用法: sf <6-12>\r\n");
    } else if (!strcmp(tok, "bw")) {
        tok = strtok(0, " \t");
        if (tok) {
            unsigned long a = parse_dec(tok);
            if (a == 125 || a == 250 || a == 500) { rf_dbg_set_modem(0, (uint16_t)a, 0); printf("BW=%lukHz\r\n", a); }
            else printf("BW 取 125/250/500\r\n");
        } else printf("用法: bw <125|250|500>\r\n");
    } else if (!strcmp(tok, "cr")) {
        tok = strtok(0, " \t");
        if (tok) {
            unsigned long a = parse_dec(tok);
            if (a >= 5 && a <= 8) { rf_dbg_set_modem(0, 0, (uint8_t)a); printf("CR=4/%lu\r\n", a); }
            else printf("CR 取 5~8 (4/5..4/8)\r\n");
        } else printf("用法: cr <5-8>\r\n");
    } else if (!strcmp(tok, "pwr")) {
        tok = strtok(0, " \t");
        if (tok) {
            unsigned long a = parse_dec(tok);
            if (a >= 2 && a <= 17) { rf_dbg_set_power((int8_t)a); printf("功率=%ludBm\r\n", a); }
            else printf("功率取 2~17 dBm\r\n");
        } else printf("用法: pwr <2-17>\r\n");
    } else if (!strcmp(tok, "tx")) {
        char *rest = s;                       /* 用原始行: tmp 已被 strtok 破坏 */
        while (*rest == ' ' || *rest == '\t') rest++;
        rest += 2;                            /* 跳过 "tx" */
        while (*rest == ' ' || *rest == '\t') rest++;
        cmd_tx(rest);
    } else if (!strcmp(tok, "txn")) {
        char *rest = s;
        while (*rest == ' ' || *rest == '\t') rest++;
        rest += 3;                            /* 跳过 "txn" */
        while (*rest == ' ' || *rest == '\t') rest++;
        cmd_txn(rest);
    } else if (!strcmp(tok, "rx")) {
        char *rest = s;
        while (*rest == ' ' || *rest == '\t') rest++;
        rest += 2;                            /* 跳过 "rx" */
        while (*rest == ' ' || *rest == '\t') rest++;
        cmd_rx(rest);
    } else if (!strcmp(tok, "scan")) {
        char *rest = s;
        while (*rest == ' ' || *rest == '\t') rest++;
        rest += 4;                            /* 跳过 "scan" */
        while (*rest == ' ' || *rest == '\t') rest++;
        cmd_scan(rest);
    } else if (!strcmp(tok, "ack")) {
        char *rest = s;
        while (*rest == ' ' || *rest == '\t') rest++;
        rest += 3;                            /* 跳过 "ack" */
        while (*rest == ' ' || *rest == '\t') rest++;
        rf_dbg_set_ack(parse_dec(rest) != 0);
        printf("ACK %s (收到数据帧自动回 0xAA)\r\n", RF_DBG_ACK ? "开" : "关");
    } else if (!strcmp(tok, "rssi")) {
        rf_dbg_write_reg(RF_REG_OPMODE,
                         (rf_dbg_read_reg(RF_REG_OPMODE) & ~0x07) | 0x05); /* RXCONT */
        delay(100);
        printf("RSSI=%ddBm\r\n", (int)((int16_t)rf_dbg_read_reg(RF_REG_PKT_RSSI) - 164));
        rf_dbg_write_reg(RF_REG_OPMODE,
                         (rf_dbg_read_reg(RF_REG_OPMODE) & ~0x07) | 0x01); /* STDBY */
    } else if (!strcmp(tok, "dump")) {
        cmd_dump();
    } else if (!strcmp(tok, "carr")) {
        char *rest = s;
        while (*rest == ' ' || *rest == '\t') rest++;
        rest += 4;                            /* 跳过 "carr" */
        while (*rest == ' ' || *rest == '\t') rest++;
        cmd_carr(rest);
    } else if (!strcmp(tok, "txc")) {
        char *rest = s;
        while (*rest == ' ' || *rest == '\t') rest++;
        rest += 3;                            /* 跳过 "txc" */
        while (*rest == ' ' || *rest == '\t') rest++;
        cmd_txc(rest);
    } else if (!strcmp(tok, "txl")) {
        char *rest = s;
        while (*rest == ' ' || *rest == '\t') rest++;
        rest += 3;                            /* 跳过 "txl" */
        while (*rest == ' ' || *rest == '\t') rest++;
        cmd_txl(rest);
    } else if (!strcmp(tok, "fe")) {
        cmd_fe(s);
    } else if (!strcmp(tok, "tune")) {
        char *rest = s;
        while (*rest == ' ' || *rest == '\t') rest++;
        rest += 4;                            /* 跳过 "tune" */
        while (*rest == ' ' || *rest == '\t') rest++;
        cmd_tune(rest);
    } else {
        printf("未知命令 '%s', 输入 help 看列表\r\n", tok);
    }
}

/* ==================== 主流程 ==================== */
void setup()
{
    clock_init();      /* HSE 24MHz (需 OPT7 WAITSTATE=1) */
    spi_init();        /* SPI + CS 引脚 */
    uart3_init();      /* UART3 115200 */

    printf("\r\n=== RA-01 SX1278 调试器 (UART3 115200) ===\r\n");
    cmd_help();
    printf("默认初始化 ...\r\n");
    {
        uint8_t ok = rf_dbg_init();
        printf("RF 初始化 %s | Version=0x%02X | OPMODE=0x%02X (期望 0x86)\r\n",
               ok ? "完成" : "失败",
               rf_dbg_read_reg(RF_REG_VERSION),
               rf_dbg_read_reg(RF_REG_OPMODE));
    }
    printf(">\r\n");
}

void loop()
{
    static char line[64];
    static uint8_t n = 0;
    uint8_t rbuf[64], rlen, r;
    int8_t rssi, snr;

    /* RF 自动接收 (平时一直 RXCONT): 收到帧打印 + 自动 ACK */
    r = rf_dbg_rx_check(rbuf, &rlen, &rssi, &snr);
    if (r == 1) {
        uint8_t i;
        printf("[收] %uB:", rlen);
        for (i = 0; i < rlen; i++) printf(" %02X", rbuf[i]);
        printf(" | RSSI=%ddBm SNR=%ddB\r\n", rssi, snr);
        if (RF_DBG_ACK) {
            printf("[回ACK]\r\n");
            rf_dbg_tx((const uint8_t *)"\xAA", 1);   /* 回 ACK, 发完自动回 RXCONT */
        }
    } else if (r == 2) {
        printf("[ACK收到] rssi=%ddBm\r\n", rssi);
    } else if (r == 3) {
        printf("[CRC错] rssi=%ddBm\r\n", rssi);
    }

    /* UART3 命令轮询 */
    if (UART3->SR & 0x20) {             /* RXNE: 收到一个字符 */
        char c = (char)UART3->DR;
        if (c == '\r' || c == '\n') {
            if (n) {
                line[n] = '\0';
                cmd_dispatch(line);
                n = 0;
                printf(">\r\n");
            }
        } else if (n < sizeof(line) - 1) {
            line[n++] = c;
        }
    }
}

/* ==================== 中断向量空实现 (sduino 向量表引用) ==================== */
void TRAP_IRQHandler(void) __trap
{
    for (;;) ;
}

void UART3_RX_IRQHandler(void) __interrupt(ITC_IRQ_UART3_RX)
{
    /* 调试用轮询接收, 无需中断处理 */
}

void UART3_TX_IRQHandler(void) __interrupt(ITC_IRQ_UART3_TX)
{
}
