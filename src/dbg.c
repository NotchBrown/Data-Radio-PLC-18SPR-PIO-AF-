/*
 * dbg.c - UART3 调试打印实现
 *
 * 复用 UART3 TX (与协议回复共享串口, 诊断时用串口监视器观察)。
 * 打印前检查 dbg_ready (uart3_init 后才置 1), 未初始化直接返回,
 * 保证早期 TIM4 ISR (rf_poll) 调用安全 (不写未初始化 UART3 / 不死等)。
 */
#include "dbg.h"
#include <stm8s.h>

static volatile uint8_t dbg_ready = 0;

/* 初始化: 在 uart3_init 之后调用 */
void dbg_init(void)
{
    dbg_ready = 1;
}

static void dbg_putchar(char c)
{
    uint16_t guard = 0;
    if (!dbg_ready) return;
    while (!(UART3->SR & 0x80)) {   /* 等 TXE */
        if (++guard > 2000) return;   /* 超时保护: 不死循环 */
    }
    UART3->DR = (uint8_t)c;
}

void dbg_puts(const char *s)
{
    while (*s)
        dbg_putchar(*s++);
}

void dbg_hex8(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    dbg_putchar(hex[(v >> 4) & 0x0F]);
    dbg_putchar(hex[v & 0x0F]);
}

void dbg_dec(uint16_t v)
{
    char buf[6];
    uint8_t i = 0;
    if (v == 0) { dbg_putchar('0'); return; }
    while (v && i < 5) {
        buf[i++] = (char)('0' + (v % 10));
        v = (uint16_t)(v / 10);
    }
    while (i)
        dbg_putchar(buf[--i]);
}

void dbg_newline(void)
{
    dbg_putchar('\r');
    dbg_putchar('\n');
}

/* ===== 卡死位置码 (RAM) =====
 * 各关键函数写入, SWIM 调试时看变量定位卡死点。
 * 用 RAM 而非 EEPROM: rf_app_poll 每圈写位置码, 若写 EEPROM 会频繁擦写
 * 磨损 + 写 EEPROM 时 CPU stall(约几 ms/次), 反而诱发主循环卡死。
 * RAM 赋值零开销, 无副作用。 */
static volatile uint8_t DBG_POS_RAM;

void dbg_pos(uint8_t p)
{
    DBG_POS_RAM = p;
}

uint8_t dbg_pos_get(void)
{
    return DBG_POS_RAM;
}
