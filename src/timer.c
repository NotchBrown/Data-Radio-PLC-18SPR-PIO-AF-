/*
 * timer.c - TIM4 系统节拍 + RTC 实现
 * 时钟: 外部无源晶振 HSE 24MHz (需 OPT7 WAITSTATE=1, 见 doc/clock.md)
 *
 * TIM4 6kHz 中断(166.7us)作为最小节拍, 仅供内部标志/累加使用,
 * 不对外提供 us 计数。每 6 次(=1ms, 精确)递增 RTC_MS, 再按秒/分/时/日/月/年进位。
 *
 * 频率说明: 24MHz / 6000 = 4000 = 2^3 * 500, 可精确整除!
 *   预分频8 -> 3MHz, ARR=499 -> 周期500 -> 6000Hz 精确, 无偏差
 *   6kHz / 1000 = 6 次/ms 为整数, RTC 走时准确
 *   (16MHz 时同 PSCR/ARR: 16M/8/500=4kHz, TICKS_PER_MS=4)
 *   降到 6kHz: 减半 TIM4 中断开销, 给 UART3(115200)/UART1(485) 更充足的中断响应
 *   (DIO0 轮询延迟 83us->167us, RF 事件电平保持足够检测)
 *
 * 注意: 本固件不使用框架 Arduino 的 millis()/micros()/delay()
 *       (框架核心 wiring-millis.c 也定义 TIM4 中断, 若被链接会重复符号;
 *        故本模块以同名 TIM4_UPD_OVF_IRQHandler 提供自己的实现,
 *        需要延时时请基于 RTC_MS 等实现)
 */
#include "timer.h"
#include "rf.h"
#include <stm8s.h>

/* 私有: 最小节拍子计数, 每 TICKS_PER_MS 次=1ms (16M/4k=4, 24M/6k=6) */
#define TICKS_PER_MS  6
static volatile uint8_t rtc_sub_ms = 0;

/* RTC 全局变量(大写) */
volatile uint16_t RTC_MS   = 0;
volatile uint8_t  RTC_S    = 0;
volatile uint8_t  RTC_MIN  = 0;
volatile uint8_t  RTC_HOUR = 0;
volatile uint8_t  RTC_DAY  = 1;
volatile uint8_t  RTC_MON  = 1;
volatile uint8_t  RTC_YEAR = 0;

/* 1ms 递增毫秒计数 (uint16 回绕, 差值正确) */
volatile uint16_t TICK_MS = 0;

/* 当月天数(含闰年2月) */
static uint8_t days_in_month(uint8_t mon, uint8_t year)
{
    static const uint8_t dm[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (mon == 2) {
        /* 闰年: 能被4整除且(不能被100整除或能被400整除) */
        if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
            return 29;
    }
    return dm[mon - 1];
}

/* TIM4 更新中断(向量23): 每 ~167us 一次
 * 使用与 stm8s_it.h 声明一致的处理器名(向量23被其占用, 不能用别的名字) */
void TIM4_UPD_OVF_IRQHandler(void) __interrupt(ITC_IRQ_TIM4_OVF)
{
    /* 6kHz 最小节拍: 每 TICKS_PER_MS 次=1ms */
    if (++rtc_sub_ms >= TICKS_PER_MS) {   /* 6次 = 1ms(精确) */
        rtc_sub_ms = 0;
        TICK_MS++;                        /* 毫秒累计 (每1ms+1, uint16 回绕差值正确) */
        if (++RTC_MS >= 1000) {           /* 1s */
            RTC_MS = 0;
            if (++RTC_S >= 60) {          /* 1min */
                RTC_S = 0;
                if (++RTC_MIN >= 60) {    /* 1h */
                    RTC_MIN = 0;
                    if (++RTC_HOUR >= 24) {       /* 1day */
                        RTC_HOUR = 0;
                        if (++RTC_DAY > days_in_month(RTC_MON, RTC_YEAR)) { /* 1mon */
                            RTC_DAY = 1;
                            if (++RTC_MON > 12) { /* 1year */
                                RTC_MON = 1;
                                RTC_YEAR++;
                            }
                        }
                    }
                }
            }
        }
    }
    /* 清更新标志(写0清除) */
    TIM4->SR1 = (uint8_t)(~TIM4_IT_UPDATE);

    /* RF 轮询: 检测 DIO0/RxDone/TxDone (无事件时开销极小) */
    rf_poll();
}

void timer_init(void)
{
    rtc_sub_ms = 0;
    TICK_MS   = 0;
    RTC_MS   = 0;
    RTC_S    = 0;
    RTC_MIN  = 0;
    RTC_HOUR = 0;
    RTC_DAY  = 1;
    RTC_MON  = 1;
    RTC_YEAR = 0;

    /* 预分频8 -> 3MHz; ARR=499 -> 周期500 -> 6000Hz 精确(167us) @24MHz */
    TIM4->PSCR = (uint8_t)TIM4_PRESCALER_8;
    TIM4->ARR  = 499;
    /* 清更新标志 */
    TIM4->SR1  = (uint8_t)(~TIM4_IT_UPDATE);
    /* 使能更新中断 */
    TIM4->IER |= TIM4_IT_UPDATE;
    /* 使能计数 */
    TIM4->CR1 |= TIM4_CR1_CEN;
}

/* ==================================================================
 * RTC 软接口 (原子)
 * __critical 块: 进入关全局中断, 退出自动恢复原中断状态,
 * ================================================================== */

/* ---- 批量读取 ---- */
void rtc_get_time(RTC_Time *t)
{
    __critical {
        t->ms   = RTC_MS;
        t->s    = RTC_S;
        t->min  = RTC_MIN;
        t->hour = RTC_HOUR;
        t->day  = RTC_DAY;
        t->mon  = RTC_MON;
        t->year = RTC_YEAR;
    }
}

/* ---- 批量写入 (调用方需保证数值合法) ---- */
void rtc_set_time(const RTC_Time *t)
{
    __critical {
        RTC_MS   = t->ms;
        RTC_S    = t->s;
        RTC_MIN  = t->min;
        RTC_HOUR = t->hour;
        RTC_DAY  = t->day;
        RTC_MON  = t->mon;
        RTC_YEAR = t->year;
    }
}

/* ---- 单一读取 ---- */
uint16_t rtc_get_ms(void)
{
    uint16_t v;
    __critical { v = RTC_MS; }
    return v;
}
uint8_t rtc_get_s(void)
{
    uint8_t v;
    __critical { v = RTC_S; }
    return v;
}
uint8_t rtc_get_min(void)
{
    uint8_t v;
    __critical { v = RTC_MIN; }
    return v;
}
uint8_t rtc_get_hour(void)
{
    uint8_t v;
    __critical { v = RTC_HOUR; }
    return v;
}
uint8_t rtc_get_day(void)
{
    uint8_t v;
    __critical { v = RTC_DAY; }
    return v;
}
uint8_t rtc_get_mon(void)
{
    uint8_t v;
    __critical { v = RTC_MON; }
    return v;
}
uint8_t rtc_get_year(void)
{
    uint8_t v;
    __critical { v = RTC_YEAR; }
    return v;
}

/* ---- 单一写入 (调用方需保证数值合法) ---- */
void rtc_set_ms(uint16_t v)
{
    __critical { RTC_MS = v; }
}
void rtc_set_s(uint8_t v)
{
    __critical { RTC_S = v; }
}
void rtc_set_min(uint8_t v)
{
    __critical { RTC_MIN = v; }
}
void rtc_set_hour(uint8_t v)
{
    __critical { RTC_HOUR = v; }
}
void rtc_set_day(uint8_t v)
{
    __critical { RTC_DAY = v; }
}
void rtc_set_mon(uint8_t v)
{
    __critical { RTC_MON = v; }
}
void rtc_set_year(uint8_t v)
{
    __critical { RTC_YEAR = v; }
}
