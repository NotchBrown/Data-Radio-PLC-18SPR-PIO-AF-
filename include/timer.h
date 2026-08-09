/*
 * timer.h - TIM4 系统节拍 + RTC
 * STM8S208MBT6 @ 16MHz 内部 HSI
 * TIM4 中断频率 8kHz(125us), 每 8 次=1ms(精确无漂移),
 * 中断里递增 RTC 时间变量
 *
 * 约定: 全局变量一律大写
 */
#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

/* 初始化 TIM4: 8kHz(125us)中断, 驱动 RTC */
void timer_init(void);

/* RTC 时间变量 (由 TIM4 中断递增) */
extern volatile uint16_t RTC_MS;    /* 毫秒 0..999 */
extern volatile uint8_t  RTC_S;     /* 秒   0..59  */
extern volatile uint8_t  RTC_MIN;   /* 分   0..59  */
extern volatile uint8_t  RTC_HOUR;  /* 时   0..23  */
extern volatile uint8_t  RTC_DAY;   /* 日   1..31  */
extern volatile uint8_t  RTC_MON;   /* 月   1..12  */
extern volatile uint8_t  RTC_YEAR;  /* 年   0..255 */

/* RTC 时间结构体(批量接口用), 字段与 RTC_* 全局一一对应 */
typedef struct {
    uint16_t ms;    /* 毫秒 0..999 */
    uint8_t  s;     /* 秒   0..59  */
    uint8_t  min;   /* 分   0..59  */
    uint8_t  hour;  /* 时   0..23  */
    uint8_t  day;   /* 日   1..31  */
    uint8_t  mon;   /* 月   1..12  */
    uint8_t  year;  /* 年   0..255 */
} RTC_Time;

/* ---- 批量接口 (原子: 内部 __critical 关中断) ---- */
void rtc_get_time(RTC_Time *t);          /* 读整个 RTC 到 *t  */
void rtc_set_time(const RTC_Time *t);    /* 用 *t 写整个 RTC  */

/* ---- 单一接口 (原子) ---- */
uint16_t rtc_get_ms(void);
uint8_t  rtc_get_s(void);
uint8_t  rtc_get_min(void);
uint8_t  rtc_get_hour(void);
uint8_t  rtc_get_day(void);
uint8_t  rtc_get_mon(void);
uint8_t  rtc_get_year(void);

void rtc_set_ms(uint16_t v);
void rtc_set_s(uint8_t v);
void rtc_set_min(uint8_t v);
void rtc_set_hour(uint8_t v);
void rtc_set_day(uint8_t v);
void rtc_set_mon(uint8_t v);
void rtc_set_year(uint8_t v);

#endif /* __TIMER_H */

