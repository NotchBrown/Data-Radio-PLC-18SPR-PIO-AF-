#include <Arduino.h>
#include "timer.h"
#include "spi.h"
#include "dio.h"
#include "adc.h"
#include "dac.h"
#include "uart3.h"
#include "mode1.h"
#include "mode2.h"
#include "mode3.h"
#include "mode4.h"

/* ------------------------------------------------------------------
 * 1. 时钟: 使用外部无源晶振 HSE 24MHz (OSCIN/OSCOUT)
 *    - 前提: OPT7 WAITSTATE=1 (Flash 等待状态, fCPU>16MHz 必须, 见 doc/clock.md)
 *    - 带超时保护: 若 HSE 未就绪(晶振/负载电容问题)则保持 HSI, 不死机
 * ------------------------------------------------------------------ */
static void clock_init(void)
{
    uint16_t guard = 0;

    CLK_HSECmd(ENABLE);                                   /* 使能 HSE 24MHz */
    while (CLK_GetFlagStatus(CLK_FLAG_HSERDY) == RESET) { /* 等 HSE 就绪     */
        if (++guard > 100000) return;                     /* 超时: 保持 HSI  */
    }

    /* 切主时钟源到 HSE 24MHz (自动切换, 切换完成后关当前源) */
    CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE,
                          DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);

    CLK_HSICmd(DISABLE);                                  /* 关 HSI 省电    */
}

/* ------------------------------------------------------------------
 * LED / 拨码 (func.md: LED 推挽高电平点亮; 拨码使能=低)
 *   RUN_LED=PA6, DEBUG_LED=PH0, SYSTEM_LED=PH1
 *   RUN_CTRL=PH2, DEBUG_CTRL=PH3
 * ------------------------------------------------------------------ */
#define LED_RUN_ON()      (GPIOA->ODR |= 0x40)                    /* PA6 */
#define LED_RUN_OFF()     (GPIOA->ODR &= (uint8_t)~0x40)
#define LED_DEBUG_ON()    (GPIOH->ODR |= 0x01)                    /* PH0 */
#define LED_DEBUG_OFF()   (GPIOH->ODR &= (uint8_t)~0x01)
#define LED_SYSTEM_ON()   (GPIOH->ODR |= 0x02)                    /* PH1 */
#define SW_DEBUG_ON()     (!(GPIOH->IDR & 0x08))                  /* PH3 低=使能 */
#define SW_RUN_ON()       (!(GPIOH->IDR & 0x04))                  /* PH2 低=使能 */

static void led_sw_init(void)
{
    /* LED 推挽输出, 初始灭(低) */
    GPIO_Init(GPIOA, GPIO_PIN_6, GPIO_MODE_OUT_PP_HIGH_SLOW);
    GPIO_Init(GPIOH, GPIO_PIN_0 | GPIO_PIN_1, GPIO_MODE_OUT_PP_HIGH_SLOW);
    GPIO_WriteLow(GPIOA, GPIO_PIN_6);
    GPIO_WriteLow(GPIOH, GPIO_PIN_0 | GPIO_PIN_1);

    /* 拨码上拉输入 (未使能=高, 使能=低) */
    GPIO_Init(GPIOH, GPIO_PIN_2 | GPIO_PIN_3, GPIO_MODE_IN_PU_NO_IT);
}

/* ------------------------------------------------------------------
 * setup(): 初始化顺序
 *   1. 时钟   -- 必须最先: TIM4/UART3 时序都依赖实际主频
 *   2. TIM4   -- 1ms 节拍(12kHz/83us), 按 24MHz 配预分频并开中断, 紧跟时钟
 *   3. SPI    -- 软件 NSS + 各片选引脚
 *   4. DIO    -- DI 浮空输入, DO 推挽输出高
 *   5. ADC    -- ADC2 10bit 初始化
 *   6. DAC    -- AD5314 初始化
 *   7. LED/拨码 -- 指示灯 + 模式拨码
 *   8. UART3  -- 8N1 收发 + RX 中断 (见 uart3.c)
 * ------------------------------------------------------------------ */
void setup()
{
    clock_init();    /* 1. 16MHz HSI        */
    timer_init();    /* 2. TIM4 1ms 节拍中断 */
    spi_init();      /* 3. SPI 主控 + 软件NSS(见 spi.c) */
    dio_init();      /* 4. DI/DO 初始化      */
    adc_init();      /* 5. ADC2 10bit        */
    dac_init();      /* 6. AD5314 SPI        */
    led_sw_init();   /* 7. LED + 拨码        */
    uart3_init();    /* 8. UART3 工作模式    */
    uart3_send_id(); /* 9. 上电上报 MCU ID 帧 */
}

/* ------------------------------------------------------------------
 * loop(): 顺序 = 改变LED -> 检测拨码 -> 运行对应模式
 *   模式号 = 1 + RUN*2 + DEBUG  (拨码使能=1, 见 doc/func.md 模式表)
 *     模式1 DEBUG=0 RUN=0 : 只读配置
 *     模式2 DEBUG=1 RUN=0 : 读写配置
 *     模式3 DEBUG=0 RUN=1 : 远程发射 (待实现)
 *     模式4 DEBUG=1 RUN=1 : 本机直通 (DI->DO, AI->AO)
 * ------------------------------------------------------------------ */
void loop()
{
    uint8_t dbg, run, mode;

    /* 1. 先读拨码 (同周期内保持一致) */
    dbg = SW_DEBUG_ON();
    run = SW_RUN_ON();

    /* 2. 改变 LED: 拨码使能=灯亮 */
    if (dbg) LED_DEBUG_ON(); else LED_DEBUG_OFF();
    if (run) LED_RUN_ON();   else LED_RUN_OFF();

    /* 3. 检测拨码 -> 模式号 */
    mode = (uint8_t)(1 + (run ? 2 : 0) + (dbg ? 1 : 0));

    /* 4. switch 跳转到对应模式的周期性子程序 */
    switch (mode) {
    case 1: mode1_run(); break;   /* 只读配置   */
    case 2: mode2_run(); break;   /* 读写配置   */
    case 3: mode3_run(); break;   /* 远程发射   */
    case 4: mode4_run(); break;   /* 本机直通   */
    }
}

/* ==================================================================
 * CPU 异常处理 (TRAP): 非法指令/软件陷阱/堆栈错误
 * SYSTEM_LED 常亮并停机 (等看门狗复位)
 * ================================================================== */
void TRAP_IRQHandler(void) __trap
{
    LED_SYSTEM_ON();     /* SYSTEM_LED 常亮 */
    sim();               /* 关中断 (stm8s.h, __asm__("sim")) */
    for (;;) ;           /* 死循环           */
}