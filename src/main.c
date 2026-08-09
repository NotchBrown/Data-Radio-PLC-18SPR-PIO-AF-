#include <Arduino.h>
#include "timer.h"
#include "spi.h"
#include "dio.h"
#include "adc.h"
#include "dac.h"
#include "uart3.h"

/* ------------------------------------------------------------------
 * 1. 时钟: 使用内置 HSI 16MHz (init() 已配 HSIDIV1/CKDIVR=0),
 *    关闭外部晶振 HSE 省电 (本阶段不用外部晶振)
 * ------------------------------------------------------------------ */
static void clock_init(void)
{
    CLK_HSICmd(ENABLE);                                   /* 使能 HSI 16MHz */
    while (CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == RESET);  /* 等待就绪       */
    CLK_HSECmd(DISABLE);                                  /* 关 HSE 省电    */
}

/* ------------------------------------------------------------------
 * LED / 拨码 (func.md: LED 推挽高电平点亮; 拨码使能=低)
 *   RUN_LED=PA6, DEBUG_LED=PH0, SYSTEM_LED=PH1
 *   RUN_CTRL=PH2, DEBUG_CTRL=PH3
 * ------------------------------------------------------------------ */
#define LED_RUN_ON()      (GPIOA->ODR |= 0x40)                    /* PA6 */
#define LED_RUN_OFF()     (GPIOA->ODR &= (uint8_t)~0x40)
#define LED_SYSTEM_ON()   (GPIOH->ODR |= 0x02)                    /* PH1 */
#define SW_DEBUG_ON()     (!(GPIOH->IDR & 0x08))                  /* PH3 低=使能 */

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
 *   2. TIM4   -- 1ms 节拍(8kHz/125us), 按 16MHz 配预分频并开中断, 紧跟时钟
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

void loop()
{
    if (SW_DEBUG_ON()) {      /* DEBUG 拨码=使能: 发射模式 */
        uart3_enable(0);      /* 禁用 UART3 配置口      */
        LED_RUN_ON();         /* 运行发射: RUN LED 常亮  */
        uart3_tx_run();       /* 发射流程调度            */
    } else {                  /* 配置模式 */
        uart3_enable(1);
        LED_RUN_OFF();
        uart3_process();      /* UART3 上位机帧处理 */
    }

    /* 后续: RF 接收、485 透传、遥测打包/解包 */
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