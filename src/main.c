#include <Arduino.h>
#include "timer.h"
#include "dio.h"

/* UART3 上位机配置口波特率(协议未定, 可改) */
#define UART3_BAUD  115200UL

/* ------------------------------------------------------------------
 * 1. 时钟: 切换到 24MHz 外部晶振 (HSE)
 *    init() 已把主时钟分频 CKDIVR 置 0, 切到 HSE 即 24MHz
 * ------------------------------------------------------------------ */
static void clock_init(void)
{
    CLK_HSECmd(ENABLE);                                   /* 使能外部晶振  */
    while (CLK_GetFlagStatus(CLK_FLAG_HSERDY) == RESET);  /* 等待振荡就绪  */
    CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE,
                          DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);
    CLK_HSICmd(DISABLE);                                  /* 关 HSI 省电   */
}

/* ------------------------------------------------------------------
 * 2. SPI: 取消硬件 NSS(软件从机管理), 设置所有软件 NSS(片选)引脚
 *    CS: GD25Q64=PF0, RA-01=PH6, AD5314=PH7 (推挽输出, 初始高)
 *    SCK=PC5/MOSI=PC6 输出, MISO=PC7 浮空输入
 * ------------------------------------------------------------------ */
static void spi_nss_init(void)
{
    /* 软件从机管理: 硬件 NSS 引脚(PE5/DI13)不再影响 SPI */
    SPI->CR2 |= SPI_CR2_SSM | SPI_CR2_SSI;
    SPI->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;   /* 主机并使能 */

    pinMode(SCK,  OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);

    pinMode(PF0, OUTPUT); digitalWrite(PF0, HIGH);  /* GD25Q64 CS */
    pinMode(PH6, OUTPUT); digitalWrite(PH6, HIGH);  /* RA-01 NSS  */
    pinMode(PH7, OUTPUT); digitalWrite(PH7, HIGH);  /* AD5314 SYNC*/
}

/* ------------------------------------------------------------------
 * 3. UART3: 上位机配置口 (PD5=TX, PD6=RX), 8N1 收发使能
 * ------------------------------------------------------------------ */
static void uart3_init(void)
{
    GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST); /* TX 推挽 */
    GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);      /* RX 浮空 */
    UART3_Init(UART3_BAUD, UART3_WORDLENGTH_8D, UART3_STOPBITS_1,
               UART3_PARITY_NO, UART3_MODE_TXRX_ENABLE);
}

/* ------------------------------------------------------------------
 * setup(): 初始化顺序
 *   1. 时钟   -- 必须最先: TIM4/UART3 时序都依赖实际主频
 *   2. TIM4   -- 1ms 节拍, 按 24MHz 重配预分频并开中断, 紧跟时钟
 *   3. SPI    -- 软件 NSS + 各片选引脚
 *   4. DIO    -- DI 浮空输入, DO 推挽输出高
 *   5. UART3  -- 8N1 收发 (波特率由实际时钟算出)
 * ------------------------------------------------------------------ */
void setup()
{
    clock_init();    /* 1. 24MHz HSE        */
    timer_init();    /* 2. TIM4 1ms 节拍中断 */
    spi_nss_init();  /* 3. SPI 软件 NSS      */
    dio_init();      /* 4. DI/DO 初始化      */
    uart3_init();    /* 5. UART3 工作模式    */
}

void loop()
{
    /* 后续: RF 收发、485 透传、遥测打包/解包 */
}