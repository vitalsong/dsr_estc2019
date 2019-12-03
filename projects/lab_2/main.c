#include "main.h"
#include "stm32f4xx_rcc.h"

//-------------------------------------------------------------------------------------------
//BLINKING LED WITH TIMER INTERRUPT
//-------------------------------------------------------------------------------------------

#define SYSCLK (84000000)
#define AHB_PRESC (1)
#define APB1_PRESC (4)
#define APBX (1)
#define TIM_CLOCK ((SYSCLK) / (AHB_PRESC) / (APB1_PRESC) * (APBX))
#define TIM_PRESCALER (8400)
#define CLK_PER_SEC ((TIM_CLOCK) / (TIM_PRESCALER))
#define BLINK_PERIOD_MSEC (1000)
#define BLINK_PERIOD_CLK ((CLK_PER_SEC) * ((BLINK_PERIOD_MSEC) / 1000))

//-------------------------------------------------------------------------------------------
static volatile uint16_t g_current_led = GPIO_Pin_8;

//-------------------------------------------------------------------------------------------
static void _NextLedColor(void)
{
    GPIO_ToggleBits(GPIOA, g_current_led);

    switch (g_current_led)
    {
    case GPIO_Pin_8:
        g_current_led = GPIO_Pin_9;
        break;

    case GPIO_Pin_9:
        g_current_led = GPIO_Pin_10;
        break;

    case GPIO_Pin_10:
        g_current_led = GPIO_Pin_8;
        break;

    default:
        break;
    }

    GPIO_ToggleBits(GPIOA, g_current_led);
}

//-------------------------------------------------------------------------------------------
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        _NextLedColor();
    }
}

//-------------------------------------------------------------------------------------------
static void _InitTimerInterrupt(void)
{
    NVIC_InitTypeDef def;
    def.NVIC_IRQChannel = TIM2_IRQn;
    def.NVIC_IRQChannelPreemptionPriority = 0;
    def.NVIC_IRQChannelSubPriority = 1;
    def.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&def);
}

//-------------------------------------------------------------------------------------------
static void _InitClock(void)
{
    //start HSE
    RCC_HSEConfig(RCC_HSE_ON);
    RCC_WaitForHSEStartUp();
    RCC_PLLConfig(RCC_PLLSource_HSE, 4, 336, 8, 4);     //84MHz
    RCC_PLLCmd(ENABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}

//-------------------------------------------------------------------------------------------
static void _InitLed(void)
{
    GPIO_InitTypeDef param;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    param.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    param.GPIO_Mode = GPIO_Mode_OUT;
    param.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &param);

    GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_RESET);
}

//-------------------------------------------------------------------------------------------
static void _InitTimer(void)
{
    TIM_TimeBaseInitTypeDef param;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    param.TIM_Prescaler = TIM_PRESCALER - 1;
    param.TIM_CounterMode = TIM_CounterMode_Up;
    param.TIM_Period = BLINK_PERIOD_CLK - 1;
    param.TIM_ClockDivision = TIM_CKD_DIV1;
    param.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &param);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

//-------------------------------------------------------------------------------------------
static void _BlinkByTimerCounter()
{
    _InitClock();
    _InitLed();

    while(1)
    {
        uint32_t value = TIM_GetCounter(TIM2);
        if (value == BLINK_PERIOD_CLK) {
            _NextLedColor();
        }
    }
}

//-------------------------------------------------------------------------------------------
static void _BlinkByTimerInterrupt()
{
    _InitClock();
    _InitLed();
    _InitTimer();
    _InitTimerInterrupt();

    while(1) {
        //nothing to do
    }
}

//-------------------------------------------------------------------------------------------
int main(void)
{
#if defined(LAB2_BLINK_BY_INTERRUPT)
    _BlinkByTimerInterrupt();
#elif defined(LAB2_BLINK_BY_COUNTER)
    _BlinkByTimerCounter();
#else
    _BlinkByTimerInterrupt();
#endif
}
