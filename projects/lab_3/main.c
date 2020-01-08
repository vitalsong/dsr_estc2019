#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>

#define SWITCH_DELAY (500000)

#define SYS_CLK (84000000)
#define TIM_FREQ (1000)
#define PWM_FREQ (100)
#define ARR_VAL (TIM_FREQ/PWM_FREQ)
#define PSC (SYS_CLK/TIM_FREQ)

static uint16_t g_pulse;
static uint8_t g_color;

//-------------------------------------------------------------------------------------------
static void _DirtyDelay()
{
    int i;
    for (i = 0; i < SWITCH_DELAY; i++);
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
static void _UpdateLeds(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    switch (g_color)
    {
    case 0:
        TIM1->CCR1 = g_pulse;
        break;

    case 1:
        TIM1->CCR2 = g_pulse;
        break;

    case 2:
        TIM1->CCR3 = g_pulse;
        break;
    }
}

//-------------------------------------------------------------------------------------------
static void _NextColor(void)
{
    g_color = (g_color + 1) % 3;
    _UpdateLeds();
}

//-------------------------------------------------------------------------------------------
static void _NextBrightness(void)
{
    //increase by 20%
    g_pulse = (g_pulse + ARR_VAL/5) % ARR_VAL;
    _UpdateLeds();
}

//-------------------------------------------------------------------------------------------
int main(void)
{
    GPIO_InitTypeDef port;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef pwm;

    g_pulse = 0;
    g_color = 0;

    //init clock
    _InitClock();

    //init leds
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&port);
    port.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    port.GPIO_Mode = GPIO_Mode_AF;
    port.GPIO_Speed = GPIO_Speed_50MHz;
    port.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &port);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);

    //init user buttons
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_StructInit(&port);
    port.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    port.GPIO_Mode = GPIO_Mode_IN;
    port.GPIO_OType = GPIO_OType_PP;
    port.GPIO_Speed = GPIO_Speed_100MHz;
    port.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &port);

    //init timer
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = PSC - 1;
    timer.TIM_Period = ARR_VAL - 1;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &timer);

    //init pwm
    TIM_OCStructInit(&pwm);
    pwm.TIM_OCMode = TIM_OCMode_PWM1;
    pwm.TIM_OutputState = TIM_OutputState_Enable;
    pwm.TIM_Pulse = 0;
    pwm.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OC1Init(TIM1, &pwm);
    TIM_OC2Init(TIM1, &pwm);
    TIM_OC3Init(TIM1, &pwm);
    
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    while(1)
    {
        if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0)) {
            _NextBrightness();
            _DirtyDelay();
        }

        if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)) {
            _NextColor();
            _DirtyDelay();
        }
    }
}
