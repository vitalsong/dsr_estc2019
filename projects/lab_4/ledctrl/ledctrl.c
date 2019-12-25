#include "ledctrl.h"
#include <assert.h>

#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>

//------------------------------------------------------------------------------------------------------------------
struct _LedState
{
    int8_t active;
    GPIO_TypeDef* GPIOx;
    PinGroup group;
    enum LedMode mode;
    LedColor color;
    TIM_TypeDef* tim;
};

//------------------------------------------------------------------------------------------------------------------
#define FD_TB_LEN (8)
static _LedState g_fdtb[FD_TB_LEN];

//------------------------------------------------------------------------------------------------------------------
static void _InitLeds(GPIO_TypeDef* GPIOx, PinGroup group)
{
    GPIO_InitTypeDef port;
    uint32_t gpio_pin = (1L << group.r_pin) | (1L << group.g_pin) | (1L << group.b_pin);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&port);
    port.GPIO_Pin = gpio_pin;
    port.GPIO_Mode = GPIO_Mode_AF;
    port.GPIO_Speed = GPIO_Speed_50MHz;
    port.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOx, &port);
    GPIO_PinAFConfig(GPIOx, group.r_pin, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOx, group.g_pin, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOx, group.b_pin, GPIO_AF_TIM1);
}

//------------------------------------------------------------------------------------------------------------------
static void _InitTimer(TIM_TypeDef* tim)
{
    TIM_TimeBaseInitTypeDef timer;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 840 - 1;
    timer.TIM_Period = 255;
    timer.TIM_ClockDivision = 0;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &timer);
}

//------------------------------------------------------------------------------------------------------------------
static void _InitPwm(TIM_TypeDef* tim)
{
    TIM_OCInitTypeDef pwm;
    TIM_OCStructInit(&pwm);
    pwm.TIM_OCMode = TIM_OCMode_PWM1;
    pwm.TIM_OutputState = TIM_OutputState_Enable;
    pwm.TIM_Pulse = 0;
    pwm.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OC1Init(tim, &pwm);
    TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);

    TIM_OC2Init(tim, &pwm);
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);

    TIM_OC3Init(tim, &pwm);
    TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(tim, ENABLE);
    TIM_Cmd(tim, ENABLE);
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_Init(void)
{
    int i;
    for (i=0; i < FD_TB_LEN; ++i) {
        g_fdtb[i].active = 0;
    }
}

//------------------------------------------------------------------------------------------------------------------
static LedFd _GetFreeFd(void)
{
    int i;
    for (i=0; i < FD_TB_LEN; ++i) {
        if (!g_fdtb[i].active) {
            return &g_fdtb[i];
        }
    }

    return 0;
}

//------------------------------------------------------------------------------------------------------------------
LedFd LedCtrl_Config(GPIO_TypeDef* GPIOx, PinGroup group, TIM_TypeDef* tim)
{
    _InitLeds(GPIOx, group);
    _InitTimer(tim);
    _InitPwm(tim);

    LedFd led = _GetFreeFd();
    led->group = group;
    led->GPIOx = GPIOx;
    led->color = 0;
    led->active = 1;
    led->tim = tim;
    led->mode = LED_MODE_STABLE;
    return led;
}

//------------------------------------------------------------------------------------------------------------------
static void _SetColor(TIM_TypeDef* tim, LedColor color)
{
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = (color) & 0xFF;
    tim->CCR1 = r;
    tim->CCR2 = g;
    tim->CCR3 = b;
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_On(LedFd led)
{
    _SetColor(led->tim, led->color);
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_Off(LedFd led)
{
    _SetColor(led->tim, 0);
}

//------------------------------------------------------------------------------------------------------------------
enum LedMode LedCtrl_GetMode(LedFd led)
{
    return led->mode;
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_SetStable(LedFd led, LedColor color)
{
    led->color = color;
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_SetPulseFreq(LedFd led, LedColor color, int freq)
{
    //...
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_SetGradientMode(LedFd led)
{
    //...
}