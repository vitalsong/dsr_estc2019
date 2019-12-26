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
    int32_t period;
    int32_t counter;
    uint8_t on;
};

#define GRADIENT_COLOR_SIZE (11)
const uint32_t GRADIENT_COLOR[GRADIENT_COLOR_SIZE] = 
{0xE5004B, 0xCE1350, 0xB72655, 0xA0395A, 0x894C60, 0x725F65, 0x5B726A, 0x448570, 0x2D9875, 0x16AB7A, 0x00BF80};

//------------------------------------------------------------------------------------------------------------------
#define FD_TB_LEN (8)
static _LedState g_fdtb[FD_TB_LEN];

#define SYSCLK (84000000)
#define AHB_PRESC (1)
#define APB1_PRESC (4)
#define APBX (1)
#define TIM_CLOCK ((SYSCLK) / (AHB_PRESC) / (APB1_PRESC) * (APBX))
#define TIM_PRESCALER (8400)
#define CLK_PER_SEC ((TIM_CLOCK) / (TIM_PRESCALER))

//------------------------------------------------------------------------------------------------------------------
static void _SetColor(TIM_TypeDef* tim, LedColor color);
static void _LedToggle(LedFd led);
static void _InitUpdateTimer(void);
static void _InitLeds(GPIO_TypeDef* GPIOx, PinGroup group);
static void _InitTimer(TIM_TypeDef* tim);
static void _InitPwm(TIM_TypeDef* tim);
static LedFd _GetFreeFd(void);
static void _SetColor(TIM_TypeDef* tim, LedColor color);
static void _NextColor(LedFd led);

//------------------------------------------------------------------------------------------------------------------
static void _NextColor(LedFd led)
{
    led->color = (led->color + 1) % GRADIENT_COLOR_SIZE;
    _SetColor(led->tim, GRADIENT_COLOR[led->color]);
}

//------------------------------------------------------------------------------------------------------------------
static void _LedToggle(LedFd led)
{
    if (led->on) {
        led->on = 0;
        _SetColor(led->tim, 0x0);
    }
    else {
        led->on = 1;
        _SetColor(led->tim, led->color);
    }
}

//------------------------------------------------------------------------------------------------------------------
void TIM2_IRQHandler(void)
{
    uint16_t i;
    LedFd led;
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        for (i=0; i < FD_TB_LEN; ++i)
        {
            led = &g_fdtb[i];
            if (led->active && (led->mode != LED_MODE_STABLE))
            {
                led->counter += 1;
                if (led->counter >= led->period)
                {
                    led->counter = 0;
                    if (led->mode == LED_MODE_BLINK) {
                        _LedToggle(led);
                    }
                    else if (led->mode == LED_MODE_GRADIENT) {
                        _NextColor(led);
                    }
                }
            }
        }
    }
}

//------------------------------------------------------------------------------------------------------------------
static void _InitUpdateTimer(void)
{
    TIM_TimeBaseInitTypeDef tim;
    uint32_t period_clk = CLK_PER_SEC / 1000;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    tim.TIM_Prescaler = TIM_PRESCALER - 1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = period_clk - 1;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

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
    _InitUpdateTimer();

    LedFd led = _GetFreeFd();

    led->group = group;
    led->GPIOx = GPIOx;
    led->color = 0;
    led->active = 1;
    led->tim = tim;
    led->period = 0;
    led->counter = 0;
    led->mode = LED_MODE_STABLE;
    led->on = 0;

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
    led->mode = LED_MODE_STABLE;
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_SetPulseFreq(LedFd led, LedColor color, int freq)
{
    led->color = color;
    led->period = CLK_PER_SEC / freq;
    led->counter = 0;
    led->mode = LED_MODE_BLINK;
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_SetGradientMode(LedFd led)
{
    led->mode = LED_MODE_GRADIENT;
    led->counter = 0;
    led->period = 100;
}