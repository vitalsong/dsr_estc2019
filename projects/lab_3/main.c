#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>

#define PERIOD 100
#define SWITCH_DELAY (100000)

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
int main(void)
{
    uint16_t TIM_Pulse = 0;
    GPIO_InitTypeDef port;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef pwm;

    //init clock
    _InitClock();

    //init leds
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&port);
    port.GPIO_Pin = GPIO_Pin_8;
    port.GPIO_Mode = GPIO_Mode_AF;
    port.GPIO_Speed = GPIO_Speed_50MHz;
    port.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &port);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

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
    timer.TIM_Prescaler = 8400 - 1;
    timer.TIM_Period = PERIOD;
    timer.TIM_ClockDivision = 0;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &timer);

    //init pwm
    TIM_OCStructInit(&pwm);
    pwm.TIM_OCMode = TIM_OCMode_PWM1;
    pwm.TIM_OutputState = TIM_OutputState_Enable;
    pwm.TIM_Pulse = PERIOD / 2;
    pwm.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init(TIM1, &pwm);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    while(1)
    {
        if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0)) {
            if (TIM_Pulse > 0) {
                --TIM_Pulse;
            }
            TIM1->CCR1 = TIM_Pulse;
        }

        if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)) {
            if (TIM_Pulse < PERIOD) {
                ++TIM_Pulse;
            }
            TIM1->CCR1 = TIM_Pulse;
        }

        _DirtyDelay();
    }
}
