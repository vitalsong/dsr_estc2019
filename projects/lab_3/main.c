#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>

#define PERIOD 1000
#define SWITCH_DELAY (1000000)

//-------------------------------------------------------------------------------------------
static void _DirtyDelay()
{
    int i;
    for (i = 0; i < SWITCH_DELAY; i++);
}

//-------------------------------------------------------------------------------------------
int main(void)
{
    uint16_t TIM_Pulse = 0;
    GPIO_InitTypeDef port;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef pwm;

    //init leds
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&port);
    port.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    port.GPIO_Mode = GPIO_Mode_OUT;
    port.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &port);

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
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 720;
    timer.TIM_Period = PERIOD;
    timer.TIM_ClockDivision = 0;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &timer);

    //init pwm
    TIM_OCStructInit(&pwm);
    pwm.TIM_OCMode = TIM_OCMode_PWM1;
    pwm.TIM_OutputState = TIM_OutputState_Enable;
    pwm.TIM_Pulse = 10;
    pwm.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &pwm);

    TIM_Cmd(TIM4, ENABLE);

    while(1)
    {
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0) == 0) {
            if (TIM_Pulse < PERIOD) {
                TIM_Pulse++;
            }
            TIM4->CCR1 = TIM_Pulse;
        }

        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1) == 0) {
            if (TIM_Pulse > 0) {
                TIM_Pulse--;
            }
            TIM4->CCR1 = TIM_Pulse;
        }

        _DirtyDelay();
    }
}
