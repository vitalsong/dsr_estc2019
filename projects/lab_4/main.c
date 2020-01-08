#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include "ledctrl/ledctrl.h"

#define SWITCH_DELAY (100000)

//-------------------------------------------------------------------------------------------
static void _DirtyDelay(void)
{
    int i;
    for (i = 0; i < SWITCH_DELAY; i++);
}

static LedFd g_led = 0;

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
static void _InitButtons(void)
{
    GPIO_InitTypeDef port;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_StructInit(&port);
    port.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    port.GPIO_Mode = GPIO_Mode_IN;
    port.GPIO_OType = GPIO_OType_PP;
    port.GPIO_Speed = GPIO_Speed_100MHz;
    port.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &port);
}

//-------------------------------------------------------------------------------------------
static void _OnClickButton1(void)
{
    LedCtrl_On(g_led);
}

//-------------------------------------------------------------------------------------------
static void _OnClickButton2(void)
{
    LedCtrl_Off(g_led);
}

//-------------------------------------------------------------------------------------------
int main(void)
{
    //init clock
    _InitClock();

    //init user buttons
    _InitButtons();

    //init module
    LedCtrl_Init();

    //init led
    PinGroup g = {GPIO_PinSource8, GPIO_PinSource9, GPIO_PinSource10};
    g_led = LedCtrl_Config(GPIOA, g, TIM1);

    //set stable color
    LedCtrl_SetStable(g_led, 0xCD5C5C);
    //LedCtrl_SetPulseFreq(g_led, 0xCD5C5C, 5);
    //LedCtrl_SetGradientMode(g_led);

    //enable
    LedCtrl_On(g_led);

    while(1)   {
        if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0)) {
            _OnClickButton1();
        }

        if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)) {
            _OnClickButton2();
        }

        //delay
        _DirtyDelay();
    }
}
