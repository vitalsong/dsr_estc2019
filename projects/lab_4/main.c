#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include "ledctrl.h"

#define SWITCH_DELAY (100000)

//-------------------------------------------------------------------------------------------
static void _DirtyDelay(void)
{
    int i;
    for (i = 0; i < SWITCH_DELAY; i++);
}

static uint32_t g_led = 0;

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
    PinGroup g;
    g.r_pin = GPIO_Pin_8;
    g.g_pin = GPIO_Pin_9;
    g.b_pin = GPIO_Pin_10;
    g_led = LedCtrl_Config(GPIOA, g);

    LedColor color;
    color.r = 0;
    color.g = 0;
    color.b = 255;
    color.a = 255;
    LedCtrl_SetStable(g_led, color);

    while(1)
    {
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
