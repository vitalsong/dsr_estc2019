#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include "ledctrl.h"

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
static void _InitButtons()
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
static void _OnClickButton1()
{
    //...
}

//-------------------------------------------------------------------------------------------
static void _OnClickButton2()
{
    //...
}

//-------------------------------------------------------------------------------------------
int main(void)
{
    //init clock
    _InitClock();

    //init user buttons
    _InitButtons();

    //init led
    LedCtrl_On(0);

    while(1)
    {
        if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0)) {
            _OnClickButton1();
        }

        if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)) {
            _OnClickButton2();
        }

        //delay
        //...
    }
}
