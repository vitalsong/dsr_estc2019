#include "main.h"
#include "stm32f4xx_rcc.h"

#define SWITCH_DELAY (1000000)

//-------------------------------------------------------------------------------------------
static void _dirty_delay()
{
    int i;
    for (i = 0; i < SWITCH_DELAY; i++);
}

//-------------------------------------------------------------------------------------------
static void _switch_pll(int status)
{
    //PLL settings
    RCC_DeInit();
    RCC_HSICmd(ENABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    while (RCC_GetSYSCLKSource() != 0x00);

    //start HSE
    RCC_HSEConfig(RCC_HSE_ON);
    RCC_WaitForHSEStartUp();

    //start PLL
    if (status) {
        RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 4, 4);     //84 MHz
    }
    else {
        RCC_PLLConfig(RCC_PLLSource_HSE, 16, 336, 4, 4);    //42 MHz
    }

    RCC_PLLCmd(ENABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}

//-------------------------------------------------------------------------------------------
int main(void)
{
    int is_half = 1;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* LEDs array to toggle between them */
    /* LED to toggle during iteration */
    uint16_t current_led = GPIO_Pin_8;

    /* Enable peripheral clock for LEDs port */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    /* Init LEDs */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Init Buttons */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    _switch_pll(1);

    GPIO_SetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9);
    while (1)
    {
        GPIO_ResetBits(GPIOA, current_led);
        _dirty_delay();

        GPIO_SetBits(GPIOA, current_led);
        _dirty_delay();

        //if button clicked
        if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0))
        {
            current_led = (current_led == GPIO_Pin_8) ? (GPIO_Pin_9) : (GPIO_Pin_8);
            GPIO_SetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9);

            if (is_half) {
                _switch_pll(0);
                is_half = 0;
            }
            else {
                _switch_pll(1);
                is_half = 1;
            }

            _dirty_delay();
        }
    }
}
