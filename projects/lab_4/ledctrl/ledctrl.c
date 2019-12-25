#include "ledctrl.h"
#include <assert.h>

typedef struct
{
    int8_t active;
    GPIO_TypeDef* GPIOx;
    PinGroup group;
    enum LedMode mode;
}
LedInfo;

#define FD_TB_LEN (8)

static LedInfo g_fdtb[FD_TB_LEN];
static uint32_t g_tbidx = 0;

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_Init(void)
{
    int i;
    g_tbidx = 0;
    for (i=0; i < FD_TB_LEN; ++i) {
        g_fdtb[i].active = 0;
    }
}

//------------------------------------------------------------------------------------------------------------------
LedFd LedCtrl_Config(GPIO_TypeDef* GPIOx, PinGroup group)
{
    GPIO_InitTypeDef led;
    LedFd fd = 0;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    led.GPIO_Pin = group.b_pin | group.g_pin | group.r_pin;
    led.GPIO_Mode = GPIO_Mode_OUT;
    led.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &led);
    GPIO_SetBits(GPIOx, group.b_pin | group.g_pin | group.r_pin);

    fd = g_tbidx++;
    g_fdtb[fd].group = group;
    g_fdtb[fd].GPIOx = GPIOx;
    return fd;
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_On(LedFd led)
{
    LedInfo info = g_fdtb[led];
    PinGroup g = info.group;
    uint16_t pin = g.r_pin | g.g_pin | g.b_pin;
    GPIO_SetBits(info.GPIOx, pin);
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_Off(LedFd led)
{
    LedInfo info = g_fdtb[led];
    PinGroup g = info.group;
    uint16_t pin = g.r_pin | g.g_pin | g.b_pin;
    GPIO_ResetBits(info.GPIOx, pin);
}

//------------------------------------------------------------------------------------------------------------------
enum LedMode LedCtrl_GetMode(LedFd led)
{
    LedInfo fd = g_fdtb[led];
    return fd.mode;
}

//------------------------------------------------------------------------------------------------------------------
void LedCtrl_SetStable(LedFd led, LedColor color)
{
    //...
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