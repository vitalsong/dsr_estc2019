#pragma once

#include <stm32f4xx.h>

//led mode
enum LedMode
{
    LED_MODE_STABLE,
    LED_MODE_BLINK,
    LED_MODE_GRADIENT
};

//led color struct
//TODO: use hex format instead?
typedef struct 
{
    uint8_t r;  //red [0 - 255]
    uint8_t g;  //green [0 - 255]
    uint8_t b;  //blue [0 - 255]
    uint8_t a;  //brightness [0 - 255]
}
LedColor;

typedef struct 
{
    uint16_t r_pin;
    uint16_t g_pin;
    uint16_t b_pin;
}
PinGroup;

typedef uint32_t LedFd;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Module init
 * @warning Must be called first
 */
void LedCtrl_Init(void);

/**
 * @brief Configurate led
 * @param GPIOx Gpio port
 * @param group Group of pin for control
 * @return LedFd Led descriptor
 */
LedFd LedCtrl_Config(GPIO_TypeDef* GPIOx, PinGroup group);

/**
 * @brief Enable led
 * @param led Led descriptor
 */
void LedCtrl_On(LedFd led);

/**
 * @brief Disable led
 * @param led Led descriptor
 */
void LedCtrl_Off(LedFd led);

/**
 * @brief Get current view mode
 * @param led Led descriptor
 * @return enum LedMode Current view mode
 */
enum LedMode LedCtrl_GetMode(LedFd led);

/**
 * @brief Set stable view mode
 * @param led Led descriptor
 * @param color Led color
 */
void LedCtrl_SetStable(LedFd led, LedColor color);

/**
 * @brief Set pulse mode
 * @param led Led descriptor
 * @param color Led color
 * @param freq Pulse freq
 */
void LedCtrl_SetPulseFreq(LedFd led, LedColor color, int freq);

/**
 * @brief Set gradient mode
 * @param led Led descriptor
 */
void LedCtrl_SetGradientMode(LedFd led);

#ifdef __cplusplus
}
#endif

