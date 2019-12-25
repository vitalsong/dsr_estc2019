#pragma once

#include <stm32f4xx.h>

#define COLOR_RED (0xFF0000)
#define COLOR_GREEN (0x00FF00)
#define COLOR_BLUE (0x0000FF)

//led mode
enum LedMode
{
    LED_MODE_STABLE,
    LED_MODE_BLINK,
    LED_MODE_GRADIENT
};

//led color (example 0xFF0000 for red)
typedef uint32_t LedColor;

typedef struct 
{
    uint8_t r_pin;
    uint8_t g_pin;
    uint8_t b_pin;
}
PinGroup;

typedef struct _LedState _LedState;
typedef _LedState* LedFd;

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
 * @param tim Timer for pwd (see ref manual)
 * @return LedFd Led descriptor
 */
LedFd LedCtrl_Config(GPIO_TypeDef* GPIOx, PinGroup group, TIM_TypeDef* tim);

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

