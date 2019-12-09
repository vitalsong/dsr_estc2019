#ifndef LEDCTRL_H
#define LEDCTRL_H

#ifdef __cplusplus
extern "C" {
#endif

//led mode
enum LedMode
{
    LED_MODE_STABLE,
    LED_MODE_BLINK,
    LED_MODE_GRADIENT
};

//led color struct
typedef struct 
{
    int r;  //red [0 - 255]
    int g;  //green [0 - 255]
    int b;  //blue [0 - 255]
    int a;  //brightness [0 - 255]
}
LedColor;

//enable led
void LedCtrl_On(int led_num);

//disable led
void LedCtrl_Off(int led_num);

//get current mode
enum LedMode LedCtrl_GetMode(int led_num);

//set stable mode
void LedCtrl_SetStable(int led_num, const LedColor* color);

//set pulse mode
void LedCtrl_SetPulseFreq(int led_num, const LedColor* color, int freq);

//set gradient mode
void LedCtrl_SetGradientMode(int led_num);

#ifdef __cplusplus
}
#endif

#endif  /* LEDCTRL_H */