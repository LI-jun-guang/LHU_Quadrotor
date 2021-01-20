#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"

#define RGB(G,R,B) TIM8->CCR1=G; TIM8->CCR2=R; TIM8->CCR3=B

