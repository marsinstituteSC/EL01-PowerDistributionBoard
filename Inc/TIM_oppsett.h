
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIM_OPPSETT_H_
#define TIM_OPPSETT_H_
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

extern TIM_HandleTypeDef htim;

extern void _Error_Handler(char *, int);

void TIMER_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);



#endif

