#include "mux.h"

void MUX_select(uint8_t select){
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)!=(select&(0b0001))){ //s0
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
	}
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)!=((select&(0b0010))>>1)){ //s1
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_2);
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3);
	}
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)!=((select&(0b0100))>>2)){ //s3
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3);
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_2);
	}
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)!=((select&(0b1000))>>3)){ //s4
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	}
}

