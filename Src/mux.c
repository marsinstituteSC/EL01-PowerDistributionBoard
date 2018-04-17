#include "mux.h"

void MUX_select(uint8_t select){
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)!=(select&0b0001)){
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	}
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)!=(select&0b0010)){
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_2);
	}
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)!=(select&0b0100)){
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3);
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)!=(select&0b1000)){
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
	}
}

