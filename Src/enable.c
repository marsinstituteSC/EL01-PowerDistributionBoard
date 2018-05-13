#include "enable.h"

void MODUS_select(uint8_t select) {

	switch (select) {

		case 0xFF:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET); // LED 2
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET); // LED 1

			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_SET); // Kjoremotor
	//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2, GPIO_PIN_SET); // Rotasjonsmotor
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET); // Kameramast
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4, GPIO_PIN_SET); // Arm/Gravemekanisme

			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6, GPIO_PIN_SET); // Drill
			break;
		case 0x00:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET); // LED 1
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET); // LED 2

			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_RESET); // Kjoremotor
		//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2, GPIO_PIN_RESET); // Rotasjonsmotor
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET); // Kameramast
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4, GPIO_PIN_RESET); // Arm/Gravemekanisme

			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6, GPIO_PIN_RESET); // Drill
			break;
		default:
			break;
	}

//	if(select&0xFF) {
//
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET); // LED 2
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET); // LED 1
//
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_SET); // Kjoremotor
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2, GPIO_PIN_SET); // Rotasjonsmotor
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET); // Kameramast
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4, GPIO_PIN_SET); // Arm/Gravemekanisme
//
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6, GPIO_PIN_SET); // Drill
//
////		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_1); // Kjoremotor
////		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2); // Rotasjonsmotor
////		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_3); // Kameramast
////		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_4); // Arm/Gravemekanisme
////
////		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6); // Drill
//
//	} else {
//
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET); // LED 1
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET); // LED 2
//
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_RESET); // Kjoremotor
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2, GPIO_PIN_RESET); // Rotasjonsmotor
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET); // Kameramast
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4, GPIO_PIN_RESET); // Arm/Gravemekanisme
//
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6, GPIO_PIN_RESET); // Drill
//	}

}


