#include "AvPaaKontroll.h"

void avPaaKontroll(uint8_t select) {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, (select&0b10000000)>>7); // motor forran
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, (select&0b01000000)>>6); // motor midten
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, (select&0b00100000)>>5); // motor bak
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, (select&0b00010000)>>4); // servo motor
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (select&0b00001000)>>3); // drill
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (select&0b00000100)>>2); // kamera
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, (select&0b00000010)>>1); // maskinmodul
}
