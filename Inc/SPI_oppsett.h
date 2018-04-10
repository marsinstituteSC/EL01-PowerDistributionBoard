

#ifndef SPI_OPPSETT_H_
#define SPI_OPPSETT_H_
#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx_hal.h"
 #include "main.h"

 extern SPI_HandleTypeDef hspi;

 extern void _Error_Handler(char *, int);

 void SPI_CAN_Init(void);



#endif /* SPI_OPPSETT_H_ */
