
#ifndef MOTORDRIVER_H_
#define MOTORDRIVER_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef union {
  struct {
    uint8_t mikrobus;
    uint16_t driveFilter;
    uint8_t startByte;
    uint8_t antallBytes;
    uint16_t pulseRev;
    uint16_t revPerSekund;
    uint8_t unsignedsigned;
    uint8_t gpiomode;
  } oppsett;
  uint8_t array[11];
} MotorSetting;


#ifdef motorVF
uint16_t prosent = 140/100;
#endif

#ifdef motorVM
uint16_t prosent = 120/100;
#endif

#ifdef motorVB
uint16_t prosent = 100/100;
#endif

#ifdef motorHF
uint16_t prosent = 80/100;
#endif

#ifdef motorHM
uint16_t prosent = 60/100;
#endif

#ifdef motorHB
uint16_t prosent = 40/100;
#endif

void PWM_Set_Frekvens(uint16_t fart);


#endif /* MOTORDRIVER_H_ */
