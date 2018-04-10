#include "motorDriver.h"
#include "main.h"
#include "tim.h"
#include "math.h"

extern uint32_t fartkonst;
//extern uint16_t fart;



void PWM_Set_Frekvens(uint16_t tempfart){

//	bool retning = (tempfart>>15) & 0x01;
//	tempfart &= 0xEF;

//	if (retning==1) {
//		MOTOR_FRAM();
//	}
//	else{
//		MOTOR_BAK();
//	}

	if(tempfart >= 1){

		tempfart = 10500000000 / (tempfart*5826); //*128
		if (tempfart <=55){tempfart = 55;}
		if (tempfart >= 65000){tempfart = 65000;}

//		if((fartkonst<=0.5)||(fartkonst>=1.5)){
//			tempfart = tempfart;
//		}else if ((fartkonst<=-0.5)||(fartkonst>=-1.5)){
//			tempfart = tempfart;
//		}else
////		{
//			tempfart = (uint16_t) ((tempfart*fartkonst)/1000);
//		}
//		fart = tempfart;
		PWM_TIMER->ARR = tempfart;
		PWM_TIMER->CCR1 = (tempfart/2);
		PWM_TIMER->CR1 = 0x81;
//		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	}else{
//		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
//		MOTOR_DISABLE()
		PWM_TIMER->ARR = 10000;
		PWM_TIMER->CCR1 = 10000;
		PWM_TIMER->CR1 = 0x81;
	}
}

