/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "CANSPI.h"
#include "math.h"
#include "adc.h"
#include "mux.h"
#include "enable.h"
#include "MCP2515.h"
#include "AvPaaKontroll.h"

/* USER CODE BEGIN Includes */     
#define B(x) S_to_binary_(#x)
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId SettFartTaskHandle;
osThreadId AckermannTaskHandle;
osThreadId CANbehandlingHandle;
osThreadId ADCTaskHandle;
QueueHandle_t MeldingQueueHandle;
QueueHandle_t FartQueueHandle;
QueueHandle_t AckerQueueHandle;
QueueHandle_t EnableCTRLQueueHandle;
SemaphoreHandle_t ISRSemaHandle;


uint16_t teller2 = 0;
extern uint16_t tellertest;


uint32_t enablekontrolltest;


/* USER CODE BEGIN Variables */


/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void MX_ADC1_Init(void);



/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityRealtime, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SettFartTask */
  osThreadDef(SettFartTask, StartTask02, osPriorityNormal, 0, 128);
  SettFartTaskHandle = osThreadCreate(osThread(SettFartTask), NULL);

  /* definition and creation of AckermannTask */
  osThreadDef(AckermannTask, StartTask03, osPriorityRealtime, 0, 128);
  AckermannTaskHandle = osThreadCreate(osThread(AckermannTask), NULL);

  /* definition and creation of CANbehandling */
  osThreadDef(CANbehandling, StartTask04, osPriorityRealtime, 0, 128);
  CANbehandlingHandle = osThreadCreate(osThread(CANbehandling), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(ADCtask, StartTask05, osPriorityRealtime, 0, 128);
  ADCTaskHandle = osThreadCreate(osThread(ADCtask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of FartQueue */
//  osMessageQDef(FartQueue, 10, uint16_t);
//  FartQueueHandle = osMessageCreate(osMessageQ(FartQueue), NULL);
//
//  /* definition and creation of RadiusQueue */
//  osMessageQDef(RadiusQueue, 10, uint32_t);
//  RadiusQueueHandle = osMessageCreate(osMessageQ(RadiusQueue), NULL);
//
//  /* definition and creation of MeldingQueue */
//  osMessageQDef(MeldingQueue, 16, uCAN_MSG);
//  MeldingQueueHandle = osMessageCreate(osMessageQ(MeldingQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  MeldingQueueHandle = xQueueCreate(16,sizeof(uCAN_MSG));
  FartQueueHandle = xQueueCreate(16,sizeof(uint16_t));
  AckerQueueHandle = xQueueCreate(16,sizeof(uint32_t));
  EnableCTRLQueueHandle = xQueueCreate(32, sizeof(uint32_t));

  ISRSemaHandle = xSemaphoreCreateBinary();
//  ADCSemaHandle = xSemaphoreCreateBinary();

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}


/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	  vTaskDelete(defaultTaskHandle);
	  MCP2515_WriteByte(MCP2515_CANINTF,0x00);

  }
  /* USER CODE END StartDefaultTask */
}

/* StartTask02 function */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */

  for(;;)
  {


  }
  /* USER CODE END StartTask02 */
}

/* StartTask03 function Enable*/
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
		uCAN_MSG tempRxMessage;
		uint8_t byte0;


  /* Infinite loop */
  for(;;)
  {
//	 if(xSemaphoreTake(ISRSemaHandle,osWaitForever)){
		if(CANSPI_Receive(&tempRxMessage)) {
  			 switch (tempRxMessage.frame.id) {
  			  case 0x200:
// 				byte0 = (tempRxMessage.frame.data0);
//				ENABLE_select(byte0);
  			  case 0x050:
  				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
  				byte0 = (tempRxMessage.frame.data0);
  				avPaaKontroll(byte0);
  			  default:
  				break;
//  			}
//
//			enableControll  = (tempRxMessage.frame.data0);
//			ENABLE_select(enableControll);
//			vTaskDelay(10);
//		   if(tempRxMessage.frame.id&0x050) {
//				enableControll  = (tempRxMessage.frame.data0);
//				ENABLE_select(enableControll);
//				vTaskDelay(10);
		   }


	 }
  } // for loop
  /* USER CODE END StartTask03 */
}


/* StartTask04 function */
void StartTask04(void const * argument)
{

 for(;;)
 {

 } // for loop end
//    /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Application */

/* StartTask05 function ADC */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */

	uCAN_MSG tempADCtxmessage;
  	uCAN_MSG tempADCtxmessage2;

  	uint8_t i = 0;
  	uint16_t adcarray[16];
 	MX_ADC1_Init();


  /* Infinite loop */

  for(;;)
  {
	  	  	tempADCtxmessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	  		tempADCtxmessage.frame.id = 0x051;
	  		tempADCtxmessage.frame.dlc = 8;

	  		tempADCtxmessage2.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	  		tempADCtxmessage2.frame.id = 0x052;
	  		tempADCtxmessage2.frame.dlc = 8;

	  		MUX_select(i);
	  		vTaskDelay(10);

	  		HAL_ADC_Start(&hadc1);
	  		HAL_ADC_PollForConversion(&hadc1, 100);
	  		adcarray[i] = HAL_ADC_GetValue(&hadc1);
	  		HAL_ADC_Stop(&hadc1);

	  		vTaskDelay(10);
	  		i++;
	  		if(i>=16){


	  			tempADCtxmessage.frame.data0 = adcarray[0]; // batteri
	  			tempADCtxmessage.frame.data1 = adcarray[1]; // kjøremotor1
	  			tempADCtxmessage.frame.data2 = adcarray[2]; // kjøremotor2
	  			tempADCtxmessage.frame.data3 = adcarray[3]; // kjøremotor3
	  			tempADCtxmessage.frame.data4 = adcarray[4]; // kjøremotor4
	  			tempADCtxmessage.frame.data5 = adcarray[5]; // kjøremotor5
	  			tempADCtxmessage.frame.data6 = adcarray[6]; // kjøremotor6
	  			tempADCtxmessage.frame.data7 = adcarray[7]; // rotasjonsmotor6

	  			CANSPI_Transmit(&tempADCtxmessage);
	  			tempADCtxmessage2.frame.data0 = adcarray[8]; // kameramast
	  			tempADCtxmessage2.frame.data1 = adcarray[9]; // arm/gravefunksjon48
	  			tempADCtxmessage2.frame.data2 = adcarray[10]; // arm/gravefunksjon12
	  			tempADCtxmessage2.frame.data3 = adcarray[11]; // temp1
	  			tempADCtxmessage2.frame.data4 = adcarray[12]; // temp2
	  			tempADCtxmessage2.frame.data5 = adcarray[13]; // drill
	  			tempADCtxmessage2.frame.data6 = adcarray[14]; // xx
	  			tempADCtxmessage2.frame.data7 = adcarray[15]; // xx

	  			CANSPI_Transmit(&tempADCtxmessage2);
	  			i=0;
	  			vTaskDelay(100);
	  		}


  }
  /* USER CODE END StartTask02 */
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
