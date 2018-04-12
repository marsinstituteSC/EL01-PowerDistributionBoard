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
//#include "adc.h"

/* USER CODE BEGIN Includes */     
#define B(x) S_to_binary_(#x)
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId SettFartTaskHandle;
osThreadId AckermannTaskHandle;
osThreadId CANbehandlingHandle;
QueueHandle_t MeldingQueueHandle;
QueueHandle_t FartQueueHandle;
QueueHandle_t AckerQueueHandle;
QueueHandle_t EnableCTRLQueueHandle;
SemaphoreHandle_t ISRSemaHandle;

uint16_t teller2 = 0;
extern uint16_t tellertest;

uint32_t enablekontrolltest;

//osMessageQId FartQueueHandle;
//osMessageQId RadiusQueueHandle;
//osMessageQId MeldingQueueHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
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
	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	  vTaskDelete(defaultTaskHandle);

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

/* StartTask03 function */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
		uCAN_MSG tempRxMessage;
		uCAN_MSG temptxmessage;

		uint32_t enableControll;
		uint32_t enableControll2;
		uint32_t enableControll3;
		uint32_t enableControll4;
		uint32_t enableControll5;

  /* Infinite loop */
  for(;;)
  {
	 if(xSemaphoreTake(ISRSemaHandle,osWaitForever)){
		if(CANSPI_Receive(&tempRxMessage)){
		   switch (tempRxMessage.frame.id) {

			case 0x050:

				enableControll  = (tempRxMessage.frame.data0); // Fartsmotor
				enableControll2 = (tempRxMessage.frame.data1); // Rotasjonsmotor
				enableControll3 = tempRxMessage.frame.data2; // Kameramast
				enableControll4 = tempRxMessage.frame.data3; // Robotarm/Gravefunksjon
				enableControll5 = tempRxMessage.frame.data4; // Drill

				if(enableControll != 0) {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET); // Fartsmotor på
				}else if(enableControll == 0) {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET); // Fartsmotor av
				}
				if(enableControll2 != 0) {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET); // Rotasjonsmotor på
				}else if(enableControll2 == 0) {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET); // Rotasjonsmotor av
				}
				if(enableControll3 != 0) {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET); // Kameramast på
				}else if(enableControll3 == 0) {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET); // Kameramast av
				}
				if(enableControll4 != 0) {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET); // Gravefunksjon på
				}else if(enableControll4 == 0) {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET); // Gravefunksjon av
				}
				if(enableControll5 != 0) {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET); // Drill på
				}else if(enableControll5 == 0) {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET); // Drill av
				}
			default:
				break;
		   }
		}
	 }
  } // for loop
  /* USER CODE END StartTask03 */
}


  /* StartTask04 function */
void StartTask04(void const * argument)
{
  	uCAN_MSG tempRxMessage;
  	uCAN_MSG temptxmessage;
    for(;;)
    {

//    	    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); // Pinne PD2

    	    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET); // LED2

    	    	temptxmessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    	    	temptxmessage.frame.id = 0x050;
    	    	temptxmessage.frame.dlc = 8;
    	    	temptxmessage.frame.data0 = 0xFF;
    	    	temptxmessage.frame.data1 = 0xFF;

    	    	CANSPI_Transmit(&temptxmessage);
    	    	vTaskDelay(100);

    	    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
    	    	temptxmessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    	    	temptxmessage.frame.id = 0x050;
    	    	temptxmessage.frame.dlc = 8;
    	    	temptxmessage.frame.data0 = 0x0;
    	    	temptxmessage.frame.data1 = 0x0;

    	    	CANSPI_Transmit(&temptxmessage);
    	    	vTaskDelay(100);

    	////    	tempRxMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    	////    	tempRxMessage.frame.id = 0x050;
    	////    	tempRxMessage.frame.dlc = 8;
    	////    	tempRxMessage.frame.data0 = 0x10;
    	////    	tempRxMessage.frame.data1 = 0xFF;
    	//
    	//
    	//  	  if(xSemaphoreTake(ISRSemaHandle,osWaitForever)){
    	//  		  if(CANSPI_Receive(&tempRxMessage)){
    	//  			  switch (tempRxMessage.frame.id) {
    	//  				case 0x050:
    	//
    	//  					xQueueSend(EnableCTRLQueueHandle,&enableControll,0);
    	//  					//vTaskDelay(1000);
    	//  					xQueueSend(EnableCTRLQueueHandle,&enableControll2,0);
    	//  					xQueueSend(EnableCTRLQueueHandle,&enableControll3,0);
    	//  					xQueueSend(EnableCTRLQueueHandle,&enableControll4,0);
    	//  					xQueueSend(EnableCTRLQueueHandle,&enableControll5,0);
    	//  	//				fart = (tempRxMessage.frame.data0<<8)+tempRxMessage.frame.data1;
    	//  	//			  	radius = (tempRxMessage.frame.data2<<24)+(tempRxMessage.frame.data3<<16)+(tempRxMessage.frame.data4<<8)+tempRxMessage.frame.data5;
    	//  	//				xQueueSend(FartQueueHandle,&fart,0);
    	//  	//			  	xQueueSend(AckerQueueHandle,&radius,0);
    	//  				default:
    	//  					break;
    	//  			}

    	//  //				}
    	// 			}
    	// 	 }
    }
//    /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Application */

//
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
