///**
//  ******************************************************************************
//  * File Name          : TIM.c
//  * Description        : This file provides code for the configuration
//  *                      of the TIM instances.
//  ******************************************************************************
//  * This notice applies to any and all portions of this file
//  * that are not between comment pairs USER CODE BEGIN and
//  * USER CODE END. Other portions of this file, whether
//  * inserted by the user or by software development tools
//  * are owned by their respective copyright owners.
//  *
//  * Copyright (c) 2018 STMicroelectronics International N.V.
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted, provided that the following conditions are met:
//  *
//  * 1. Redistribution of source code must retain the above copyright notice,
//  *    this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  *    this list of conditions and the following disclaimer in the documentation
//  *    and/or other materials provided with the distribution.
//  * 3. Neither the name of STMicroelectronics nor the names of other
//  *    contributors to this software may be used to endorse or promote products
//  *    derived from this software without specific written permission.
//  * 4. This software, including modifications and/or derivative works of this
//  *    software, must execute solely and exclusively on microcontroller or
//  *    microprocessor devices manufactured by or for STMicroelectronics.
//  * 5. Redistribution and use of this software other than as permitted under
//  *    this license is void and will automatically terminate your rights under
//  *    this license.
//  *
//  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
//  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
//  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
//  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  ******************************************************************************
//  */
//
///* Includes ------------------------------------------------------------------*/
//#include "tim.h"
//
///* USER CODE BEGIN 0 */
////
///* USER CODE END 0 */
//
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim4;
//
///* TIM1 init function */
//void MX_TIM1_Init(void)
//{
//  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_OC_InitTypeDef sConfigOC;
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
//
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 7;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 0;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//  HAL_TIM_MspPostInit(&htim1);
//
//}
///* TIM4 init function */
//void MX_TIM4_Init(void)
//{
//  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_OC_InitTypeDef sConfigOC;
//
//  htim4.Instance = TIM4;
//  htim4.Init.Prescaler = 7;
//  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim4.Init.Period = 0;
//  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//
//  HAL_TIM_MspPostInit(&htim4);
//
//}
//
//void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
//{
//
//  if(tim_pwmHandle->Instance==TIM1)
//  {
//  /* USER CODE BEGIN TIM1_MspInit 0 */
//////
//  /* USER CODE END TIM1_MspInit 0 */
//    /* TIM1 clock enable */
//    __HAL_RCC_TIM1_CLK_ENABLE();
//  /* USER CODE BEGIN TIM1_MspInit 1 */
//////
//  /* USER CODE END TIM1_MspInit 1 */
//  }
//  else if(tim_pwmHandle->Instance==TIM4)
//  {
//  /* USER CODE BEGIN TIM4_MspInit 0 */
//////
//  /* USER CODE END TIM4_MspInit 0 */
//    /* TIM4 clock enable */
//    __HAL_RCC_TIM4_CLK_ENABLE();
//  /* USER CODE BEGIN TIM4_MspInit 1 */
//////
//  /* USER CODE END TIM4_MspInit 1 */
//  }
//}
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
//{
//
//  GPIO_InitTypeDef GPIO_InitStruct;
//  if(timHandle->Instance==TIM1)
//  {
//  /* USER CODE BEGIN TIM1_MspPostInit 0 */
//////
//  /* USER CODE END TIM1_MspPostInit 0 */
//    /**TIM1 GPIO Configuration
//    PE9     ------> TIM1_CH1
//    */
//    GPIO_InitStruct.Pin = DRIVE_PWM2_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
//    HAL_GPIO_Init(DRIVE_PWM2_GPIO_Port, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN TIM1_MspPostInit 1 */
//////
//  /* USER CODE END TIM1_MspPostInit 1 */
//  }
//  else if(timHandle->Instance==TIM4)
//  {
//  /* USER CODE BEGIN TIM4_MspPostInit 0 */
//////
//  /* USER CODE END TIM4_MspPostInit 0 */
//
//    /**TIM4 GPIO Configuration
//    PD12     ------> TIM4_CH1
//    */
//    GPIO_InitStruct.Pin = DRIVE_PWM_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//    HAL_GPIO_Init(DRIVE_PWM_GPIO_Port, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN TIM4_MspPostInit 1 */
//////
//  /* USER CODE END TIM4_MspPostInit 1 */
//  }
//
//}
//
//void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
//{
//
//  if(tim_pwmHandle->Instance==TIM1)
//  {
//  /* USER CODE BEGIN TIM1_MspDeInit 0 */
//////
//  /* USER CODE END TIM1_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM1_CLK_DISABLE();
//  /* USER CODE BEGIN TIM1_MspDeInit 1 */
//////
//  /* USER CODE END TIM1_MspDeInit 1 */
//  }
//  else if(tim_pwmHandle->Instance==TIM4)
//  {
//  /* USER CODE BEGIN TIM4_MspDeInit 0 */
//////
//  /* USER CODE END TIM4_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM4_CLK_DISABLE();
//  /* USER CODE BEGIN TIM4_MspDeInit 1 */
//////
//  /* USER CODE END TIM4_MspDeInit 1 */
//  }
//}
//
///* USER CODE BEGIN 1 */
////
///* USER CODE END 1 */
//
///**
//  * @}
//  */
//
///**
//  * @}
//  */
//
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
