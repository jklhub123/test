/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "spi.h"
ctrl_channel ctrl_group[24] = {
	{
		// CN1
		// ADC3	PF6
		// PWM3	PB8
		.htim = &htim10,
		.tim_channel = TIM_CHANNEL_1,
		.in_value = 0,
		//.hadc = &hadc3,
		//.adc_channel = ADC_CHANNEL_4,
		// .adc_value = 0,
	},
	{
		// CN2
		// ADC2	PF5
		// PWM2	PB9
		.htim = &htim11,
		.tim_channel = TIM_CHANNEL_1,
		.in_value = 0,
		//	.hadc = &hadc3,
		//	.adc_channel = ADC_CHANNEL_15,
		// .adc_value = 0,
	},
	{
		// CN3
		// ADC1	PF4
		// PWM1	PE5
		.htim = &htim9,
		.tim_channel = TIM_CHANNEL_1,
		.in_value = 0,
		//	.hadc = &hadc3,
		//	.adc_channel = ADC_CHANNEL_14,
		// .adc_value = 0,
	},
	{
		// CN4
		// ADC0	PF3
		// PWM0	PE6
		.htim = &htim9,
		.tim_channel = TIM_CHANNEL_2,
		.in_value = 0,
		//	.hadc = &hadc3,
		//	.adc_channel = ADC_CHANNEL_9,
		// .adc_value = 0,
	},
	{
		// CN5
		// ADC7	PF7
		// PWM7	PE14
		.htim = &htim1,
		.tim_channel = TIM_CHANNEL_4,
		.in_value = 0,
		//	.hadc = &hadc3,
		//.adc_channel = ADC_CHANNEL_5,
		// .adc_value = 0,
	},
	{
		// CN6
		// ADC6	PF8
		// PWM6	PE13
		.htim = &htim1,
		.tim_channel = TIM_CHANNEL_3,
		.in_value = 0,
		//	.hadc = &hadc3,
		//	.adc_channel = ADC_CHANNEL_6,
		// .adc_value = 0,
	},
	{
		// CN7
		// ADC5	PF9
		// PWM5	PE9
		.htim = &htim1,
		.tim_channel = TIM_CHANNEL_1,
		.in_value = 0,
		//	.hadc = &hadc3,
		//	.adc_channel = ADC_CHANNEL_7,
		// .adc_value = 0,
	},
	{
		// CN8
		// ADC4	PF10
		// PWM4	PE11
		.htim = &htim1,
		.tim_channel = TIM_CHANNEL_2,
		.in_value = 0,
		//	.hadc = &hadc3,
		//	.adc_channel = ADC_CHANNEL_8,
		// .adc_value = 0,
	},
	{
		// CN9
		// ADC11	PC0
		// PWM11	PB10
		.htim = &htim2,
		.tim_channel = TIM_CHANNEL_3,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_10,
		// .adc_value = 0,
	},
	{
		// CN10
		// ADC10	PC1
		// PWM10	PB11
		.htim = &htim2,
		.tim_channel = TIM_CHANNEL_4,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_11,
		// .adc_value = 0,
	},
	{
		// CN11
		// ADC9	PC2
		// PWM9	PB14
		.htim = &htim12,
		.tim_channel = TIM_CHANNEL_1,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_12,
		// .adc_value = 0,
	},
	{
		// CN12
		// ADC8	PC3
		// PWM8	PB15
		.htim = &htim12,
		.tim_channel = TIM_CHANNEL_2,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_13,
		// .adc_value = 0,
	},
	{
		// CN13
		// ADC20	PA4
		// PWM20	PB5
		.htim = &htim3,
		.tim_channel = TIM_CHANNEL_2,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_4,
		// .adc_value = 0,
	},
	{
		// CN14
		// ADC21	PA5
		// PWM21	PB4
		.htim = &htim3,
		.tim_channel = TIM_CHANNEL_1,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_5,
		// .adc_value = 0,
	},
	{
		// CN15
		// ADC22	PA6
		// PWM22	PB3
		.htim = &htim2,
		.tim_channel = TIM_CHANNEL_2,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_6,
		// .adc_value = 0,
	},
	{
		// CN16
		// ADC23	PA7
		// PWM23	PA15
		.htim = &htim2,
		.tim_channel = TIM_CHANNEL_1,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_7,
		// .adc_value = 0,
	},
	{
		// CN17
		// ADC16	PC4
		// PWM16	PC9
		.htim = &htim8,
		.tim_channel = TIM_CHANNEL_4,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_14,
		// .adc_value = 0,
	},
	{
		// CN18
		// ADC17	PC5
		// PWM17	PC8
		.htim = &htim8,
		.tim_channel = TIM_CHANNEL_3,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_15,
		// .adc_value = 0,
	},
	{
		// CN19
		// ADC18	PB0
		// PWM18	PC7
		.htim = &htim8,
		.tim_channel = TIM_CHANNEL_2,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_8,
		// .adc_value = 0,
	},
	{
		// CN20
		// ADC19	PB1
		// PWM19	PC6
		.htim = &htim8,
		.tim_channel = TIM_CHANNEL_1,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_9,
		// .adc_value = 0,
	},
	{
		// CN21
		// ADC12	PA0
		// PWM12	PD15
		.htim = &htim4,
		.tim_channel = TIM_CHANNEL_4,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_0,
		// .adc_value = 0,
	},
	{
		// CN22
		// ADC13	PA1
		// PWM13	PD14
		.htim = &htim4,
		.tim_channel = TIM_CHANNEL_3,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_1,
		// .adc_value = 0,
	},
	{
		// CN23
		// ADC14	PA2
		// PWM14	PD13
		.htim = &htim4,
		.tim_channel = TIM_CHANNEL_2,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_2,
		// .adc_value = 0,
	},
	{
		// CN24
		// ADC15	PA3
		// PWM15	PD12
		.htim = &htim4,
		.tim_channel = TIM_CHANNEL_1,
		.in_value = 0,
		//	.hadc = &hadc1,
		//	.adc_channel = ADC_CHANNEL_3,
		// .adc_value = 0,
	},
};
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // Identifier mask mode
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x140 << 5; // id
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000; // id mask	// 0xFC00
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterBank = 0;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Send(uint16_t id, uint8_t *buf)
{
	// 点亮PB0的LED
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	CAN_TxHeaderTypeDef TxHeader;
	// uint16_t motorId = id;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	static uint32_t txMailBox;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, buf, &txMailBox);

	//	 while(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, buf, &txMailBox) != HAL_OK)
	//		{
	//			printf("TxMsg Failed!!");
	//      HAL_Delay(100);
	//
	//		}
	//		printf("\nSend Tx Message Success!!Tx_Mail:%d", txMailBox);
}

void SendAngle1_CAN(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4)
{
	static uint8_t cmdtemp[8] = {0};

	cmdtemp[0] = (angle1 >> 8) & 0x00FF; // 输出角度1
	cmdtemp[1] = angle1 & 0x00FF;
	cmdtemp[2] = (angle2 >> 8) & 0x00FF; // 输出角度2
	cmdtemp[3] = angle2 & 0x00FF;
	cmdtemp[4] = (angle3 >> 8) & 0x00FF; // 输出角度3
	cmdtemp[5] = angle3 & 0x00FF;
	cmdtemp[6] = (angle4 >> 8) & 0x00FF; // 输出角度4
	cmdtemp[7] = angle4 & 0x00FF;

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	// HAL_UART_Transmit(&huart1,(uint8_t *)cmdtemp,4,1000);
	CAN_Send(0x102, cmdtemp);
}

void SendAngle2_CAN(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4)
{
	static uint8_t cmdtemp[8] = {0};

	cmdtemp[0] = (angle1 >> 8) & 0x00FF; // 输出角度1
	cmdtemp[1] = angle1 & 0x00FF;
	cmdtemp[2] = (angle2 >> 8) & 0x00FF; // 输出角度2
	cmdtemp[3] = angle2 & 0x00FF;
	cmdtemp[4] = (angle3 >> 8) & 0x00FF; // 输出角度3
	cmdtemp[5] = angle3 & 0x00FF;
	cmdtemp[6] = (angle4 >> 8) & 0x00FF; // 输出角度4
	cmdtemp[7] = angle4 & 0x00FF;

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	// HAL_UART_Transmit(&huart1,(uint8_t *)cmdtemp,4,1000);
	CAN_Send(0x103, cmdtemp);
}

void OutPut(uint8_t airway1, uint16_t output1, uint8_t airway2, uint16_t output2, uint8_t cmd)
{
	static uint8_t cmdtemp[8] = {0};
	//	cmdtemp[0]='A';
	//	cmdtemp[1]='0';
	//	cmdtemp[2]=(uint8_t)(sencenum<v<1)+indexx+'0';
	//	cmdtemp[3]='B';
	//	cmdtemp[4]=output/1000+'0';vvv
	//	cmdtemp[5]=(output%1000)/100+'0';v
	//	cmdtemp[6]=(output%100)/10+'0';
	//	cmdtemp[7]=output%10+'0';
	//	HAL_UART_Transmit(&huart2,(uint8_t *)cmdtemp,8,1000);

	cmdtemp[0] = airway1;				  // 气道�??1
	cmdtemp[1] = cmd;					  // 命令,
	cmdtemp[2] = (output1 >> 8) & 0x00FF; // 输出实际�??
	cmdtemp[3] = output1 & 0x00FF;

	// 由于�??份数据只占用4个字节，�??次发送两份数�??
	cmdtemp[4] = airway2;
	cmdtemp[5] = cmd;
	cmdtemp[6] = (output2 >> 8) & 0x00FF;
	cmdtemp[7] = output2 & 0x00FF;

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	// HAL_UART_Transmit(&huart1,(uint8_t *)cmdtemp,4,1000);
	CAN_Send(0x101, cmdtemp);
}
/* USER CODE END 1 */