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
#include "spi.h"
#include "ADS1263.h"

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t RxData[8];
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
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

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Filter_Init(void)
{
	CAN_FilterTypeDef canFilterConfig;
	canFilterConfig.FilterActivation = ENABLE;				 // 打开过滤器
	canFilterConfig.FilterBank = 0;							 // 过滤器0 这里可设0-13
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;		 // 采用掩码模式
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;	 // 采用32位掩码模式
	canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 采用FIFO0

	canFilterConfig.FilterIdHigh = 0x0200;	   // 设置过滤器ID高16位
	canFilterConfig.FilterIdLow = 0x0000;	   // 设置过滤器ID低16位
	canFilterConfig.FilterMaskIdHigh = 0x0200; // 设置过滤器掩码高16位
	canFilterConfig.FilterMaskIdLow = 0x0000;  // 设置过滤器掩码低16位

	if (HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK) // 初始化过滤器
	{
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan) != HAL_OK) // 启动can
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // 开启接受邮箱0挂起中断
	{
		Error_Handler();
	}
}
// // 数采板的发送
// void CAN_Send(uint8_t id, uint8_t *buf)
// {
// 	CAN_TxHeaderTypeDef TxHeader;
// 	uint8_t motorId = id;
// 	TxHeader.StdId = 0x110 + motorId;
// 	TxHeader.RTR = CAN_RTR_DATA;
// 	TxHeader.IDE = CAN_ID_STD;
// 	TxHeader.DLC = 8;
// 	static uint32_t txMailBox;
// 	HAL_CAN_AddTxMessage(&hcan, &TxHeader, buf, &txMailBox);
// }
void CAN_Send(uint16_t id, uint8_t *buf)
{
	// 点亮PB0的LED
	HAL_Delay(100);
	// HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	CAN_TxHeaderTypeDef TxHeader;
	// uint16_t motorId = id;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	static uint32_t txMailBox;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, buf, &txMailBox);

	//	 while(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, buf, &txMailBox) != HAL_OK)
	//		{
	//			printf("TxMsg Failed!!");
	//      HAL_Delay(100);
	//
	//		}
	//		printf("\nSend Tx Message Success!!Tx_Mail:%d", txMailBox);
}

void SendAngle_CAN(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4, int i)
{
	static uint8_t cmdtemp[8] = {0};
	uint16_t id = 0x102 + i;
	// printf("read id is:%d", id);

	cmdtemp[0] = (angle1 >> 8) & 0x00FF; // 输出角度1
	cmdtemp[1] = angle1 & 0x00FF;
	cmdtemp[2] = (angle2 >> 8) & 0x00FF; // 输出角度2
	cmdtemp[3] = angle2 & 0x00FF;
	cmdtemp[4] = (angle3 >> 8) & 0x00FF; // 输出角度3
	cmdtemp[5] = angle3 & 0x00FF;
	cmdtemp[6] = (angle4 >> 8) & 0x00FF; // 输出角度4
	cmdtemp[7] = angle4 & 0x00FF;

	// HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_TogglePin(GPIOA, CTX_LED_Pin);
//	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOC, CRX_LED_Pin);
	HAL_Delay(500);
	// HAL_UART_Transmit(&huart1,(uint8_t *)cmdtemp,4,1000);
	CAN_Send(id, cmdtemp);
}

// //给气阀板的发送  uint8_t buf[0]=0x00ch buf[2]=气阀值
// void CAN_SendToFa(uint8_t id, uint8_t *buf)
// {
// 	CAN_TxHeaderTypeDef TxHeader;
// 	uint8_t motorId = id;
// 	TxHeader.StdId = 0x120 + motorId;
// 	TxHeader.RTR = CAN_RTR_DATA;
// 	TxHeader.IDE = CAN_ID_STD;
// 	TxHeader.DLC = 4;
// 	static uint32_t txMailBox;
// 	HAL_CAN_AddTxMessage(&hcan, &TxHeader, buf, &txMailBox);
// }

// //回调函数对于角度数据的筛选
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
// 	uint8_t aRxData[8] = {0};
// 	uint8_t stemp[8];
// 	if (hcan->Instance == CAN1)
// 	{
// 		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, aRxData) == HAL_OK)
// 		{
// 			switch (aRxData[0])
// 			{
// 			case 0x10: // read all slaver information(angles and pressures)
// 				for (int i = 0; i < 12; i++)
// 				{
// 					if (i <= 4)
// 					{
// 						stemp[0] = (Hand_Angel[4 * i] >> 8) & 0x00FF;
// 						stemp[1] = Hand_Angel[4 * i] & 0x00FF;
// 						stemp[2] = (Hand_Angel[4 * i + 1] >> 8) & 0x00FF;
// 						stemp[3] = Hand_Angel[4 * i + 1] & 0x00FF;
// 						stemp[4] = (Hand_Angel[4 * i + 2] >> 8) & 0x00FF;
// 						stemp[5] = Hand_Angel[4 * i + 2] & 0x00FF;
// 						stemp[6] = (Hand_Angel[4 * i + 3] >> 8) & 0x00FF;
// 						stemp[7] = Hand_Angel[4 * i + 3] & 0x00FF;
// 						CAN_Send(i, stemp);
// 					}
// 					else if (i > 4 && i < 7)
// 					{
// 						stemp[0] = (Arm_Angel[4 * i - 20] >> 8) & 0x00FF;
// 						stemp[1] = Arm_Angel[4 * i - 20] & 0x00FF;
// 						stemp[2] = (Arm_Angel[4 * i - 20 + 1] >> 8) & 0x00FF;
// 						stemp[3] = Arm_Angel[4 * i - 20 + 1] & 0x00FF;
// 						stemp[4] = (Arm_Angel[4 * i - 20 + 2] >> 8) & 0x00FF;
// 						stemp[5] = Arm_Angel[4 * i - 20 + 2] & 0x00FF;
// 						stemp[6] = (Arm_Angel[4 * i - 20 + 3] >> 8) & 0x00FF;
// 						stemp[7] = Arm_Angel[4 * i - 20 + 3] & 0x00FF;
// 						CAN_Send(i, stemp);
// 					}
// 					else
// 					{
// 						stemp[0] = *(uint8_t *)(&vol_print[i - 7]);
// 						stemp[1] = *((uint8_t *)(&vol_print[i - 7]) + 1);
// 						stemp[2] = *((uint8_t *)(&vol_print[i - 7]) + 2);
// 						stemp[3] = *((uint8_t *)(&vol_print[i - 7]) + 3);
// 						stemp[4] = *((uint8_t *)(&vol_print[i - 7]) + 4);
// 						stemp[5] = *((uint8_t *)(&vol_print[i - 7]) + 5);
// 						stemp[6] = *((uint8_t *)(&vol_print[i - 7]) + 6);
// 						stemp[7] = *((uint8_t *)(&vol_print[i - 7]) + 7);
// 						CAN_Send(i, stemp);
// 					}
// 				}
// 				break;

// 			case 0x11: // read slaver information from HAND
// 				for (int i = 0; i < 5; i++)
// 				{
// 					stemp[0] = (Hand_Angel[4 * i] >> 8) & 0x00FF;
// 					stemp[1] = Hand_Angel[4 * i] & 0x00FF;
// 					stemp[2] = (Hand_Angel[4 * i + 1] >> 8) & 0x00FF;
// 					stemp[3] = Hand_Angel[4 * i + 1] & 0x00FF;
// 					stemp[4] = (Hand_Angel[4 * i + 2] >> 8) & 0x00FF;
// 					stemp[5] = Hand_Angel[4 * i + 2] & 0x00FF;
// 					stemp[6] = (Hand_Angel[4 * i + 3] >> 8) & 0x00FF;
// 					stemp[7] = Hand_Angel[4 * i + 3] & 0x00FF;
// 					CAN_Send(i, stemp);
// 				}
// 				break;

// 			case 0x12: // read slaver information from ARM
// 				for (int i = 5; i < 7; i++)
// 				{
// 					stemp[0] = (Arm_Angel[4 * i - 20] >> 8) & 0x00FF;
// 					stemp[1] = Arm_Angel[4 * i - 20] & 0x00FF;
// 					stemp[2] = (Arm_Angel[4 * i - 20 + 1] >> 8) & 0x00FF;
// 					stemp[3] = Arm_Angel[4 * i - 20 + 1] & 0x00FF;
// 					stemp[4] = (Arm_Angel[4 * i - 20 + 2] >> 8) & 0x00FF;
// 					stemp[5] = Arm_Angel[4 * i - 20 + 2] & 0x00FF;
// 					stemp[6] = (Arm_Angel[4 * i - 20 + 3] >> 8) & 0x00FF;
// 					stemp[7] = Arm_Angel[4 * i - 20 + 3] & 0x00FF;
// 					CAN_Send(i, stemp);
// 				}
// 				break;

// 			default:
// 				break;
// 			}
// 		}
// 	}
// }

/* USER CODE END 1 */
