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
  canFilterConfig.FilterActivation = ENABLE;               // 打开过滤�?
  canFilterConfig.FilterBank = 0;                          // 过滤�?0 这里可设0-13
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;      // 采用掩码模式
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;     // 采用32位掩码模�?
  canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 采用FIFO0

  canFilterConfig.FilterIdHigh = 0x0200;     // 设置过滤器ID�?16�?
  canFilterConfig.FilterIdLow = 0x0000;      // 设置过滤器ID�?16�?
  canFilterConfig.FilterMaskIdHigh = 0x0200; // 设置过滤器掩码高16�?
  canFilterConfig.FilterMaskIdLow = 0x0000;  // 设置过滤器掩码低16�?

  if (HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK) // 初始化过滤器
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan) != HAL_OK) // 启动can
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // �?启接受邮�?0挂起中断
  {
    Error_Handler();
  }
}

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

/* USER CODE END 1 */
