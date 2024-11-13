/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "protocol.h"

extern uint16_t angle[2][4];

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
  hcan1.Init.Prescaler = 15;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 0);
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
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Filter_Init(void)
{
	CAN_FilterTypeDef canFilterConfig;
	canFilterConfig.FilterActivation = ENABLE;//打开过滤器
	canFilterConfig.FilterBank = 0;//过滤器0 这里可设0-13
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//采用掩码模式
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//采用32位掩码模式
	canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;//采用FIFO0

	canFilterConfig.FilterIdHigh = 0x0200; //设置过滤器ID高16位
	canFilterConfig.FilterIdLow = 0x0000;//设置过滤器ID低16位
	canFilterConfig.FilterMaskIdHigh = 0x0200;//设置过滤器掩码高16位
	canFilterConfig.FilterMaskIdLow = 0x0000;//设置过滤器掩码低16位
	
	if(HAL_CAN_ConfigFilter(&hcan1,&canFilterConfig) != HAL_OK)//初始化过滤器
	{
		Error_Handler();
	}
	if(HAL_CAN_Start(&hcan1) != HAL_OK)//启动can
	{
		Error_Handler();
	}
	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)//开启接受邮箱0挂起中断
	{
		Error_Handler();
	}
}
//主控上用的，FIFO1特殊点，要配置FIFO1的筛选器
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	static uint8_t RxData[8];
//  CAN_RxHeaderTypeDef RxHeader;
//    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
//    { uint8_t cmd_type = (RxHeader.StdId&0x00F0)>>4;
//			uint8_t packet_ch = (RxHeader.StdId&0x000F);
//			switch(cmd_type){
//				case 0x01:
//				{HAL_UART_Transmit(&huart2, RxData, 8, HAL_MAX_DELAY);
//				//	angle[packet_ch][j]=RxData[j];
//				}
//				break;
//			}
//			
//		}   
//		
//}


//主控接收数采
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t RxData[8]={0};
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t rxData[8]={0};
    if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    { uint8_t cmd_type = (RxHeader.StdId&0x00F0)>>4;
			uint8_t packet_ch = (RxHeader.StdId&0x000F);
			
//			HAL_UART_Transmit(&huart1, &cmd_type, 1, HAL_MAX_DELAY);
//			HAL_UART_Transmit(&huart1, &packet_ch, 1, HAL_MAX_DELAY);
			switch(cmd_type){
				case 0x01:
				{
//					HAL_UART_Transmit(&huart1, RxData, 8, HAL_MAX_DELAY);
					rxData[1]=RxData[0];                 //RxData[0]为高八位，RxData[1]为低八位 离数采板近的第一个传感器
					rxData[0]=RxData[1];
					rxData[3]=RxData[2];
					rxData[2]=RxData[3];
					rxData[5]=RxData[4];
					rxData[4]=RxData[5];
					rxData[7]=RxData[6];
					rxData[6]=RxData[7];
					
					set_computer_value(SEND_FACT_CMD, packet_ch, rxData, 4);//主控实时将角度值发送给上位机
					angle[packet_ch][0]=(RxData[0]<<8)|RxData[1];            //全局变量，用于pid等控制
					angle[packet_ch][1]=(RxData[2]<<8)|RxData[3];
					angle[packet_ch][2]=(RxData[4]<<8)|RxData[5];
					angle[packet_ch][3]=(RxData[6]<<8)|RxData[7];
//					uint8_t temp=RxData[0];
//					HAL_UART_Transmit(&huart1, &temp, 1, 0xFFFFF);
				}
				break;
			}
			
		}   
	
	
}
//气阀板的接收
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	static uint8_t RxData[8]={0};
//  CAN_RxHeaderTypeDef RxHeader;

//    if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
//    { 
//			uint8_t cmd_ch = RxData[0];
//			uint8_t cmd_type = RxData[1];
//			
//			uint16_t value = (RxData[2]<<8)+RxData[3];
////		if(value==0x0304)
////			HAL_UART_Transmit(&huart1,(uint8_t *)value, 2, HAL_MAX_DELAY);
//			value/=9;
//			switch(cmd_type){
//				case 0x00: //气阀给值
//				{if(cmd_ch<=24||cmd_ch>0)
//				__HAL_TIM_SET_COMPARE(ctrl_group[cmd_ch-1].htim, ctrl_group[cmd_ch-1].tim_channel, value);
//					if(cmd_ch<=48||cmd_ch>24)
//				__HAL_TIM_SET_COMPARE(ctrl_group[cmd_ch-25].htim, ctrl_group[cmd_ch-25].tim_channel, value);
//				}
//				break;
//				case 0x01: //清零
//				{
//				}
//				break;
//			}
//			
//		}   

//}
////数采板的发送
//void CAN_Send(uint8_t id,uint8_t *buf)
//{   
//	  CAN_TxHeaderTypeDef TxHeader;
//    uint8_t motorId = id;
//    TxHeader.StdId = 0x110 + motorId;
//    TxHeader.RTR = CAN_RTR_DATA;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.DLC = 8;
//    static uint32_t txMailBox;
//	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, buf, &txMailBox);
//}
//给气阀板的发送  uint8_t buf[0]=ch buf[2]=气阀值 例CAN_Send(1,buf);
void CAN_Send(uint8_t id,uint8_t *buf)
{   
	  CAN_TxHeaderTypeDef TxHeader;
    uint8_t motorId = id;
    TxHeader.StdId = 0x120 + motorId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 4;
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

void OutPut(uint8_t airway,uint16_t output,uint8_t cmd)
{
	static uint8_t cmdtemp[4]={0};
//	cmdtemp[0]='A';
//	cmdtemp[1]='0';
//	cmdtemp[2]=(uint8_t)(sencenum<<1)+indexx+'0';
//	cmdtemp[3]='B';
//	cmdtemp[4]=output/1000+'0';
//	cmdtemp[5]=(output%1000)/100+'0';
//	cmdtemp[6]=(output%100)/10+'0';
//	cmdtemp[7]=output%10+'0';
//	HAL_UART_Transmit(&huart2,(uint8_t *)cmdtemp,8,1000);
	cmdtemp[0]=airway;
	cmdtemp[1]=cmd;
	cmdtemp[2]=(output>>8)&0x00FF;
	cmdtemp[3]=output&0x00FF;
	HAL_UART_Transmit(&huart1,(uint8_t *)cmdtemp,4,1000);
	CAN_Send(0,cmdtemp);
}
/* USER CODE END 1 */
