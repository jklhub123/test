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
#include <stdlib.h> 
float last_angle[4] = {0};
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
	//HAL_Delay(100);
	//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
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
void CAN_Send_DLC_ZERO(uint16_t id, uint8_t *buf)
{
	// 点亮PB0的LED
	//HAL_Delay(100);
	//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	CAN_TxHeaderTypeDef TxHeader;
	// uint16_t motorId = id;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 0;
	static uint32_t txMailBox;
	buf = NULL;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, buf, &txMailBox);
}

//void SendAngle1_CAN(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4)
//{
//	static uint8_t cmdtemp[8] = {0};

//	cmdtemp[0] = (angle1 >> 8) & 0x00FF; // 输出角度1
//	cmdtemp[1] = angle1 & 0x00FF;
//	cmdtemp[2] = (angle2 >> 8) & 0x00FF; // 输出角度2
//	cmdtemp[3] = angle2 & 0x00FF;
//	cmdtemp[4] = (angle3 >> 8) & 0x00FF; // 输出角度3
//	cmdtemp[5] = angle3 & 0x00FF;
//	cmdtemp[6] = (angle4 >> 8) & 0x00FF; // 输出角度4
//	cmdtemp[7] = angle4 & 0x00FF;

//	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

//	// HAL_UART_Transmit(&huart1,(uint8_t *)cmdtemp,4,1000);
//	CAN_Send(0x102, cmdtemp);
//}

//void SendAngle2_CAN(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4)
//{
//	static uint8_t cmdtemp[8] = {0};

//	cmdtemp[0] = (angle1 >> 8) & 0x00FF; // 输出角度1
//	cmdtemp[1] = angle1 & 0x00FF;
//	cmdtemp[2] = (angle2 >> 8) & 0x00FF; // 输出角度2
//	cmdtemp[3] = angle2 & 0x00FF;
//	cmdtemp[4] = (angle3 >> 8) & 0x00FF; // 输出角度3
//	cmdtemp[5] = angle3 & 0x00FF;
//	cmdtemp[6] = (angle4 >> 8) & 0x00FF; // 输出角度4
//	cmdtemp[7] = angle4 & 0x00FF;

//	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

//	// HAL_UART_Transmit(&huart1,(uint8_t *)cmdtemp,4,1000);
//	CAN_Send(0x103, cmdtemp);
//}

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


void SendAngle_CAN(int16_t angle1, int16_t angle2, int16_t angle3, int16_t angle4, int16_t angle5, short i)
{
  static uint8_t cmdtemp[8] = {0};
  uint16_t id = 0x102 + i;
  // printf("read id is:%d", id);

  cmdtemp[0] = (angle2 >> 8) & 0x00FF; // 输出角度1
  cmdtemp[1] = angle2 & 0x00FF;
  cmdtemp[2] = (angle3 >> 8) & 0x00FF; // 输出角度2
  cmdtemp[3] = angle3 & 0x00FF;
  cmdtemp[4] = (angle4 >> 8) & 0x00FF; // 输出角度3
  cmdtemp[5] = angle4 & 0x00FF;
  cmdtemp[6] = (angle5 >> 8) & 0x00FF; // 输出角度4
  cmdtemp[7] = angle5 & 0x00FF;

  // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  //	HAL_GPIO_TogglePin(GPIOA, CTX_LED_Pin);
  //	HAL_Delay(500);
  // HAL_GPIO_TogglePin(GPIOA, CTX_LED_Pin);
  // HAL_Delay(500);
  // HAL_UART_Transmit(&huart1,(uint8_t *)cmdtemp,4,1000);
  CAN_Send(id, cmdtemp);
  //SendShouldAngle(3-i , angle1 , 300);
}
void SendShouldAngle(uint16_t id, int16_t S_angle, uint16_t speedLimit)
{
	//-------------------------------------------------------------------------------------------------
	//|  单圈位置闭环控制命令 2�?1 帧）0xA6                                                           |
	//|  主机发�?�该命令以控制电机的位置（单圈角度）�?                                                 |
	//|   1. 控制�? spinDirection 设置电机转动的方向，�? uint8_t 类型�?                               |
	//|      0x00 代表顺时针，0x01 代表逆时�?                                                         |
	//|   2. angleControl �? uint32_t 类型，对应实际位置为 0.01degree/LSB，即 36000 代表 360°�?       |
	//|   3. 速度控制�? maxSpeed 限制了电机转动的�?大�?�度，为 uint16_t 类型，对应实际转�?             |
	//|      1dps/LSB，即 360 代表 360dps�?                                                           |
    //|   DATA[0] 命令字节 0xA6                                                                       |
    //|   DATA[1] 转动方向字节   DATA[1] = spinDirection                                              |
    //|   DATA[2] 速度限制低字�? DATA[2] = *( uint8_t *)(&maxSpeed)                                   |
    //|   DATA[3] 速度限制高字�? DATA[3] = *((uint8_t *)(&maxSpeed)+1)                                |
    //|   DATA[4] 位置控制低字�? DATA[4] = *( uint8_t *)(&angleControl)                               |
    //|   DATA[5] 位置控制       DATA[5] = *((uint8_t *)(&angleControl)+1)                            |
    //|   DATA[6] 位置控制       DATA[6] = *((uint8_t *)(&angleControl)+2)                            |
    //|   DATA[7] 位置控制高字�? DATA[7] = *((uint8_t *)(&angleControl)+3)                            |
    //|   备注�?                                                                                      |
    //|   1. 该控制模式下，电机的�?大加速度由上位机中的 Max Acceleration 值限制�??                     |
    //|   2. 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中�? Max Torque Current               |
	//|      值限制；MS 电机的最大功率由上位机中�? Max Power 值限制�??                                 |
	//-------------------------------------------------------------------------------------------------

	if(id == 0){id = 0x142;}
	if(id == 1){id = 0x141;}
	if(id == 2){id = 0x144;}
	if(id == 3){id = 0x143;}
	uint8_t cmdtemp[8] = {0};
	int16_t position_add;
	float position_temp = ((float)S_angle) / 8192 * 180 * 100 * 6;
	position_add = (int)(position_temp - last_angle[id]);
	if(id == 0x142){position_add = -position_add;}//check motor 1 's rotate direction
	if(id == 0x144){position_add = -position_add;}//check motor 1 's rotate direction
	cmdtemp[0] = 0xA8;
	cmdtemp[1] = 0x00;
	cmdtemp[2] =  speedLimit & 0xFF;
	cmdtemp[3] = (speedLimit >> 8) & 0xFF;
	cmdtemp[4] =  position_add & 0xFF;
	cmdtemp[5] = (position_add >> 8) & 0xFF;
	cmdtemp[6] = (position_add >> 16) & 0xFF;
	cmdtemp[7] = (position_add >> 24) & 0xFF;
	CAN_Send(id, cmdtemp);
//	if (id == 1)
//		{
//			printf("+++CAN SENDDING \r\n");
//		}
	last_angle[id] = position_temp;//赋角度�?�给下一次比�?
}

void MotorOpen(uint8_t id)
{
	uint8_t cmdtemp[8] = {0};
	cmdtemp[0] = 0x80;
	CAN_Send(id + 0x140, cmdtemp);
}

void MotorClose(uint8_t id)
{
	uint8_t cmdtemp[8] = {0};
	cmdtemp[0] = 0x81;
	CAN_Send(id + 0x140, cmdtemp);
}

void MotorSetZero(uint8_t id)
{
	uint8_t cmdtemp[8] = {0};
	cmdtemp[0] = 0x19;
	CAN_Send(id + 0x140, cmdtemp);
}
void left_arm_button_1_press()
{
	uint16_t id = 0x106;
	uint8_t cmdtemp[8] = {0};
	CAN_Send(id, cmdtemp);
}

void left_arm_button_2_press()
{
	uint16_t id = 0x107;
	uint8_t cmdtemp[8] = {0};
	CAN_Send(id, cmdtemp);
}

void right_arm_button_1_press()
{
	uint16_t id = 0x108;
	uint8_t cmdtemp[8] = {0};
	CAN_Send(id, cmdtemp);
}

void right_arm_button_2_press()
{
	uint16_t id = 0x109;
	uint8_t cmdtemp[8] = {0};
	CAN_Send(id, cmdtemp);
}

void Send_MOVP(int8_t joint_id , int16_t angle[4])
{
	//开环控制 气压指令
	//左手肩部joint_id=0,根部关节joint_id=3
	//            can id = 110~113
	//右手肩部joint_id=4,根部关节joint_id=7
	//            can id = 114~117
  int16_t id = 0x110 + joint_id;
  static uint8_t cmdtemp[8] = {0};
  cmdtemp[0] = (angle[0] >> 8) & 0x00FF; // 输出气压1
  cmdtemp[1] = angle[0] & 0x00FF;
  cmdtemp[2] = (angle[1] >> 8) & 0x00FF; // 输出气压2
  cmdtemp[3] = angle[1] & 0x00FF;
  cmdtemp[4] = (angle[2] >> 8) & 0x00FF; // 输出气压3
  cmdtemp[5] = angle[2] & 0x00FF;
  cmdtemp[6] = (angle[3] >> 8) & 0x00FF; // 输出气压4
  cmdtemp[7] = angle[3] & 0x00FF;
  CAN_Send(id, cmdtemp);
}

void Send_DEEP_Motor(int8_t MOTOR_id , uint16_t MOTOR_MAT[5])
{
	/*
	--------云深处电机指令--------
	CAN ID   BIT 0~4     目标关节ID
			 BIT 5~6       4
	CAN DLC  BIT 0~3       8
	CAN DATA BIT0~15     目标角度
	         BIT16~29    目标角速度
	         BIT30~39    KP刚度
	         BIT40~47    KD阻尼
	         BIT48~63    目标扭矩
	------1.位置控制模式-------	
	进行位置控制时，速度阻尼kd取值建议在1.0-15.0之间，否则容易造成
	电机在目标位置附近不停振荡，甚至失控。
	p=3.14
	v=0
	t=0
	kp=50
	kd=5
	------2.速度控制模式-------	
	进行速度控制时，速度阻尼kd取值建议在1.0-15.0之间，否则容易造成
	电机在目标位置附近不停振荡，甚至失控。
	p=0
	v=5
	t=0
	kp=0
	kd=5
	------3.扭矩控制模式-------	
	(空载下即使给定很小扭矩，电机也会加速到最大速度旋转)
	p=0
	v=0
	t=0.5
	kp=0
	kd=0
	------4.阻尼模式-------	
	阻尼模式通常用于控制电机的减速、制动或阻尼操作。在阻尼模式下，电机的目标是通过施
	加适当的电机输出，以产生阻力、制动或减速效果。通过控制电机输出，可以调整阻尼力或
	制动力的大小，从而实现不同程度的运动减速或制动效果。p例如，给定kd不为 0，其余运
	动参数为0：
	p=0
	v=0
	t=0
	kp=0
	kd=5
	------5.零扭矩模式-------	
	零扭矩模式下，电机的目标是将输出扭矩维持在零或接近零的水平，但电机并不是停止运转，
	而是主动产生扭矩来抵抗自身的摩擦扭矩。给定所有运动参数都为0：
	p=0
	v=0
	t=0
	kp=0
	kd=0
	------6.力位混合控制模式-------
	p=3.14
	v=0
	t=1
	kp=30
	kd=5
	------------------------------
	*/
	
	// CAN ID   BIT 0~4     目标关节ID
	// 		 BIT 5~6       4
	//int16_t id = 1;
	static uint8_t cmdtemp[8] = {0};
	//id = (((MOTOR_id & 0b00001111) << 2) | 0b00000011);//这一句使用二进制与操作和合并，编译器会报错括号分号问题
  	int8_t cmd = CONTROL_MOTOR;
	int16_t id = (cmd << CAN_ID_SHIFT_BITS) | MOTOR_id;
  	/*
	           位数        名称               取值范围
  	CAN DATA BIT0~15     目标角度     [-40rad,40rad]     -> [0,65535]     FFFF
	         BIT16~29    目标角速度   [-40rad/s,40rad/s] -> [0,16383]     3FFF
	         BIT30~39    KP刚度      [0,1023]           -> [0,1023]      3FF
	         BIT40~47    KD阻尼      [0.0,51.0]         -> [0,255]       FF
	         BIT48~63    目标扭矩    [-40Nm,40Nm]       -> [0,65535]     FFFF
	*/
	//目标角度 0000 ~ FFFF 
	cmdtemp[0] = MOTOR_MAT[0] & 0x00FF;          //储存低8位
	cmdtemp[1] = (MOTOR_MAT[0] >> 8) & 0x00FF;   //储存高8位
	// 角速度 0~3FFF
	cmdtemp[2] = MOTOR_MAT[1] & 0x00FF;          //储存低8位
	cmdtemp[3] = ((MOTOR_MAT[1] >> 8) & 0x003F) |//储存高8位
				 ((MOTOR_MAT[2] & 0x03) << 6);   //储存KP低6位
	//KP刚度,只并0X3FF,最大值只有1023
	cmdtemp[4] = (MOTOR_MAT[2] >> 2) & 0x00FF;   //储存KP高2位
	//KD阻尼,只取后2位,最大值只有255
	cmdtemp[5] = MOTOR_MAT[3] & 0x00FF;
	// 扭矩
	cmdtemp[6] = MOTOR_MAT[4] & 0x00FF; 
	cmdtemp[7] = (MOTOR_MAT[4] >> 8) & 0x00FF;
	CAN_Send(id, cmdtemp);
}
void turn_on_DEEP_Motor(int8_t MOTOR_id)
{
	int8_t cmd = ENABLE_MOTOR;
	int16_t id;
	id = (cmd << CAN_ID_SHIFT_BITS) | MOTOR_id;
	static uint8_t cmdtemp[8] = {0};
	CAN_Send_DLC_ZERO(id, cmdtemp);
}
void turn_off_DEEP_Motor(int8_t MOTOR_id)
{
	int8_t cmd = DISABLE_MOTOR;
	int16_t id;
	id = (cmd << CAN_ID_SHIFT_BITS) | MOTOR_id;
	static uint8_t cmdtemp[8] = {0};
	CAN_Send_DLC_ZERO(id, cmdtemp);
}

/* USER CODE END 1 */
