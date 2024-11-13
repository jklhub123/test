/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
extern ctrl_channel ctrl_group[24];

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Send(uint16_t id,uint8_t *buf);
void OutPut(uint8_t airway1, uint16_t output1, uint8_t airway2,uint16_t output2,uint8_t cmd);
//void SendAngle1_CAN(uint16_t angle1, uint16_t angle2,uint16_t angle3, uint16_t angle4);
//void SendAngle2_CAN(uint16_t angle1, uint16_t angle2,uint16_t angle3, uint16_t angle4);

void CAN_Send(uint16_t id, uint8_t *buf);    // 数据发鿁函敿
void SendAngle_CAN(int16_t angle1, int16_t angle2, int16_t angle3, int16_t angle4, int16_t angle5, int i);
void SendShouldAngle(uint16_t id, int16_t S_angle, uint16_t speedLimit);
void MotorOpen(uint8_t id);
void MotorClose(uint8_t id);
void MotorSetZero(uint8_t id);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

