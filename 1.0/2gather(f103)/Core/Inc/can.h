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

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
#define CAN_RxExtId 0x1800D8D0
#define CAN_TxExtId 0x1800D0D8
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
  void CAN_Filter_Init(void);                  // 过滤器配置函�?
  void CAN_Send(uint16_t id, uint8_t *buf);    // 数据发�?�函�?
  void CAN_SendToFa(uint8_t id, uint8_t *buf); // 气阀数据发�?�函�?
  void SendAngle_CAN(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4, int i);

  extern CAN_TxHeaderTypeDef TxHeader; // 发�??
  extern CAN_RxHeaderTypeDef RxHeader; // 接收
  extern uint8_t RxData[8];            // 数据接收数组
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

