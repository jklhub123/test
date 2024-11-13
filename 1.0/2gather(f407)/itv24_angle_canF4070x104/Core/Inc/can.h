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
//云深处CAN协议
  #define CAN_ID_SHIFT_BITS 5

  #define POSITION_MIN -40.0f
  #define POSITION_MAX 40.0f
  #define VELOCITY_MIN -40.0f
  #define VELOCITY_MAX 40.0f
  #define KP_MIN 0.0f
  #define KP_MAX 1023.0f
  #define KD_MIN 0.0f
  #define KD_MAX 51.0f
  #define TORQUE_MIN -40.0f
  #define TORQUE_MAX 40.0f

  #define GEAR_RATIO_MIN 0.0f
  #define GEAR_RATIO_MAX 50.0f

  #define MOTOR_TEMP_MIN -20.0f
  #define MOTOR_TEMP_MAX 200.0f
  #define DRIVER_TEMP_MIN -20.0f
  #define DRIVER_TEMP_MAX 200.0f
  #define CURRENT_MIN 0.0f
  #define CURRENT_MAX 40.0f

  #define SEND_POSITION_LENGTH 16
  #define SEND_VELOCITY_LENGTH 14
  #define SEND_KP_LENGTH 10
  #define SEND_KD_LENGTH 8
  #define SEND_TORQUE_LENGTH 16
  #define SEND_GEAR_RATIO_LENGTH 16
  #define SEND_LIMIT_CURRENT_LENGTH 16

  #define RECEIVE_POSITION_LENGTH 20
  #define RECEIVE_VELOCITY_LENGTH 20
  #define RECEIVE_TORQUE_LENGTH 16
  #define RECEIVE_TEMP_FLAG_LENGTH 1
  #define RECEIVE_TEMP_LENGTH 7

  #define ERROR_CODE_LENGTH 16
  #define ERROR_VOLTAGE_LENGTH 16
  #define ERROR_CURRENT_LENGTH 16
  #define ERROR_MOTOR_TEMP_LENGTH 8
  #define ERROR_DRIVER_TEMP_LENGTH 8

  // Commands
  #define DISABLE_MOTOR 1
  #define ENABLE_MOTOR 2
  #define CALIBRATE_START 3
  #define CONTROL_MOTOR 4
  #define RESET_MOTOR 5
  #define SET_HOME 6
  #define SET_GEAR 7
  #define SET_ID 8
  #define SET_CAN_TIMEOUT 9
  #define SET_BANDWIDTH 10
  #define SET_LIMIT_CURRENT 11
  #define SET_UNDER_VOLTAGE 12
  #define SET_OVER_VOLTAGE 13
  #define SET_MOTOR_TEMPERATURE 14
  #define SET_DRIVE_TEMPERATURE 15
  #define SAVE_CONFIG 16
  #define ERROR_RESET 17
  #define WRITE_APP_BACK_START 18
  #define WRITE_APP_BACK 19
  #define CHECK_APP_BACK 20
  #define DFU_START 21
  #define GET_FW_VERSION 22
  #define GET_STATUS_WORD 23
  #define GET_CONFIG 24
  #define CALIB_REPORT 31

  // SendDLC
  #define SEND_DLC_DISABLE_MOTOR 0
  #define SEND_DLC_ENABLE_MOTOR 0
  #define SEND_DLC_CALIBRATE_START 0
  #define SEND_DLC_CONTROL_MOTOR 8
  #define SEND_DLC_RESET_MOTOR 0
  #define SEND_DLC_SET_HOME 0
  #define SEND_DLC_SET_GEAR 2
  #define SEND_DLC_SET_ID 1
  #define SEND_DLC_SET_CAN_TIMEOUT 1
  #define SEND_DLC_SET_BANDWIDTH 2
  #define SEND_DLC_SET_LIMIT_CURRENT 2
  #define SEND_DLC_SET_UNDER_VOLTAGE 2
  #define SEND_DLC_SET_OVER_VOLTAGE 2
  #define SEND_DLC_SET_MOTOR_TEMPERATURE 2
  #define SEND_DLC_SET_DRIVE_TEMPERATURE 2
  #define SEND_DLC_SAVE_CONFIG 0
  #define SEND_DLC_ERROR_RESET 0
  #define SEND_DLC_WRITE_APP_BACK_START 0
  #define SEND_DLC_WRITE_APP_BACK 8
  #define SEND_DLC_CHECK_APP_BACK 8
  #define SEND_DLC_DFU_START 0
  #define SEND_DLC_GET_FW_VERSION 0
  #define SEND_DLC_GET_STATUS_WORD 0
  #define SEND_DLC_GET_CONFIG 0
  #define SEND_DLC_CALIB_REPORT 8
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
//void CAN_Send(uint16_t id, uint8_t *buf);
void OutPut(uint8_t airway1, uint16_t output1, uint8_t airway2,uint16_t output2,uint8_t cmd);
//void SendAngle1_CAN(uint16_t angle1, uint16_t angle2,uint16_t angle3, uint16_t angle4);
//void SendAngle2_CAN(uint16_t angle1, uint16_t angle2,uint16_t angle3, uint16_t angle4);

void CAN_Send(uint16_t id, uint8_t *buf);    // 数据发鿁函敿
void SendAngle_CAN(int16_t angle1, int16_t angle2, int16_t angle3, int16_t angle4, int16_t angle5, short i);
void SendShouldAngle(uint16_t id, int16_t S_angle, uint16_t speedLimit);
void MotorOpen(uint8_t id);
void MotorClose(uint8_t id);
void MotorSetZero(uint8_t id);
void left_arm_button_1_press();
void left_arm_button_2_press();
void right_arm_button_1_press();
void right_arm_button_2_press();
void Send_MOVP(int8_t joint_id , int16_t angle[4]);
void Send_DEEP_Motor(int8_t MOTOR_id , uint16_t MOTOR_MAT[5]);
void turn_on_DEEP_Motor(int8_t MOTOR_id);
void turn_off_DEEP_Motor(int8_t MOTOR_id);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

