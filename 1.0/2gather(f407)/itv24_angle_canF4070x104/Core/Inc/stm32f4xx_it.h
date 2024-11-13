/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void CAN1_RX1_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void TIM7_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
/* USER CODE BEGIN EFP */
#define FRAME_HEADER     0x4C4246    // 帧头

/* 通道宏定�? */
#define CURVES_CH0      0x00
#define CURVES_CH1      0x01
#define CURVES_CH2      0x02
#define CURVES_CH3      0x03
#define CURVES_CH4      0x04
#define CURVES_CH5      0x05
#define CURVES_CH6      0x06
#define CURVES_CH7      0x07
#define CURVES_CH8      0x08

/* 指令(下位�? -> 上位�?) */
#define SEND_TARGET_CMD      0x01     // 发�?�上位机通道的目标�??
#define SEND_FACT_CMD        0x02     // 发�?��?�道实际�?
#define SEND_P_I_D_CMD       0x03     // 发�?? PID 值（同步上位机显示的值）
#define SEND_START_CMD       0x04     // 发�?�启动指令（同步上位机按钮状态）
#define SEND_STOP_CMD        0x05     // 发�?�停止指令（同步上位机按钮状态）
#define SEND_PERIOD_CMD      0x06     // 发�?�周期（同步上位机显示的值）

/* 指令(上位�? -> 下位�?) */
#define SET_P_I_D_CMD        0x10     // 设置 PID �?
#define SET_TARGET_CMD       0x11     // 设置目标�?
#define START_CMD            0x12     // 启动指令
#define STOP_CMD             0x13     // 停止指令
#define RESET_CMD            0x14     // 复位指令
#define SET_AIRWAY_CMD       0x15     // 设置气道
#define SEND_PRESSURE_VALUE  0x16     // 发�?�气�?

/* 空指�? */
#define CMD_NONE             0xFF     // 空指�?

/* 索引值宏定义 */
//#define HEAD_INDEX_VAL       0x2u     // 包头索引值（3字节�?
#define CHX_INDEX_VAL        0x3u     // 通道索引值（1字节�?
#define LEN_INDEX_VAL        0x4u     // 包长索引值（1字节�?
#define CMD_INDEX_VAL        0x5u     // 命令索引值（1字节�?

#define EXCHANGE_H_L_BIT(data)      ((((data) << 24) & 0xFF000000) |\
                                     (((data) <<  8) & 0x00FF0000) |\
                                     (((data) >>  8) & 0x0000FF00) |\
                                     (((data) >> 24) & 0x000000FF))     // 交换高低字节

#define COMPOUND_32BIT(data)        (((*(data-0) << 24) & 0xFF000000) |\
                                     ((*(data-1) << 16) & 0x00FF0000) |\
                                     ((*(data-2) <<  8) & 0x0000FF00) |\
                                     ((*(data-3) <<  0) & 0x000000FF))      // 合成为一个字
  

#define COMPOUND_16BIT(data)        (((*(data-0) << 8) & 0xFF00) |\
                                     ((*(data-1) << 0) & 0x00FF))
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
