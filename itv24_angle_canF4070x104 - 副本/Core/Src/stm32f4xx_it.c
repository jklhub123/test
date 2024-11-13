/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "protocol.h"
#include "usart.h"
#include "tim.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN EV */
extern uint8_t UART_RCV_DATA;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  printf("HardFault_Handler");
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  // HAL_UART_Receive_IT(&huart1, &UART_RCV_DATA, 1);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  // HAL_UART_Receive_IT(&huart2, &UART_RCV_DATA, 1);
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
// in data
uint8_t UART_RCV_BUFFER[10] = {0Xff};
// in tag
uint8_t rcvstate = 0x00;

// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 处理USART1串口接收
  if (huart->Instance == USART1)
  {
    switch (aRxBuffer[0])
    {
    case 0x42:
      if (rcvstate == 0x00)
      {
        // 如果接收到字�?0x42，设置数据类型为 'B'，计数为1
        UART_RCV_BUFFER[0] = 0x42;
        rcvstate = 0x81;
      }
      break;

    case 0x4C:
      // 如果接收到字�?0x4C且之前已接收�? 'B'，设置数据类型为 'L'，计数为2
      if (rcvstate == 0x81)
      {
        rcvstate = 0x82;
        UART_RCV_BUFFER[1] = 0x4C;
      }
      else
      {
        rcvstate = 0x00; // 重置状�??
      }
      break;

    case 0x46:
      // 如果接收到字�?0x46且之前已接收�? 'L'，设置数据类型为 'F'，计数为3
      if (rcvstate == 0x82)
      {
        rcvstate = 0x83;
        UART_RCV_BUFFER[2] = 0x46;
      }
      else
      {
        rcvstate = 0x00; // 重置状�??
      }
      break;

    default:
      // 处理其他情况
      if (rcvstate >= 0x83)
      {
        // 判断是否已经接收到包头，将数据存储到对应位置，数据计数加1
        UART_RCV_BUFFER[rcvstate - 0X80] = aRxBuffer[0];
        rcvstate += 1;
      }
      else
      {
        rcvstate = 0x00; // 重置状�??
      }

      // 如果数据接收完成
      if ((rcvstate - 0X80) == UART_RCV_BUFFER[4])
      {
        rcvstate = 0x20; // 设置完成标志

        // 解析数据
        uint8_t CMD_temp = UART_RCV_BUFFER[5];
        switch (CMD_temp)
        {
        case 0X01:
          HAL_GPIO_TogglePin(LED0_GPIO_Port, LED1_Pin);
          break;
        case 0X02:
          HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
          break;
        // case SET_P_I_D_CMD:
        //   HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        //   break;
        // case SET_TARGET_CMD:
        //   HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        //   break;
        case START_CMD:
          HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
          break;
        case STOP_CMD:
          HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
          break;
          // case RESET_CMD:
          //   HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
          //   break;
          // case SEND_PRESSURE_VALUE:
          //   HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
          break;
        case 0X30:
          uint8_t pressurech = UART_RCV_BUFFER[6];
          uint16_t pressurevalue = (UART_RCV_BUFFER[7] << 8) | UART_RCV_BUFFER[8];
          if (pressurech == 0X10)
          {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
          }
          if (pressurevalue == 1000)
          {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
          }

          break;

        default:
          break;
        }

        rcvstate = 0x00;           // 重置状�??
        UART_RCV_BUFFER[4] = 0xFF; // 重置校验�?
      }
      break;
    }
    if (rcvstate == 0x20)
    {
      // 数据接收完成后的处理

      rcvstate = 0x00; // 重置状�??
    }
  }

  else if (huart->Instance == USART2) // 处理蓝牙串口接收
  {
    switch (aRxBuffer[0]) // 根据接收到的第一个字节判断数据类�?
    {
    case 'A': // 数据类型�? 'A'
      UART_RCV_BUFFER[0] = 'A';
      rcvstate |= 0x81; // 设置A标志和计数为1
      break;
    case 'B':               // 数据类型�? 'B'
      if (rcvstate == 0x83) // 如果已经接收�? 'A' 并且数据计数�?3
      {
        rcvstate = 0xC4; // 设置A到B的标志，计数�?4
        UART_RCV_BUFFER[3] = 'B';
      }
      else
      {
        rcvstate = 0x00; // 重置状�??
      }
      break;
    default:                       // 其他情况
      if (aRxBuffer[0] - '0' <= 9) // 判断是否为数�?
      {
        if (rcvstate & 0xC0) // 如果状�?�为 0x8X �? 0xCX，则作为数据处理
        {
          UART_RCV_BUFFER[rcvstate & 0x0F] = aRxBuffer[0]; // 将数据存储到对应位置
          rcvstate += 1;                                   // 数据计数�?1
        }
        else
        {
          rcvstate = 0x00; // 重置状�??
        }
      }
      else // 非数字字�?
      {
        rcvstate = 0x00; // 重置状�??
      }
      if ((rcvstate & 0x0F) == 8) // 数据接收完成
      {
        rcvstate |= 0x20; // 设置完成标志
        // 解析数据
        uint32_t CMD_temp = 0x00000000;
        CMD_temp = 10 * (UART_RCV_BUFFER[1] - '0') + (UART_RCV_BUFFER[2] - '0');
        CMD_temp <<= 16;
        CMD_temp += 1000 * (UART_RCV_BUFFER[4] - '0') + 100 * (UART_RCV_BUFFER[5] - '0') + 10 * (UART_RCV_BUFFER[6] - '0') + 1 * (UART_RCV_BUFFER[7] - '0');

        rcvstate = 0x00;                        // 重置状�??
        uint8_t channel = CMD_temp >> 16;       // 获取通道
        uint16_t value = CMD_temp & 0x0000FFFF; // 获取数�??

        // 处理接收到的数据
        if (channel < 24)
        {
          ctrl_group[channel].in_value = (int)value / 10;
          HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&channel, 1);
          __HAL_TIM_SET_COMPARE(ctrl_group[channel].htim, ctrl_group[channel].tim_channel, ctrl_group[channel].in_value);
        }

        if (channel == 99)
        {
          // 处理接收到的数据
        }

        if (channel == 88)
        {
          // 处理接收到的数据
          for (uint8_t i = 0; i < 24; i++)
          {
            ctrl_group[i].in_value = 0;
            __HAL_TIM_SET_COMPARE(ctrl_group[i].htim, ctrl_group[i].tim_channel, ctrl_group[i].in_value);
          }
        }
      }
      break;
    }
  }
  // 继续接收下一个字�?
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer, 1); // 使能蓝牙串口中断接收
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1); // 使能USB串口中断接收
}

/*
data  0     1 2 3 4 5 6 7
      cmd


*/

// can receive callback function
// CAN接收FIFO0消息挂起回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  // �?查CAN实例是否为CAN1
  if (hcan->Instance == CAN1)
  {
    // 获取CAN消息
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      // 获取CAN消息的标识符
      uint16_t qifa_id = RxHeader.StdId;

      // 创建缓冲区，用于存储待发送的字符串消�?
      char aTxBuffer[64];

      switch (qifa_id)
      {
      case 0x101:
      {
        // 解析CAN数据
        int pressure_ch1 = RxData[0];
        int pressure_ch2 = RxData[4];
        int pressure1 = (RxData[2] << 8) | RxData[3];
        int pressure2 = (RxData[6] << 8) | RxData[7];
        pressure[pressure_ch1] = pressure1;
        pressure[pressure_ch2] = pressure2;

        // 创建格式化字符串消息
        sprintf((char *)aTxBuffer, "p%d:%d, p%d:%d\r\n", pressure_ch1, pressure1, pressure_ch2, pressure2);
        // 通过UART使用DMA传输字符串消�?
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)aTxBuffer, strlen(aTxBuffer));

        // 处理压力数据
        if (pressure_ch1 < 24)
        {
          ctrl_group[pressure_ch1].in_value = pressure1 / 10;
          // 设置定时器�?�道的比较�??
          __HAL_TIM_SET_COMPARE(ctrl_group[pressure_ch1].htim, ctrl_group[pressure_ch1].tim_channel, ctrl_group[pressure_ch1].in_value);
        }

        if (pressure_ch2 < 24)
        {
          ctrl_group[pressure_ch2].in_value = pressure2 / 10;
          // 设置定时器�?�道的比较�??
          __HAL_TIM_SET_COMPARE(ctrl_group[pressure_ch2].htim, ctrl_group[pressure_ch2].tim_channel, ctrl_group[pressure_ch2].in_value);
        }

        break;
      }
      case 0x102:
      {
        float angle[10];
        int16_t anglerx[10];
        anglerx[0] = (RxData[0] << 8) | RxData[1];
        anglerx[1] = (RxData[2] << 8) | RxData[3];
        anglerx[2] = (RxData[4] << 8) | RxData[5];
        anglerx[3] = (RxData[6] << 8) | RxData[7];
        for (int i = 0; i < 4; i++)
        {
          angle[i] = ((float)anglerx[i]) / 8192 * 180;
        }

        sprintf((char *)aTxBuffer, "angle1-1:%.3f, angle1-2:%.3f, angle1-3:%.3f, angle1-4:%.3f\r\n", angle[0], angle[1], angle[2], angle[3]);
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)aTxBuffer, strlen(aTxBuffer));

        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

        break;
      }
      case 0x103:
      {
        float angle[10];
        int16_t anglerx[10];
        anglerx[0] = (RxData[0] << 8) | RxData[1];
        anglerx[1] = (RxData[2] << 8) | RxData[3];
        anglerx[2] = (RxData[4] << 8) | RxData[5];
        anglerx[3] = (RxData[6] << 8) | RxData[7];
        for (int i = 0; i < 4; i++)
        {
          angle[i] = ((float)anglerx[i]) / 8192 * 180;
        }

        sprintf((char *)aTxBuffer, "angle2-1:%.3f, angle2-2:%.3f, angle2-3:%.3f, angle2-4:%.3f\r\n", angle[0], angle[1], angle[2], angle[3]);
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)aTxBuffer, strlen(aTxBuffer));

        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

        break;
      }

        // 处理其他CAN消息
        // ...
      }
    }
  }
}

/* USER CODE END 1 */
