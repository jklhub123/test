/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.c
 * @brief   This file provides code for the configuration
 *          of the SPI instances.
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
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* USER CODE BEGIN 0 */
#include <stdint.h>
// #include <stdlib.h> // 用于qsort函数
#include "stdio.h"
#include "main.h"
#include "can.h"
#include "protocol.h"
#include "cmsis_os2.h"
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
extern uint16_t last_angle[4];

spi_channel spi_group[4] = {
    {
        .GPIOx = GPIOE,
        .Pin = SPI2_CS3_Pin,
        .hspi = &hspi2,
    },
    {
        .GPIOx = GPIOE,
        .Pin = SPI2_CS4_Pin,
        .hspi = &hspi2,
    },
    //		{
    //        .GPIOx = GPIOE,
    //        .Pin = SPI2_CS3_Pin,
    //			  .hspi = &hspi2,
    //    },
    //		{
    //        .GPIOx = GPIOE,
    //        .Pin = SPI2_CS4_Pin,
    //			  .hspi = &hspi2,
    //    },
    {
        .GPIOx = GPIOC,
        .Pin = SPI1_CS1_Pin,
        .hspi = &hspi1,
    },
    {
        .GPIOx = GPIOC,
        .Pin = SPI1_CS2_Pin,
        .hspi = &hspi1,
    },
};

uint16_t Mechanical_Angel[4][5];
uint16_t Angel_Zero[4][5];
// uint16_t angle[2][4];//????pid???????????
// shuang huan chong quan ju bian liang
int16_t angle[2][4][5];
short angle_ReadIndex = 0;

void Angle_Reset() // 读取�?次角度，并设为初始�??
{
  // Mechanical_Angel[i][j]全部置零
  for (uint8_t i = 0; i < 4; i++)
  {
    READ_Angle(i);
    for (uint8_t j = 0; j < 5; j++)
    {
      Mechanical_Angel[i][j] = 0;
      Angel_Zero[i][j] = 0;
    }
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    READ_Angle(i);
    // osDelay(10);
    for (uint8_t j = 0; j < 5; j++)
    {
      Angel_Zero[i][j] = Mechanical_Angel[i][j];
      // printf("a:%d", Angel_Zero[i][j]);
      // printf("b:%d", Mechanical_Angel[i][j]);
    }
  }
  printf("Zero reset!\r\n"); // 置零完成
}

void READ_CMD(uint8_t num, uint16_t *data)
{
  // uint8_t i;
  uint8_t CMD_READ[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // 0xffff
  // uint8_t CMD_EF[10] =   {0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40};   // 0x4001
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(spi_group[num].hspi, CMD_READ, 5, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_SET);
  __NOP;
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(spi_group[num].hspi, CMD_READ, (uint8_t *)data, 5, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_SET);
  __NOP;
}

void READ_Angle(uint8_t num)
{
  uint8_t i, j;
  // uint8_t CMD_READ[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // 0xffff
  // uint8_t CMD_EF[10] =   {0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40};   // 0x4001
  uint16_t data[5];
  uint32_t avetemp[5] = {0};

  READ_CMD(num, data);
  for (j = 0; j < 5; j++)
  {
    avetemp[j] += data[4 - j] & 0x3fff;
  }

  for (i = 0; i < 5; i++)
  {
    Mechanical_Angel[num][i] = avetemp[i];
  }
}
/**
臂读取-无过滤
****/
// void Read_B()
// {
//   short angle_WriteIndex = 1 - angle_ReadIndex;
//   int16_t Angel[10];
//   uint8_t i, j;
//   for (i = 0; i < 4; i++)
//   {
//     READ_Angle(i);
//     for (j = 0; j < 5; j++)
//     {
//       Angel[j] = Mechanical_Angel[i][j] - Angel_Zero[i][j];
//       if (Angel[j] < -8192)
//         Angel[j] += 16384;
//       if (Angel[j] > 8192)
//         Angel[j] -= 16384;
// //			Angle_now = Angel[j];
//       angle[angle_WriteIndex][i][j] = Angel[j];
//     }
//     // SendAngle_CAN(Angel[0], Angel[1], Angel[2], Angel[3], Angel[4], i);
//     SendAngle_CAN(Angel[0], Angel[1], 0, 0, 0, i);
//     angle_ReadIndex = angle_WriteIndex;
//   }
// }

/*
臂读取-采用了中值过滤法过滤掉突变值
*/
uint8_t historyIndex[4][5] = {0}; // 每个角度的历史索引，4条链5个关节，每个关节有自己的历史索引
const uint8_t filterSize = 5;     // 滤波器的大小
int16_t temp[filterSize];         // 用于存储滤波器中的值

// 用于存储每个关节的历史值，每条链有4个关节，每个关节存储filterSize个历史值
int16_t history[4][5][filterSize] = {0};

// 插入排序函数，用于对窗口中的历史值进行排序
void insertionSort(int16_t arr[], uint8_t n)
{
  for (uint8_t i = 1; i < n; i++)
  {
    int16_t key = arr[i];
    int8_t j = i - 1;
    // 将比key大的元素向后移动
    while (j >= 0 && arr[j] > key)
    {
      arr[j + 1] = arr[j];
      j = j - 1;
    }
    arr[j + 1] = key;
  }
}

void Read_B()
{

  short angle_WriteIndex = 1 - angle_ReadIndex;
  int16_t Angel[5];
  uint8_t i, j;
  int16_t filteredAngel[5]; // 用于存储滤波后的角度值

  for (i = 0; i < 4; i++) // 遍历4条链
  {
    READ_Angle(i);          // 读取当前链的角度
    for (j = 0; j < 5; j++) // 遍历每条链的5个关节
    {
      Angel[j] = Mechanical_Angel[i][j] - Angel_Zero[i][j];

      if (Angel[j] < -8192)
      {
        Angel[j] += 16384;
      }
      if (Angel[j] > 8192)
      {
        Angel[j] -= 16384;
      }

      // 更新历史值
      history[i][j][historyIndex[i][j]] = Angel[j];

      // 计算中值滤波
      for (uint8_t k = 0; k < filterSize; k++)
      {
        temp[k] = history[i][j][(historyIndex[i][j] + k) % filterSize]; // 获取历史值
      }

      // 更新历史索引
      historyIndex[i][j] = (historyIndex[i][j] + 1) % filterSize;

      // 使用插入排序对temp数组进行排序
      insertionSort(temp, filterSize);

      // // 对temp数组进行排序，使用qsort进行中值计算
      // // *注意qsort函数复杂度较高，用插入排序效果更好。若要启用记得include<stdlib>打开*
      // qsort(temp, filterSize, sizeof(int16_t), compare);

      // 取排序后的中间值作为滤波结果
      filteredAngel[j] = temp[filterSize / 2];

      // 存储滤波后的角度值
      angle[angle_WriteIndex][i][j] = filteredAngel[j];
    }

    // 发送滤波后的角度值
    SendAngle_CAN(filteredAngel[0], filteredAngel[1], filteredAngel[2], filteredAngel[3], filteredAngel[4], i);
  }

  // 更新读取索引
  angle_ReadIndex = angle_WriteIndex;

}

// 比较函数，用于qsort排序
int compare(const void *a, const void *b)
{
  return (*(int16_t *)a - *(int16_t *)b);
}

// /*
// 臂读取-采用了低通滤波高通截止方法过滤掉了突变值
// */
// // 全局或静态变量来保存历史值
// static int history[4][5] = {0}; // 初始化为0
// void Read_B()
// {
//   short angle_WriteIndex = 1 - angle_ReadIndex;
//   int16_t Angel[4][10];
//   uint8_t i, j;

//   for (i = 0; i < 4; i++)
//   {
//     READ_Angle(i);

//     for (j = 0; j < 5; j++)
//     {
//       // 计算当前角度值
//       Angel[i][j] = Mechanical_Angel[i][j] - Angel_Zero[i][j];

//       if (Angel[i][j] < -8192)
//       {
//         Angel[i][j] += 16384;
//       }
//       if (Angel[i][j] > 8192)
//       {
//         Angel[i][j] -= 16384;
//       }

//       // 判断当前值和历史值的差异
//       if (abs(Angel[i][j] - history[i][j]) >= 3000)
//       {
//         printf("angel:%.2f\r\n",
//                (float)Angel[i][j] / 8192.0 * 180.0);
//         printf("there is an error\r\n");
//         Angel[i][j] = history[i][j]; // 当差值超过阈值时，当前值设为历史值
//         printf("angel:%.2f\t, history:%.2f\r\n",
//                (float)Angel[i][j] / 8192.0 * 180.0,
//                (float)history[i][j] / 8192.0 * 180.0);
//         printf("-------------------------------------------------------\t\r\n");
//         osDelay(500); // 延迟500ms
//       }
//       // 更新历史值
//       history[i][j] = Angel[i][j];
//       // 更新全局角度数据
//       angle[angle_WriteIndex][i][j] = Angel[i][j];
//     }
//     // 发送CAN数据
//     SendAngle_CAN(Angel[i][0], Angel[i][1], Angel[i][2], Angel[i][3], Angel[i][4], i);
//   }
// 更新读取索引
// angle_ReadIndex = angle_WriteIndex;
// }

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}
/* SPI3 init function */
void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (spiHandle->Instance == SPI1)
  {
    /* USER CODE BEGIN SPI1_MspInit 0 */

    /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI1_MspInit 1 */

    /* USER CODE END SPI1_MspInit 1 */
  }
  else if (spiHandle->Instance == SPI2)
  {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    PB10     ------> SPI2_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI2_MspInit 1 */

    /* USER CODE END SPI2_MspInit 1 */
  }
  else if (spiHandle->Instance == SPI3)
  {
    /* USER CODE BEGIN SPI3_MspInit 0 */

    /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI3_MspInit 1 */

    /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)
{

  if (spiHandle->Instance == SPI1)
  {
    /* USER CODE BEGIN SPI1_MspDeInit 0 */

    /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    /* USER CODE BEGIN SPI1_MspDeInit 1 */

    /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if (spiHandle->Instance == SPI2)
  {
    /* USER CODE BEGIN SPI2_MspDeInit 0 */

    /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    PB10     ------> SPI2_SCK
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2 | GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    /* USER CODE BEGIN SPI2_MspDeInit 1 */

    /* USER CODE END SPI2_MspDeInit 1 */
  }
  else if (spiHandle->Instance == SPI3)
  {
    /* USER CODE BEGIN SPI3_MspDeInit 0 */

    /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);

    /* USER CODE BEGIN SPI3_MspDeInit 1 */

    /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
