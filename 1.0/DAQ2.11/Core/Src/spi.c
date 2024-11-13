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

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "main.h"
#include "can.h"
#include "protocol.h"
#include "ADS1263.h"
int abc;

// 臂的四条数据链
spi_channel spi_group[2] = {
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
//    {
//        .GPIOx = GPIOB,
//        .Pin = SPI1_CS3_Pin,
//        .hspi = &hspi1,
//    },
//    {
//        .GPIOx = GPIOB,
//        .Pin = SPI1_CS4_Pin,
//        .hspi = &hspi1,
//    },
};

// 修改处
uint16_t Mechanical_Angel[2][4];
uint16_t Angel_Zero[2][4];
int16_t angle[2][2][4];
short angle_ReadIndex = 0;

int16_t Hand_Angel[20];
float Hand_Angel_float[20];
int16_t Arm_Angel[8];
float Arm_Angel_float[8];

void READ_CMD(uint8_t num, uint16_t *data)
{
   uint8_t i;
   uint8_t CMD_READ[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // 0xffff
   uint8_t CMD_EF[8] = {0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40};   // 0x4001
  // // uint8_t CMD_NOP[8] = {0x00, 0xc0,0x00, 0xc0,0x00, 0xc0,0x00, 0xc0};	//0xc000

   HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_RESET);
   HAL_SPI_Transmit(spi_group[num].hspi, CMD_READ, 4, HAL_MAX_DELAY);
   HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_SET);
   __NOP;
   HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_RESET);
   HAL_SPI_TransmitReceive(spi_group[num].hspi, CMD_READ, (uint8_t *)data, 4, HAL_MAX_DELAY);
   HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_SET);
   __NOP;

}

/*
角度读取+滤波
*/
void READ_Angle(uint8_t num)
{
  uint8_t i, j;
  //	uint16_t CMD_READ = 0xffff;
  //	uint16_t CMD_EF = 0x4001;
  //	uint16_t CMD_NOP = 0xc000;
   uint8_t CMD_READ[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // 0xffff
   uint8_t CMD_EF[8] = {0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40};   // 0x4001
  // uint8_t CMD_NOP[8] = {0x00, 0xc0,0x00, 0xc0,0x00, 0xc0,0x00, 0xc0};	//0xc000
  uint16_t data[4];
  uint32_t avetemp[4] = {0};
  //	for(i=0;i<10;i++){
  READ_CMD(num, data);
  for (j = 0; j < 4; j++)
    avetemp[j] += data[3 - j] & 0x3fff;
  //	}

  for (i = 0; i < 4; i++)
  {
    //		avetemp[i]/=10;
    Mechanical_Angel[num][i] = avetemp[i];
  }
}

/**
读取1次角度，并设为初始角度
**/
void Angle_Reset()
{
  // Mechanical_Angel[i][j]全部置零
  for (uint8_t i = 0; i < 2; i++)
  {
    READ_Angle(i);
    for (uint8_t j = 0; j < 4; j++)
    {
      Mechanical_Angel[i][j] = 0;
      Angel_Zero[i][j] = 0;
    }
  }

  for (uint8_t i = 0; i < 2; i++)
  {
    READ_Angle(i);
    // osDelay(10);
    for (uint8_t j = 0; j < 4; j++)
    {
      Angel_Zero[i][j] = Mechanical_Angel[i][j];
      // printf("a:%d", Angel_Zero[i][j]);
      // printf("b:%d", Mechanical_Angel[i][j]);
    }
  }
  printf("Zero reset!\r\n"); // 置零完成
}

/**
遥操作的臂的
**/
void Read_B()
{
  short angle_WriteIndex = 1 - angle_ReadIndex;
  int16_t Angel[10];
  uint8_t i, j;
  uint16_t temp[10];
  uint8_t stemp[8];
  float test;
  for (i = 0; i < 2; i++)
  {

    READ_Angle(i);
    for (j = 0; j < 4; j++)
    {
      Angel[j] = Mechanical_Angel[i][j] - Angel_Zero[i][j];
      // printf("angle = %d",angle[i][j]);
      if (Angel[j] < -8192)
        Angel[j] += 16384;
      if (Angel[j] > 8192)
        Angel[j] -= 16384;
      temp[j] = Angel[j];
      angle[angle_WriteIndex][i][j] = Angel[j];
      test = ((float)Angel[j]) / 8192 * 180;
      printf("angle%d-%d = %.3f\n", i + 1, j + 1, test);
    }
    SendAngle_CAN(Angel[0], Angel[1], Angel[2], Angel[3], i);
    //			set_computer_value(SEND_FACT_CMD, i, temp, 4);
    // geng xin shuang huan chong qu suo yin
    angle_ReadIndex = angle_WriteIndex;
  }
}

/**
手指的
****/
void Read_RTB()
{
  int i, j;
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 3; j++)
    {
      Mechanical_Angel[i][j] = 0;
    }
  }

  printf("A:");
  i = 6;
  READ_Angle(i);
  for (j = 0; j < 4; j++)
  {
    Hand_Angel[j] = Mechanical_Angel[i][j] - Angel_Zero[i][j];
    if (Hand_Angel[j] < -8192)
      Hand_Angel[j] += 16384;
    if (Hand_Angel[j] > 8192)
      Hand_Angel[j] -= 16384;
    printf("%d:%.3f \t", i, (float)Hand_Angel[j] / 8192 * 180);
  }
  printf("\r\n");

  //			i=1;
  //			READ_Angle(i);
  //			for(j=0;j<4;j++){
  //					Angel[j] = Mechanical_Angel[i][j] - Angel_Zero[i][j];
  //					if(Angel[j] < -8192)Angel[j] += 16384;
  //					if(Angel[j] > 8192)Angel[j] -= 16384;
  //					printf("%d:%.3f \t", i,(float)Angel[j]/8192*180 );
  //			}
  //
  //			printf("\r\n");

  HAL_Delay(50);
}

//
void Read_Hand()
{
  // read angle informations of hand
  for (int i = 0; i < 8; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      Mechanical_Angel[i][j] = 0;
    }
  }

  for (int i = 0; i <= 4; i++)
  {
    READ_Angle(i);
    for (int j = 0; j < 4; j++)
    {
      int m = j + 4 * i;
      Hand_Angel[m] = Mechanical_Angel[i][j] - Angel_Zero[i][j];
      if (Hand_Angel[m] < -8192)
        Hand_Angel[m] += 16384;
      if (Hand_Angel[m] > 8192)
        Hand_Angel[m] -= 16384;

      Hand_Angel_float[m] = (float)Hand_Angel[m] / 8192 * 180;
    }
  }
  // read pressure informations of hand
  vol_print[0] = ads1263_channel_read(ADS1263_channel_1);
  vol_print[1] = ads1263_channel_read(ADS1263_channel_2);
  vol_print[2] = ads1263_channel_read(ADS1263_channel_3);
  vol_print[3] = ads1263_channel_read(ADS1263_channel_4);
  vol_print[4] = ads1263_channel_read(ADS1263_channel_5);

  HAL_Delay(10);
}

//
void Read_Arm()
{
  for (int i = 5; i < 8; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      Mechanical_Angel[i][j] = 0;
    }
  }

  for (int i = 5; i <= 6; i++)
  {
    READ_Angle(i);
    for (int j = 0; j < 4; j++)
    {
      int m = j + 4 * (i - 5);
      Arm_Angel[m] = Mechanical_Angel[i][j] - Angel_Zero[i][j];
      if (Arm_Angel[m] < -8192)
        Arm_Angel[m] += 16384;
      if (Arm_Angel[m] > 8192)
        Arm_Angel[m] -= 16384;

      Arm_Angel_float[m] = (float)Arm_Angel[m] / 8192 * 180;
    }
  }

  HAL_Delay(10);
}

void Read_RTB_PID()
{
  int16_t Angel[20];
  int i, j;
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 3; j++)
    {
      Mechanical_Angel[i][j] = 0;
    }
  }

  // printf("A:");
  i = 6;
  READ_Angle(i);
  for (j = 0; j < 4; j++)
  {
    Angel[j] = Mechanical_Angel[i][j] - Angel_Zero[i][j];
    if (Angel[j] < -8192)
      Angel[j] += 16384;
    if (Angel[j] > 8192)
      Angel[j] -= 16384;
    if (j == 1 || j == 3)
      Angel[j] = (-Angel[j]);
  }
  set_computer_value(SEND_FACT_CMD, i, Angel, 4);
  // printf("\r\n");

  HAL_Delay(20);
}
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
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
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI2 interrupt Init */
    HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PB3     ------> SPI3_SCK
    PB4     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI3 interrupt Init */
    HAL_NVIC_SetPriority(SPI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
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
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    /* SPI2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PB3     ------> SPI3_SCK
    PB4     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

    /* SPI3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI3_IRQn);
  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
