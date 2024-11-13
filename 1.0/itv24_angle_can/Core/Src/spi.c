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
#include "cmsis_os2.h"
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

spi_channel spi_group[2] = {
    {
        .GPIOx = GPIOD,
        .Pin = SPI3_CS1_Pin,
    },
    {
        .GPIOx = GPIOD,
        .Pin = SPI3_CS2_Pin,
    },
};

uint16_t Mechanical_Angel[2][4];
uint16_t Angel_Zero[2][4];
// uint16_t angle[2][4];//????pid???????????
// shuang huan chong quan ju bian liang
int16_t angle[2][2][4];
short angle_ReadIndex = 0;

void Angle_Reset()//读取一次角度，并设为初始值
{
  //Mechanical_Angel[i][j]全部置零
  for(uint8_t i=0;i<2;i++)
	{
    READ_Angle(i);
    for(uint8_t j=0;j<4;j++)
		{
      Mechanical_Angel[i][j]=0;
			Angel_Zero[i][j]=0;
    }
  }

	for(uint8_t i=0;i<2;i++)
	{
		READ_Angle(i);
		//osDelay(10);
		for(uint8_t j=0;j<4;j++)
		{
			Angel_Zero[i][j]=Mechanical_Angel[i][j];
			//printf("a:%d", Angel_Zero[i][j]);
			//printf("b:%d", Mechanical_Angel[i][j]);
		}
	}
	printf("Zero reset!\r\n");//置零完成
}

void READ_CMD(uint8_t num, uint16_t *data)
{
  uint8_t i;
  uint8_t CMD_READ[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // 0xffff
  uint8_t CMD_EF[8] = {0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40};   // 0x4001
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, CMD_READ, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_SET);
  __NOP;
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi3, CMD_READ, (uint8_t *)data, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_SET);
  __NOP;
}

void READ_Angle(uint8_t num)
{
  uint8_t i, j;
  uint8_t CMD_READ[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // 0xffff
  uint8_t CMD_EF[8] = {0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40};   // 0x4001
  uint16_t data[4];
  uint32_t avetemp[4] = {0};
  
    READ_CMD(num, data);
    for (j = 0; j < 4; j++)
    {
      avetemp[j] += data[3 - j] & 0x3fff;
    }
  
  for (i = 0; i < 4; i++)
  {
    Mechanical_Angel[num][i] = avetemp[i];
  }
}
/**
???
****/
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
      					printf("angle%d-%d = %.3f\n",i+1,j+1,test);
    }
    if (i == 0)
    {
      SendAngle1_CAN(Angel[0], Angel[1], Angel[2], Angel[3]);
      
    }
    else
    {
      SendAngle2_CAN(Angel[0], Angel[1], Angel[2], Angel[3]);
      
    }
    //			set_computer_value(SEND_FACT_CMD, i, temp, 4);
    // geng xin shuang huan chong qu suo yin
    angle_ReadIndex = angle_WriteIndex;
  }
}

/* USER CODE END 0 */

SPI_HandleTypeDef hspi3;

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
  if(spiHandle->Instance==SPI3)
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
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI3)
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
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
