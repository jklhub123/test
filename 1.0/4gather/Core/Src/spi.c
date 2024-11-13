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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
int abc;

spi_channel spi_group[4] = {
    {
        .GPIOx = GPIOD,
        .Pin = SPI2_CS1_Pin,
        .hspi = &hspi2,
    },
    {
        .GPIOx = GPIOD,
        .Pin = SPI2_CS2_Pin,
        .hspi = &hspi2,
    },
    {
        .GPIOx = GPIOD,
        .Pin = SPI2_CS3_Pin,
        .hspi = &hspi2,
    },
    {
        .GPIOx = GPIOD,
        .Pin = SPI2_CS4_Pin,
        .hspi = &hspi2,
    },
};

uint16_t Mechanical_Angel[10][5];
uint16_t Angel_Zero[10][5];
int16_t angle[2][4][4];
short angle_ReadIndex = 0;

void READ_Angle(uint8_t num)
{
  uint8_t i, j;
  uint16_t data[4];
  uint32_t avetemp[4] = {0};

  uint8_t CMD_READ[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // 0xffff
  uint8_t CMD_EF[8] = {0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40};   // 0x4001
  uint8_t CMD_NOP[8]= {0x00, 0xc0, 0x00, 0xc0, 0x00, 0xc0, 0x00, 0xc0};    // 0x0000
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(spi_group[num].hspi, CMD_READ, (uint8_t *)data, 4, HAL_MAX_DELAY);//读取四次，存入data数组
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(spi_group[num].hspi, CMD_NOP, (uint8_t *)data, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(spi_group[num].GPIOx, spi_group[num].Pin, GPIO_PIN_SET);


  for (j = 0; j < 4; j++)
  {
    avetemp[j] += data[3 - j] & 0x3fff;
  }

  for (i = 0; i < 4; i++)
  {
    Mechanical_Angel[num][i] = avetemp[i];
  }
}

/*
读第�?次角度，并设为初始�??
*/
void Angle_Reset()
{
  // Mechanical_Angel[i][j]全部置零
  for (uint8_t i = 0; i < 4; i++)
  {
    READ_Angle(i);
    for (uint8_t j = 0; j < 4; j++)
    {
      Mechanical_Angel[i][j] = 0;
      Angel_Zero[i][j] = 0;
    }
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    READ_Angle(i);
    // osDelay(10);
    for (uint8_t j = 0; j < 4; j++)
    {
      Angel_Zero[i][j] = Mechanical_Angel[i][j];
      // printf("%d-%d A:%d , ", i + 1, j + 1, Angel_Zero[i][j]);
      // printf("%d-%d M:%d , ", i + 1, j + 1, Mechanical_Angel[i][j]);
    }
  }
  printf("\nZero reset!\r\n"); // 置零完成
}
//void Angle_Reset()//读取一次角度，并设为初始值
//{
//	
//	for(uint8_t i=0;i<4;i++)
//	{
//		READ_Angle(i);
//		//osDelay(10);
//		for(uint8_t j=0;j<4;j++)
//		{
//			Angel_Zero[i][j]=Mechanical_Angel[i][j];
//			//printf("a:%d", Angel_Zero[i][j]);
//			//printf("b:%d", Mechanical_Angel[i][j]);
//		}
//	}
//	printf("Zero reset!\r\n");//置零完成
//}


/*
读取臂的角度
*/
void Read_B()
{
  short angle_WriteIndex = 1 - angle_ReadIndex;
  int16_t Angel[10];
  uint8_t i, j;
  uint16_t temp[10];
  uint8_t stemp[8];
  float test[4][4];
  for (i = 0; i < 4; i++)
  {

    READ_Angle(i);
		HAL_Delay(100);
    for (j = 0; j < 4; j++)
    {
      Angel[j] = Mechanical_Angel[i][j] - Angel_Zero[i][j];
      // printf("Angel = %d\n",Angel[j]);
      if (Angel[j] < -8192)
        Angel[j] += 16384;
      if (Angel[j] > 8192)
        Angel[j] -= 16384;
      temp[j] = Angel[j];
      angle[angle_WriteIndex][i][j] = Angel[j];
      test[i][j] = ((float)Angel[j]) / 8192 * 180;
    }

    printf("B-%d---angle1 = %.3f   angle2 = %.3f  angle3 = %.3f  angle4 = %.3f \r\n", 
		i + 1,  test[i][0],test[i][1],test[i][2],test[i][3]);

    SendAngle_CAN(Angel[0], Angel[1], Angel[2], Angel[3], i);
    // set_computer_value(SEND_FACT_CMD, i, temp, 4);
    // geng xin shuang huan chong qu suo yin
    angle_ReadIndex = angle_WriteIndex;
  }
}

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
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

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
