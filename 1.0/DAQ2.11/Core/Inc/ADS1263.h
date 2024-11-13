#ifndef __ADS1263_H_
#define	__ADS1263_H_

#include "main.h"


//***************************
//		Pin assign	   	
//	  STM32			           ADS1263
//		GPIOA_Pin_5 		---> SCLK
//		GPIOA_Pin_6 		<--- DOUT
//		GPIOA_Pin_7 		---> DIN
//		GPIOB_Pin_5 		---> START
//		GPIOB_Pin_11    ---> RESET
//		GPIOC_Pin_4     <--- DRDY
//		GPIOC_Pin_5     ---> CS	

//***************************	

#define PORT_ADS1263_RESET		GPIOG
#define PIN_ADS1263_RESET			ADS_RESET_Pin

#define PORT_ADS1263_START		GPIOG
#define PIN_ADS1263_START			ADS_START_Pin

#define PORT_ADS1263_CS			  GPIOB
#define PIN_ADS1263_CS			  SPI3_CS1_Pin

#define PORT_ADS1263_SCLK			GPIOG
#define PIN_ADS1263_SCLK			ADS_SCLK_Pin

#define PORT_ADS1263_DIN			GPIOG
#define PIN_ADS1263_DIN			  ADS_DIN_Pin

#define PORT_ADS1263_DOUT			GPIOG
#define PIN_ADS1263_DOUT			ADS_DOUT_Pin

#define PORT_ADS1263_DRDY			GPIOG
#define PIN_ADS1263_DRDY			DRDY_Pin

//***************************	*****************************************************************************

#define ADS1263_RESET_L				HAL_GPIO_WritePin(PORT_ADS1263_RESET,PIN_ADS1263_RESET,GPIO_PIN_RESET);
#define ADS1263_RESET_H				HAL_GPIO_WritePin(PORT_ADS1263_RESET,PIN_ADS1263_RESET,GPIO_PIN_SET);

#define ADS1263_START_L				HAL_GPIO_WritePin(PORT_ADS1263_START,PIN_ADS1263_START,GPIO_PIN_RESET);
#define ADS1263_START_H				HAL_GPIO_WritePin(PORT_ADS1263_START,PIN_ADS1263_START,GPIO_PIN_SET);

#define ADS1263_CS_L				HAL_GPIO_WritePin(PORT_ADS1263_CS,PIN_ADS1263_CS,GPIO_PIN_RESET);
#define ADS1263_CS_H				HAL_GPIO_WritePin(PORT_ADS1263_CS,PIN_ADS1263_CS,GPIO_PIN_SET);

#define ADS1263_SCLK_L				HAL_GPIO_WritePin(PORT_ADS1263_SCLK,PIN_ADS1263_SCLK,GPIO_PIN_RESET);
#define ADS1263_SCLK_H				HAL_GPIO_WritePin(PORT_ADS1263_SCLK,PIN_ADS1263_SCLK,GPIO_PIN_SET);

#define ADS1263_DIN_L				HAL_GPIO_WritePin(PORT_ADS1263_DIN,PIN_ADS1263_DIN,GPIO_PIN_RESET);
#define ADS1263_DIN_H				HAL_GPIO_WritePin(PORT_ADS1263_DIN,PIN_ADS1263_DIN,GPIO_PIN_SET);

#define ADS1263_DOUT  		(PORT_ADS1263_DOUT->IDR & PIN_ADS1263_DOUT)
#define ADS1263_DRDY  		(PORT_ADS1263_DRDY->IDR & PIN_ADS1263_DRDY)



#define ADS1263_channel_1 (0x01)
#define ADS1263_channel_2 (0x23)
#define ADS1263_channel_3 (0x45)
#define ADS1263_channel_4 (0x67)
#define ADS1263_channel_5 (0x89)

void Delay(uint32_t nCount);
void ADS1263_WRITE(uint8_t data);
void ADS1263_reset(void);
uint32_t ADS1263_READ(void);//读取数据函数，返回32Bit 整形
uint8_t ADS1263_READ_REG(void);//读取寄存器函数，返回1 Byte
void ADS1263_INIT(void);

void ADS1263_print(void);
double vol_val_convertion(uint32_t adc_data);
double ads1263_channel_read(uint8_t channel);

extern uint8_t REG[27];
extern double vol_print[5];

#endif
