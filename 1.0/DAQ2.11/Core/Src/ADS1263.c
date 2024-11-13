#include "ADS1263.h"
#include "stdio.h"
#include "protocol.h"

uint8_t ADS1263_ID;
uint8_t REG[27];
uint8_t STATUS;
uint32_t ADC1_DATA,ADC2_DATA;
double Voltage;
double vol_print[5] = {0};

void Delay(uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}


void ADS1263_INIT(void)
{
  ADS1263_RESET_H;
  Delay(0x1fffff);//等待至少65535个ADC时钟
  ADS1263_START_L;//停止ADC转换，避免寄存器配置出错
  Delay(0x1fff);
  ADS1263_START_H;
  
  ADS1263_WRITE(0X41);//写入起始寄存器地址 Power Register
  ADS1263_WRITE(0x19);//写入的寄存器个数 26个
  
  ADS1263_WRITE(0x11);//复位默认值0x11，使能内部基准源
  ADS1263_WRITE(0x00);//复位默认值0x05
  ADS1263_WRITE(0x00);//MODE0
  ADS1263_WRITE(0x80);//MODE1
  ADS1263_WRITE(0x0A);//MODE2(0x09:1200sps;0x0a:2400sps;0x04:20sps;0x02:10sps)
  ADS1263_WRITE(0x01);//Input Multiplexer Register
  
  ADS1263_WRITE(0x00);//Offset Calibration Registers 1
  ADS1263_WRITE(0x00);
  ADS1263_WRITE(0x00);
  ADS1263_WRITE(0x00);//Full-Scale Calibration Registers
  ADS1263_WRITE(0x00);
  ADS1263_WRITE(0x40);
  
  ADS1263_WRITE(0xBB);//IDACMUX Register内部可控电流源控制寄存器
  
  ADS1263_WRITE(0x00);//IDACMAG Register
  ADS1263_WRITE(0x00);//REFMUX Register
  ADS1263_WRITE(0x00);//TDACP Control Register
  ADS1263_WRITE(0x00);//TDACN Control Register
  ADS1263_WRITE(0x00);//GPIO Connection Register
  ADS1263_WRITE(0x00);//GPIO Direction Register
  ADS1263_WRITE(0x00);//GPIO Data Register
  
  ADS1263_WRITE(0x00);//ADC2 Configuration Register
  ADS1263_WRITE(0x01);//ADC2 Input Multiplexer Register  
  ADS1263_WRITE(0x00);//ADC2 Offset Calibration Registers
  ADS1263_WRITE(0x00);
  ADS1263_WRITE(0x00);//ADC2 Full-Scale Calibration Registers
  ADS1263_WRITE(0x40);
	

}

double vol_val_convertion(uint32_t adc_data)
{
	int32_t adc_temp;
	double vol_val[1], vol_temp[10];
	adc_data=adc_data^0x80000000;
	adc_temp=adc_data-0x80000000;
	vol_temp[0]=adc_temp*2.5;
	vol_val[0]=vol_temp[0]/0x80000000*1000;//Voltage为转换得到的电压值
	
	//set_computer_32value(SEND_FACT_CMD, 1, vol_val, 1);
	return vol_val[0];
}

double ads1263_channel_read(uint8_t channel)
{	

	ADS1263_START_L;//停止ADC转换，避免寄存器配置出错
	Delay(0x1fff);
	ADS1263_WRITE(0x42);//写入的首个寄存器地址
  ADS1263_WRITE(0x00);//要写入的寄存器数量-1
	ADS1263_WRITE(0x00);//复位默认值0x00,禁用status,CRC
	ADS1263_WRITE(0x46);//写入的首个寄存器地址
  ADS1263_WRITE(0x00);//要写入的寄存器数量-1
	ADS1263_WRITE(channel);//Input Multiplexer Register
	ADS1263_WRITE(0x08);//START1 command，当START引脚为低电平时，可由此命令启动ADC1的转换。
	
	while(ADS1263_DRDY);
	ADS1263_WRITE(0x12);//读取ADC1
	STATUS=ADS1263_READ_REG();
	
	ADC1_DATA=ADS1263_READ();
	
//	ADS1263_WRITE(0x0A);//STOP1 command
//	ADS1263_CS_H;
	return vol_val_convertion(ADC1_DATA);

}

void ADS1263_print(void)
{

//  ADS1263_WRITE(0x08);//START1 command，当START引脚为低电平时，可由此命令启动ADC1的转换。
//	ADS1263_WRITE(0x46);//写入的首个寄存器地址
//  ADS1263_WRITE(0x00);//要写入的寄存器数量-1
//	ADS1263_WRITE(0x23);//Input Multiplexer Register

//	while(ADS1263_DRDY);
//	ADS1263_WRITE(0x12);//读取ADC1
//	STATUS=ADS1263_READ_REG();
//	ADC1_DATA=ADS1263_READ();
	
	/*
	ADS1263_WRITE(0x14);//读取ADC2
	STATUS=ADS1263_READ_REG();
	ADC2_DATA=ADS1263_READ();
	*/
	vol_print[0] = ads1263_channel_read(ADS1263_channel_1);
//	vol_print[1] = ads1263_channel_read(ADS1263_channel_2);
//	vol_print[2] = ads1263_channel_read(ADS1263_channel_3);
//	vol_print[3] = ads1263_channel_read(ADS1263_channel_4);
//	vol_print[4] = ads1263_channel_read(ADS1263_channel_5);
	
	for(int i=0; i<1; i++)
	{
		printf("%d=%.4fmV\t", i+1,vol_print[i]);
	}
	printf("\r\n");
//    vol_print[1]=ads1263_channel_read(0x23);
////		vol_print[0]=ads1263_channel_read(0x23);
//		printf("ch1=%.4fmV\r\n", vol_print[1]);
//		

}

void ADS1263_WRITE(uint8_t data)
{
  for(uint8_t si=0;si<8;si++)
  {
    ADS1263_SCLK_H;
    if(data&0x80){ADS1263_DIN_H;}
    else{ADS1263_DIN_L;}
    data<<=1;
    Delay(0x1f);
    ADS1263_SCLK_L;
    Delay(0xf);
  }
}

void ADS1263_reset(void)
{
  Delay(0x1fff);
  ADS1263_RESET_L;
  Delay(0x1fff);
  ADS1263_RESET_H;
  Delay(0x1fff);
}


uint32_t ADS1263_READ(void)
{
  uint32_t DATA_FB;
  ADS1263_SCLK_H;
  Delay(0xf);
  DATA_FB=0;
  for(uint8_t si=0;si<32;si++)
  {
    ADS1263_SCLK_H;
    DATA_FB<<=1;
    ADS1263_SCLK_L;
    Delay(0xf);
    if(ADS1263_DOUT)
    {
      DATA_FB++;
    }
  }
  return DATA_FB;
}



uint8_t ADS1263_READ_REG(void)
{
  uint8_t REG;
  ADS1263_SCLK_H;
  Delay(0xf);
  REG=0;
  for(uint8_t si=0;si<8;si++)
  {
    ADS1263_SCLK_H;
    REG<<=1;
    ADS1263_SCLK_L;
    Delay(0xf);
    if(ADS1263_DOUT)
    {
      REG++;
    }
  }
  return REG;
}


