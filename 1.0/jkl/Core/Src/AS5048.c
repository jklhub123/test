#include "as5048.h"
#include "main.h"
#include "stdio.h"
#include "spi.h"

/*
STM32ZET6
SPI1,片选引脚为PA4
CS1--PA4
CS2--PA3
CS3--PA2
CS4--PA1
CS5--PA0
CLK--PA5
MISO--PA6
MOSI--PA7
*/

uint16_t Angel_tag[20];
uint16_t Angel_Zero[20];
int16_t Angel[20]={0};

void Angle_Read(int num)//读取角度
{
	//设置读取参数	
	int i;
	uint8_t READ[40];//0xffff
//uint8_t EF[8] = {0x01, 0x40,0x01, 0x40,0x01, 0x40,0x01, 0x40};		//0x4001
	uint8_t NOP[40];//0xc000
	for(i=0;i<40;i=i+2)
	{
		 READ[i] = 0xff;READ[i +1]= 0xff;//读取
		  NOP[i] = 0x00; NOP[i +1]= 0xc0;//空读
	}
	uint16_t data[20];//临时数据
	
	//读取num次角度
	for(int n=num;n>0;n--)
	{

	if(n==5)//CS1--PA4
	{
	//读取一轮角度
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS5_Pin, GPIO_PIN_RESET);//低电平读取
	HAL_SPI_TransmitReceive(&hspi2, READ, (uint8_t *)data, 4, HAL_MAX_DELAY);//读取四次，存入data数组
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS5_Pin, GPIO_PIN_SET);//读取结束
	HAL_Delay(2);
	//空读一轮
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS5_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, NOP, (uint8_t *)data, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS5_Pin, GPIO_PIN_SET);
	HAL_Delay(2);
	
	for(i=1;i<=4;i++) Angel_tag[i+4*(num-n)-1] = data[4-i] & 0x3fff;//将临时数据data传出给tag
	}
	
	if(n==4)//CS2--PA3
	{
	//读取一轮角度	
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS4_Pin, GPIO_PIN_RESET);//低电平读取
	HAL_SPI_TransmitReceive(&hspi2, READ, (uint8_t *)data, 4, HAL_MAX_DELAY);//读取四次，存入data数组
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS4_Pin, GPIO_PIN_SET);//读取结束
	HAL_Delay(2);
	//空读一轮
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS4_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, NOP, (uint8_t *)data, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS4_Pin, GPIO_PIN_SET);
	HAL_Delay(2);
	
	for(i=1;i<=4;i++) Angel_tag[i+4*(num-n)-1] = data[4-i] & 0x3fff;//将临时数据data传出给tag
	}
	
	if(n==3)//CS3--PA2
	{
	//读取一轮角度	
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS3_Pin, GPIO_PIN_RESET);//低电平读取
	HAL_SPI_TransmitReceive(&hspi2, READ, (uint8_t *)data, 4, HAL_MAX_DELAY);//读取四次，存入data数组
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS3_Pin, GPIO_PIN_SET);//读取结束
	HAL_Delay(2);
	//空读一轮
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS3_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, NOP, (uint8_t *)data, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS3_Pin, GPIO_PIN_SET);
	HAL_Delay(2);
	
	for(i=1;i<=4;i++) Angel_tag[i+4*(num-n)-1] = data[4-i] & 0x3fff;//将临时数据data传出给tag
	}
	
	if(n==2)//CS4--PA1
	{
	//读取一轮角度	
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS2_Pin, GPIO_PIN_RESET);//低电平读取
	HAL_SPI_TransmitReceive(&hspi2, READ, (uint8_t *)data, 4, HAL_MAX_DELAY);//读取四次，存入data数组
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS2_Pin, GPIO_PIN_SET);//读取结束
	HAL_Delay(2);
	//空读一轮
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, NOP, (uint8_t *)data, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS2_Pin, GPIO_PIN_SET);
	HAL_Delay(2);
	
	for(i=1;i<=4;i++) Angel_tag[i+4*(num-n)-1] = data[4-i] & 0x3fff;//将临时数据data传出给tag
	}
	
	if(n==1)//CS5--PA0
	{
	//读取一轮角度	
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS1_Pin, GPIO_PIN_RESET);//低电平读取
	HAL_SPI_TransmitReceive(&hspi2, READ, (uint8_t *)data, 4, HAL_MAX_DELAY);//读取四次，存入data数组
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS1_Pin, GPIO_PIN_SET);//读取结束
	HAL_Delay(2);
	//空读一轮
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, NOP, (uint8_t *)data, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port,SPI2_CS1_Pin, GPIO_PIN_SET);
	HAL_Delay(2);
	
	for(i=1;i<=4;i++) Angel_tag[i+4*(num-n)-1] = data[4-i] & 0x3fff;//将临时数据data传出给tag
	}
	
	}
	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(&hspi1,READ,(uint8_t *)data,1,HAL_MAX_DELAY);
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//	Angel_tag[0] = data[0] & 0x3fff;
}

void Angle_Reset(int a)//读取一次角度，并设为初始值
{
	int i;
	for (i=0;i<a*4;i++){Angel_Zero[i]=0;}
	
	Angle_Read(a);//读取一次角度
	for (i=0;i<a*4;i++)
	{
	  Angel_Zero[i] = Angel_tag[i]; //将该角度附为初始值
	}

  printf("Zero reset!\r\n");//置零完成
	
}
void GetAngle(int n)//读取角度，减掉初始角度，得到相对角度
{
	int tag=0;
  Angle_Read(n);
	
	for(int i=0;i<n*4;i++)
	{
	Angel[i]= Angel_tag[i]- Angel_Zero[i];
	
	if(Angel[i] < -8192)Angel[i]+= 16384;//溢出补偿
	if(Angel[i] > 8192)Angel[i]-= 16384;
	
	if(tag<3)
	{
	printf("%d | %.3f\t    ", tag+1,(float)Angel[i] / 8192 * 180);		
	printf("   ");
	tag++;
	}
	else
	{
	printf("%d | %.3f\t    ", tag+1,(float)Angel[i] / 8192 * 180);		
	printf("\r\n");
	tag=0;
	}
	}
	printf("---------------------------------------------------------------------------------------------");
	printf("\r\n");
	HAL_Delay(20); 
}
