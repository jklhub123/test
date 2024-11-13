#include "main.h"
#include "hjcMotionControlLib.h"
#include "hjcForm.h"
#include "myPID.h"
#include "stdio.h"
#include "can.h"
#include "usart.h"
#include <stdlib.h>
#include "math.h"
#include "protocol.h"
#include "spi.h"
#include "stm32f4xx_it.h"
#include "stdio.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define PI 3.141592f
extern ctrl_channel ctrl_group[24];

extern int16_t angle[2][2][4];
extern short angle_ReadIndex;
////关节指令
//extern int16_t GLOBAL_ANGLE[2][3][4];
//extern short GLOBAL_ANGLE_ReadIndex;
////开环气压指令
//extern short GLOBAL_PRESSURE_FLAG;
//extern int16_t GLOBAL_PRESSURE[2][4][4];
//extern short GLOBAL_PRESSURE_ReadIndex;
//其他



void MOVJ_v1_0(int16_t ANGLE[2][3][4], short ReadIndex)
{
	//作者:HJC
	//日期：2024-7-10
	//说明：四关节臂关节运动指令（正运动学）
	//使用pid 2.0拓扑版
	//..................................
	SendAngle_CAN(ANGLE[ReadIndex][2][0],
				  ANGLE[ReadIndex][0][0],
				  ANGLE[ReadIndex][0][1],
				  ANGLE[ReadIndex][0][2],
				  ANGLE[ReadIndex][0][3],
				  2);
	SendAngle_CAN(ANGLE[ReadIndex][2][1], 
				  ANGLE[ReadIndex][1][0],
				  ANGLE[ReadIndex][1][1],
				  ANGLE[ReadIndex][1][2],
				  ANGLE[ReadIndex][1][3],
				  3);
	SendShouldAngle(0x141 , ANGLE[ReadIndex][2][0] , 120);
	SendShouldAngle(0x142 , ANGLE[ReadIndex][2][1] , 120);
	printf("SEND MOVJ\r\n");
	printf("ANGLE1 = %.3f , %.3f , %.3f , %.3f\r\n",
			(float)ANGLE[ReadIndex][0][0] / 8192 *180,
			(float)ANGLE[ReadIndex][0][1] / 8192 *180,
			(float)ANGLE[ReadIndex][0][2] / 8192 *180,
			(float)ANGLE[ReadIndex][0][3] / 8192 *180);
	printf("ANGLE2 = %.3f , %.3f , %.3f , %.3f\r\n",
			(float)ANGLE[ReadIndex][1][0] / 8192 *180,
			(float)ANGLE[ReadIndex][1][1] / 8192 *180,
			(float)ANGLE[ReadIndex][1][2] / 8192 *180,
			(float)ANGLE[ReadIndex][1][3] / 8192 *180);
	printf("MOTOR ANGLE = %.3f , %.3f\r\n",
			(float)ANGLE[ReadIndex][2][0] / 8192 *180,
			(float)ANGLE[ReadIndex][2][1] / 8192 *180);
	printf("-------------\r\n");
}


void MOVP_v1_0(int16_t pressure[2][5][4], short ReadIndex)
{
	//作者:HJC
	//日期：2024-7-11
	//说明：四关节臂关节运动指令（正运动学）
	//使用pid 2.0拓扑版
	//..................................
	Send_MOVP(0 , pressure[ReadIndex][0]);//id = 0~3为左臂
	Send_MOVP(1 , pressure[ReadIndex][1]);
	Send_MOVP(2 , pressure[ReadIndex][2]);
	Send_MOVP(3 , pressure[ReadIndex][3]);
	SendShouldAngle(0x141 , pressure[ReadIndex][4][0] , 120);
	SendShouldAngle(0x142 , pressure[ReadIndex][4][1] , 120);
	printf("SEND MOVP\r\n");
	printf("joint 0 pressure = %d , %d , %d , %d\r\n",
			pressure[ReadIndex][0][0],
			pressure[ReadIndex][0][1],
			pressure[ReadIndex][0][2],
			pressure[ReadIndex][0][3]);
	printf("joint 1 pressure = %d , %d , %d , %d\r\n",
			pressure[ReadIndex][1][0],
			pressure[ReadIndex][1][1],
			pressure[ReadIndex][1][2],
			pressure[ReadIndex][1][3]);
	printf("joint 2 pressure = %d , %d , %d , %d\r\n",
			pressure[ReadIndex][2][0],
			pressure[ReadIndex][2][1],
			pressure[ReadIndex][2][2],
			pressure[ReadIndex][2][3]);
	printf("joint 3 pressure = %d , %d , %d , %d\r\n",
			pressure[ReadIndex][3][0],
			pressure[ReadIndex][3][1],
			pressure[ReadIndex][3][2],
			pressure[ReadIndex][3][3]);
	printf("MOTOR ANGLE = %.3f , %.3f\r\n",
			(float)pressure[ReadIndex][4][0] / 8192 *180,
			(float)pressure[ReadIndex][4][1] / 8192 *180);
	printf("-------------\r\n");
}
void MOVP_v1_0_right_arm(int16_t pressure[2][5][4], short ReadIndex)
{
	//作者:HJC
	//日期：2024-7-12
	//内测日期：
	//说明：四关节臂关节运动指令（正运动学）
	//使用pid 2.0拓扑版
	//..................................
	Send_MOVP(4 , pressure[ReadIndex][0]);//id = 0~3为左臂
	Send_MOVP(5 , pressure[ReadIndex][1]);
	Send_MOVP(6 , pressure[ReadIndex][2]);
	osDelay(1);
	Send_MOVP(7 , pressure[ReadIndex][3]);
	SendShouldAngle(0x143 , pressure[ReadIndex][4][0] , 120);
	SendShouldAngle(0x144 , pressure[ReadIndex][4][1] , 120);
	printf("SEND MOVP\r\n");
	printf("Rjoint 0 pressure = %d , %d , %d , %d\r\n",
			pressure[ReadIndex][0][0],
			pressure[ReadIndex][0][1],
			pressure[ReadIndex][0][2],
			pressure[ReadIndex][0][3]);
	printf("Rjoint 1 pressure = %d , %d , %d , %d\r\n",
			pressure[ReadIndex][1][0],
			pressure[ReadIndex][1][1],
			pressure[ReadIndex][1][2],
			pressure[ReadIndex][1][3]);
	printf("Rjoint 2 pressure = %d , %d , %d , %d\r\n",
			pressure[ReadIndex][2][0],
			pressure[ReadIndex][2][1],
			pressure[ReadIndex][2][2],
			pressure[ReadIndex][2][3]);
	printf("Rjoint 3 pressure = %d , %d , %d , %d\r\n",
			pressure[ReadIndex][3][0],
			pressure[ReadIndex][3][1],
			pressure[ReadIndex][3][2],
			pressure[ReadIndex][3][3]);
	printf("RMOTOR ANGLE = %.3f , %.3f\r\n",
			(float)pressure[ReadIndex][4][0] / 8192 *180,
			(float)pressure[ReadIndex][4][1] / 8192 *180);
	printf("-------------\r\n");
}

void DEEP_MOTOR_TEST_v1_0(uint16_t MOTOR_MAT[], short ReadIndex, short id)
{
	//作者:HJC
	//日期：2024-7-16
	//说明：云深处电机测试
	/*
	           位数        名称               取值范围
  	CAN DATA BIT0~15     目标角度     [-40rad,40rad]     -> [0,65535]     FFFF
	         BIT16~29    目标角速度   [-40rad/s,40rad/s] -> [0,16383]     3FFF
	         BIT30~39    KP刚度      [0,1023]           -> [0,1023]      3FF
	         BIT40~47    KD阻尼      [0.0,51.0]         -> [0,255]       FF
	         BIT48~63    目标扭矩    [-40Nm,40Nm]       -> [0,65535]     FFFF
	*/
	//..................................
	Send_DEEP_Motor(id, MOTOR_MAT);
	float angle =  ((float)MOTOR_MAT[0] - 0) / 65535.0 * 80 - 40;
	float speed =  ((float)MOTOR_MAT[1] - 0) / 16383.0 * 80 - 40;
	float kp =      (float)MOTOR_MAT[2];
	float kd =     ((float)MOTOR_MAT[3] - 0) / 255.0 * 51.0;
	float torque = ((float)MOTOR_MAT[4] - 0) / 65535.0 * 80 - 40;
	
	printf("SEND DEEP_MOTOR\r\n");
	printf("ORINGIN : id = %d , angle = %4x , speed = %4x , kp = %4x , kd = %4x , torque = %4x\r\n",
			MOTOR_MAT[5],
			MOTOR_MAT[0],
			MOTOR_MAT[1],
			MOTOR_MAT[2],
			MOTOR_MAT[3],
			MOTOR_MAT[4]);
	printf("id = %d , angle = %.1f , speed = %.1f , kp = %.1f , kd = %.1f , torque = %.1f\r\n",
			id,
			angle,
			speed,
			kp,
			kd,
			torque);
	printf("-------------------------\r\n");
}

void BRMXZ_action_1()
{
	//作者:HJC
	//日期：2024-9-22
	//说明：BRMXZ半人马行者 动作1 拥抱
	//注意：手臂的肌肉1-1和2-1为同一个比例阀控制，并非独立控制
	//..................................
	// 0
	int16_t LeftArm_chain_1_mat_0[4]  = {0};
	int16_t LeftArm_chain_2_mat_0[4]  = {0};
	int16_t RightArm_chain_1_mat_0[4] = {0};
	int16_t RightArm_chain_2_mat_0[4] = {0};
	uint16_t DEEP_MOTOR_1_mat_0[5]    = {0};
	uint16_t DEEP_MOTOR_2_mat_0[5]    = {0};
	uint16_t DEEP_MOTOR_3_mat_0[5]    = {0};
	uint16_t DEEP_MOTOR_4_mat_0[5]    = {0};
	//------------------------------------------
	// 1
	int16_t LeftArm_chain_1_mat_1[4]  = {0};
	int16_t LeftArm_chain_2_mat_1[4]  = {0};
	int16_t RightArm_chain_1_mat_1[4] = {0};
	int16_t RightArm_chain_2_mat_1[4] = {0};
	uint16_t DEEP_MOTOR_1_mat_1[5]    = {0};
	uint16_t DEEP_MOTOR_2_mat_1[5]    = {0};
	uint16_t DEEP_MOTOR_3_mat_1[5]    = {0};
	uint16_t DEEP_MOTOR_4_mat_1[5]    = {0};
	//------------------------------------------
	// 2
	int16_t LeftArm_chain_1_mat_2[4]  = {0};
	int16_t LeftArm_chain_2_mat_2[4]  = {0};
	int16_t RightArm_chain_1_mat_2[4] = {0};
	int16_t RightArm_chain_2_mat_2[4] = {0};
	uint16_t DEEP_MOTOR_1_mat_2[5]    = {0};
	uint16_t DEEP_MOTOR_2_mat_2[5]    = {0};
	uint16_t DEEP_MOTOR_3_mat_2[5]    = {0};
	uint16_t DEEP_MOTOR_4_mat_2[5]    = {0};
	//------------------------------------------
	// 3
	int16_t LeftArm_chain_1_mat_3[4]  = {0};
	int16_t LeftArm_chain_2_mat_3[4]  = {0};
	int16_t RightArm_chain_1_mat_3[4] = {0};
	int16_t RightArm_chain_2_mat_3[4] = {0};
	uint16_t DEEP_MOTOR_1_mat_3[5]    = {0};
	uint16_t DEEP_MOTOR_2_mat_3[5]    = {0};
	uint16_t DEEP_MOTOR_3_mat_3[5]    = {0};
	uint16_t DEEP_MOTOR_4_mat_3[5]    = {0};
	//------------------------------------------
	// 4
	int16_t LeftArm_chain_1_mat_4[4]  = {0};
	int16_t LeftArm_chain_2_mat_4[4]  = {0};
	int16_t RightArm_chain_1_mat_4[4] = {0};
	int16_t RightArm_chain_2_mat_4[4] = {0};
	uint16_t DEEP_MOTOR_1_mat_4[5]    = {0};
	uint16_t DEEP_MOTOR_2_mat_4[5]    = {0};
	uint16_t DEEP_MOTOR_3_mat_4[5]    = {0};
	uint16_t DEEP_MOTOR_4_mat_4[5]    = {0};
	//------------------------------------------
	// 5
	int16_t LeftArm_chain_1_mat_5[4]  = {0};
	int16_t LeftArm_chain_2_mat_5[4]  = {0};
	int16_t RightArm_chain_1_mat_5[4] = {0};
	int16_t RightArm_chain_2_mat_5[4] = {0};
	uint16_t DEEP_MOTOR_1_mat_5[5]    = {0};
	uint16_t DEEP_MOTOR_2_mat_5[5]    = {0};
	uint16_t DEEP_MOTOR_3_mat_5[5]    = {0};
	uint16_t DEEP_MOTOR_4_mat_5[5]    = {0};
	//------------------------------------------
	//使能电机
	turn_on_DEEP_Motor(1);
	turn_on_DEEP_Motor(2);
	turn_on_DEEP_Motor(3);
	turn_on_DEEP_Motor(4);
	//点位1 张开--------------------------------------------
		Send_MOVP(0 , LeftArm_chain_1_mat_1);//id = 0~3为左臂
		Send_MOVP(1 , LeftArm_chain_2_mat_1);
		Send_MOVP(2 , RightArm_chain_1_mat_1);
		Send_MOVP(3 , RightArm_chain_2_mat_1);
		//-----------------
		Send_DEEP_Motor(1, DEEP_MOTOR_1_mat_1);
		Send_DEEP_Motor(2, DEEP_MOTOR_2_mat_1);
		Send_DEEP_Motor(3, DEEP_MOTOR_3_mat_1);
		Send_DEEP_Motor(4, DEEP_MOTOR_4_mat_1);	
		//-------------
		osDelay(1000);
	//点位2 拥抱--------------------------------------------
		Send_MOVP(0 , LeftArm_chain_1_mat_2);//id = 0~3为左臂
		Send_MOVP(1 , LeftArm_chain_2_mat_2);
		Send_MOVP(2 , RightArm_chain_1_mat_2);
		Send_MOVP(3 , RightArm_chain_2_mat_2);
		//-----------------
		Send_DEEP_Motor(1, DEEP_MOTOR_1_mat_2);
		Send_DEEP_Motor(2, DEEP_MOTOR_2_mat_2);
		Send_DEEP_Motor(3, DEEP_MOTOR_3_mat_2);
		Send_DEEP_Motor(4, DEEP_MOTOR_4_mat_2);
		//-------------
		osDelay(1000);
	//点位3 抚摸 1 -----------------------------------------
		Send_MOVP(0 , LeftArm_chain_1_mat_3);//id = 0~3为左臂
		Send_MOVP(1 , LeftArm_chain_2_mat_3);
		Send_MOVP(2 , RightArm_chain_1_mat_3);
		Send_MOVP(3 , RightArm_chain_2_mat_3);
		//-----------------
		Send_DEEP_Motor(1, DEEP_MOTOR_1_mat_3);
		Send_DEEP_Motor(2, DEEP_MOTOR_2_mat_3);
		Send_DEEP_Motor(3, DEEP_MOTOR_3_mat_3);
		Send_DEEP_Motor(4, DEEP_MOTOR_4_mat_3);
		//-------------
		osDelay(1000);	   
	//点位4 抚摸 2 -----------------------------------------
		Send_MOVP(0 , LeftArm_chain_1_mat_4);//id = 0~3为左臂
		Send_MOVP(1 , LeftArm_chain_2_mat_4);
		Send_MOVP(2 , RightArm_chain_1_mat_4);
		Send_MOVP(3 , RightArm_chain_2_mat_4);
		//-----------------
		Send_DEEP_Motor(1, DEEP_MOTOR_1_mat_4);
		Send_DEEP_Motor(2, DEEP_MOTOR_2_mat_4);
		Send_DEEP_Motor(3, DEEP_MOTOR_3_mat_4);
		Send_DEEP_Motor(4, DEEP_MOTOR_4_mat_4);
		//-------------
		osDelay(1000);
	//点位3 - 4循环  -----------------------------------------	
					   
	//点位5 放开
		Send_MOVP(0 , LeftArm_chain_1_mat_5);//id = 0~3为左臂
		Send_MOVP(1 , LeftArm_chain_2_mat_5);
		Send_MOVP(2 , RightArm_chain_1_mat_5);
		Send_MOVP(3 , RightArm_chain_2_mat_5);
		//-----------------
		Send_DEEP_Motor(1, DEEP_MOTOR_1_mat_5);
		Send_DEEP_Motor(2, DEEP_MOTOR_2_mat_5);
		Send_DEEP_Motor(3, DEEP_MOTOR_3_mat_5);
		Send_DEEP_Motor(4, DEEP_MOTOR_4_mat_5);
		//-------------
		osDelay(1000);
	//点位6 归零
		Send_MOVP(0 , LeftArm_chain_1_mat_0);//id = 0~3为左臂
		Send_MOVP(1 , LeftArm_chain_2_mat_0);
		Send_MOVP(2 , RightArm_chain_1_mat_0);
		Send_MOVP(3 , RightArm_chain_2_mat_0);
		//-----------------
		Send_DEEP_Motor(1, DEEP_MOTOR_1_mat_0);
		Send_DEEP_Motor(2, DEEP_MOTOR_2_mat_0);
		Send_DEEP_Motor(3, DEEP_MOTOR_3_mat_0);
		Send_DEEP_Motor(4, DEEP_MOTOR_4_mat_0);
	//失能放在外部发命令控制
					   
					   
					 
}