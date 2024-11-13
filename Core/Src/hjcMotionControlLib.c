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

#define PI 3.141592
extern ctrl_channel ctrl_group[24];
extern double form[101][6];
extern float form_planning[100][9];
//extern float form_planning_2[527][13];
extern float form_planning_2[1053][13];
extern float form_planning_writing[826][13];
extern uint8_t pidstart;
extern float form_planning_joint_2_MovOpen_StopPid[76][13];
extern struct joint0;
extern struct joint1;
extern int16_t angle[2][2][4];
extern short angle_ReadIndex;

uint8_t data[8];
//所有肌肉归零
void Set_all_pressure2(int pressure)
{
    __HAL_TIM_SET_COMPARE(ctrl_group[0].htim, ctrl_group[0].tim_channel, pressure);
    __HAL_TIM_SET_COMPARE(ctrl_group[1].htim, ctrl_group[1].tim_channel, pressure);
    __HAL_TIM_SET_COMPARE(ctrl_group[2].htim, ctrl_group[2].tim_channel, pressure);
    __HAL_TIM_SET_COMPARE(ctrl_group[3].htim, ctrl_group[3].tim_channel, pressure);
    __HAL_TIM_SET_COMPARE(ctrl_group[4].htim, ctrl_group[4].tim_channel, pressure);
    __HAL_TIM_SET_COMPARE(ctrl_group[5].htim, ctrl_group[5].tim_channel, pressure);
    __HAL_TIM_SET_COMPARE(ctrl_group[6].htim, ctrl_group[6].tim_channel, pressure);
    __HAL_TIM_SET_COMPARE(ctrl_group[7].htim, ctrl_group[7].tim_channel, pressure);	
	data[0]=0;data[1]=0;data[2]=0;data[3]=0;data[4]=0;data[5]=0;data[6]=0;data[7]=0;//气压数组初始化：[0-1]p0,[2-3]p1,[4-5]p2,[6-7]p3
	joint0.P = pressure;joint0.Q = pressure;
	joint1.P = pressure;joint1.Q = pressure;
	//Angle_Reset();
	osDelay(1500);
}



//开环控制画圆V1.0
//作者：HJC
//2023年11月27
//参数：最大气压max_pressure-kPa,频率hz-赫兹,一圈时间T-秒
void MOVC_open(int max_pressure, int hz, int T)
	{
		max_pressure *=1.1;
		printf("MOVC test START!\r\n");
		int delay_time = 1000/hz;//1000/5==200MS
		double delta_theta = 2*PI/(T*hz);//=2*3.14/5/5=0.25 RAD
		__HAL_TIM_SET_COMPARE(ctrl_group[0].htim, ctrl_group[0].tim_channel, max_pressure);	//0号肌肉先充满,对应cos(0)=1
		for(double theta = 0; theta <=(12*PI); theta+=delta_theta)//在直角坐标系X-Y中画一个半径为max_pressure的圆
		{
			double cos_val = cos(theta);  
			double sin_val = sin(theta);  
			if(cos_val>=0)
				{
				__HAL_TIM_SET_COMPARE(ctrl_group[0].htim, ctrl_group[0].tim_channel, 50 + fabs(max_pressure*cos(theta)));
				printf("output 0: %f\r\n", cos_val); 
				printf("theta 0: %f\r\n", theta); 					
			}
				else 
				{
					__HAL_TIM_SET_COMPARE(ctrl_group[2].htim, ctrl_group[2].tim_channel, 50 + fabs(max_pressure*cos(theta)));
				printf("output 2: %f\r\n", cos_val);  
				}
			if(sin_val>=0)
			{
				__HAL_TIM_SET_COMPARE(ctrl_group[1].htim, ctrl_group[1].tim_channel, 50 + fabs(max_pressure*sin(theta)));
				printf("output 1: %f\r\n", sin_val);  
				printf("theta 0: %f\r\n", theta); 	
			}
				else 
				{
				__HAL_TIM_SET_COMPARE(ctrl_group[3].htim, ctrl_group[3].tim_channel, 50 + fabs(max_pressure*sin(theta)));
				printf("output 4: %f\r\n", sin_val);  
				}
			Read_B();
			osDelay(delay_time);
			//printf("ONE STEP\r\n");
		}		
		printf("MOVC test END!\r\n");
	}

//气压拟合表实验(静态定点测量)
//作者：HJC
//2023年11月20
//参数：气压数组data，循环最大值max_i-kPA,循环最大值max_j-kPA,肌肉号码id1,肌肉号码id2
//说明：
//  调用函数前定义:uint8_t data[8]={0};//气压数组初始化：[0-1]p0,[2-3]p1,[4-5]p2,[6-7]p3
//  否则data内部可能为随机数
void pressure_test(	uint8_t data[8], int max_i, int max_j, char pressure_id_1, char pressure_id_2)
	{	
		for(int i = 0;i < max_i;i += 50)
			{      
			  for(int j = 0;j < max_j;j += 50)
				{       
					data[pressure_id_1*2] = (i >> 8) & 0xFF;
					data[pressure_id_1*2+1] = i  & 0xFF;	  
					data[pressure_id_2*2] = (j >> 8) & 0xFF;
					data[pressure_id_2*2+1] = j  & 0xFF;	  
					send_computer_pressure_value(SEND_PRESSURE_VALUE, 0, data, 4);//发送SEND_PRESSURE_VALUE，肌肉号码ch=0，数据个数=4个=8个16位(注意函数内部*2)
					__HAL_TIM_SET_COMPARE(ctrl_group[pressure_id_1].htim, ctrl_group[pressure_id_1].tim_channel, i);
					__HAL_TIM_SET_COMPARE(ctrl_group[pressure_id_2].htim, ctrl_group[pressure_id_2].tim_channel, j);
					osDelay(150);					
					Read_B();
					osDelay(1050);
				} 
			};
		__HAL_TIM_SET_COMPARE(ctrl_group[pressure_id_1].htim, ctrl_group[pressure_id_1].tim_channel, 0);
		__HAL_TIM_SET_COMPARE(ctrl_group[pressure_id_2].htim, ctrl_group[pressure_id_2].tim_channel, 0);
		data[pressure_id_1*2] = 0;//归零
		data[pressure_id_1*2+1] = 0;
		data[pressure_id_2*2] = 0;
		data[pressure_id_2*2+1] = 0;	
	}

//气压拟合表实验(动态移动测量)
//作者：HJC
//2023年11月20
//参数：最大气压max_pressure-kPa,最小气压min_pressure-kPa,间隔气压step,频率hz-赫兹
//
void pressure_test_trend(int max_pressure, int min_pressure, int step, int hz)
	{	
		uint8_t data[8]={0};//气压数组初始化：[0-1]p0,[2-3]p1,[4-5]p2,[6-7]p3
        //max_pressure *=1.1;min_pressure *=1.1;
		//printf("test START!\r\n");
		int delay_time = 1000/hz;//1000/5==200MS
        Set_all_pressure2(0);//肌肉归零
        osDelay(1500);
        for (int i = max_pressure; i>=min_pressure; i-=step)//开始一圈
        {
			pressure_test_trend_single_step(i, 0, delay_time);
			osDelay(1500);
			for (int j = 0; j <= i; j+=step)//A->..B,P1从小到大
			{
				pressure_test_trend_single_step(j, 1, delay_time);
				//printf("111\r\n");printf("\n%d",j);
			}
			for (int k = (i - step); k>=0; k-=step)//A..->B,P0从大到小
			{
				pressure_test_trend_single_step(k, 0, delay_time);
				//printf("222\r\n");printf("\n%d",k);
			}
			for (int l = 0; l <= i; l+=step)//B..->C,P2从小到大
			{
				pressure_test_trend_single_step(l, 2, delay_time);
				//printf("333\r\n");printf("\n%d",l);
			}
			for (int m = (i - step); m>=0; m-=step)//B->..C,P1从大到小
			{
				pressure_test_trend_single_step(m, 1, delay_time);
				//printf("444\r\n");printf("\n%d",m);
			}
			for (int n = 0; n <= i; n+=step)//C..->D,P3从小到大
			{
				pressure_test_trend_single_step(n, 3, delay_time);
				//printf("555\r\n");printf("\n%d",n);
			}
			for (int o = (i - step); o>=0; o-=step)//C->..D,P2从大到小
			{
				pressure_test_trend_single_step(o, 2, delay_time);
				//printf("666\r\n");printf("\n%d",o);
			}
			
			for (int p = 0; p <= i; p+=step)//D->..A,P0从小到大
			{
				pressure_test_trend_single_step(p, 0, delay_time);
				//printf("777\r\n");printf("\n%d",p);
			}
			for (int q = (i - step); q >= 0; q-=step)//D..->A,P3从大到小
			{
				pressure_test_trend_single_step(q, 3, delay_time);
				//printf("888\r\n");printf("\n%d",q);
			}
        }
	}

void pressure_test_trend_single_step(int pressure, char pressure_id,int delay_time)
	{	
		__HAL_TIM_SET_COMPARE(ctrl_group[pressure_id].htim, ctrl_group[pressure_id].tim_channel, pressure);
		data[pressure_id*2] = (pressure >> 8) & 0xFF;
		data[pressure_id*2+1] = pressure  & 0xFF;
		osDelay(delay_time);
		send_computer_pressure_value(SEND_PRESSURE_VALUE, 0, data, 4);//发送SEND_PRESSURE_VALUE，肌肉号码ch=0，数据个数=4个=8个16位(注意函数内部*2)
		Read_B();
	}
  
void MOV_mf_open(int hz)
	{
		int delay_time = 1000/hz;
		Set_all_pressure2(0);
		for (int i = 1; i<=sizeof(form) / sizeof(form[0][0]); i++)
		{
			__HAL_TIM_SET_COMPARE(ctrl_group[0].htim, ctrl_group[0].tim_channel, form[i][0]);
			__HAL_TIM_SET_COMPARE(ctrl_group[1].htim, ctrl_group[1].tim_channel, form[i][1]);
			__HAL_TIM_SET_COMPARE(ctrl_group[2].htim, ctrl_group[2].tim_channel, form[i][2]);
			__HAL_TIM_SET_COMPARE(ctrl_group[3].htim, ctrl_group[3].tim_channel, form[i][3]);
			osDelay(delay_time);				
			data[0] = ((int)form[i][0] >> 8) & 0xFF;
			data[1] = (int)form[i][0]  & 0xFF;
			data[2] = ((int)form[i][1] >> 8) & 0xFF;
			data[3] = (int)form[i][1]  & 0xFF;
			data[4] = ((int)form[i][2] >> 8) & 0xFF;
			data[5] = (int)form[i][2]  & 0xFF;
			data[6] = ((int)form[i][3] >> 8) & 0xFF;
			data[7] = (int)form[i][3]  & 0xFF;
			//send_computer_pressure_value(SEND_PRESSURE_VALUE, 0, data, 4);//发送SEND_PRESSURE_VALUE，肌肉号码ch=0，数据个数=4个=8个16位(注意函数内部*2)
			//Read_B();
		}
		
	}

void MOV_mf_1ring(int hz)
{
	
	int pid_delay_time = 1000/hz;
	int planning_delay_time = form_planning[1][0] - form_planning[0][0];
	if (pid_delay_time > planning_delay_time)//pid采样时间不能大于规划采样时间
	{
		pid_delay_time  = planning_delay_time;
	}
	//RTOS倒计时任务
	int max = sizeof(form_planning) / sizeof(form_planning[0]);
	printf("moving total %d step",max);
	for (int i = 0; i <= max; i++)
		{
//			int test_a = 0;
			printf("step:%d ",i);
			TickType_t startTick = xTaskGetTickCount(); //  tick count
			while (1) {
				// 执行计算操作
				PID_output_1ring(&pidcontrol0, form_planning[i][5], form_planning, i);
				PID_output_1ring(&pidcontrol1, form_planning[i][6], form_planning, i);
				osDelay(pid_delay_time);
				//printf("t = %f",form_planning[i][0]);
				// 等待规划采样时间的倒计时信号
				if (pidstart == 0){return;}
				if ((xTaskGetTickCount() - startTick) >= pdMS_TO_TICKS(200)) {break;}
			}
		}
	TickType_t remain_Tick = xTaskGetTickCount(); //remain tick count
	printf("remaining");
	while (1)//保持在终点
	{
		PID_output_1ring(&pidcontrol0, form_planning[max][1], form_planning, max);
		PID_output_1ring(&pidcontrol1, form_planning[max][2], form_planning, max);
		osDelay(pid_delay_time);
		// 保持
		if (pidstart ==0){return;}
		if ((xTaskGetTickCount() - remain_Tick) >= pdMS_TO_TICKS(4000)) {break;}
	}
	printf("end");
}
void MOV_mf_1ring_v2(int pid_delay_time)
{
	/*
	注意现在不再以赫兹作为输入，而是毫秒(要是10的整数倍)
		和v1版MOV_mf_1ring不同
		规划的采样频率10ms，但pid的采样频率比规划的大
		同时把pid的补偿调的稍小
		这意味着，按照规划走，偏离了多少，补偿多少
	*/
	float PID_OUTPUT_0 ,PID_OUTPUT_1 ;
	float angle0 ,angle1, pid0_o, pid1_o,ff0,ff1;
	int planning_delay_time = form_planning[1][0] - form_planning[0][0];
	int max = sizeof(form_planning) / sizeof(form_planning[0]);//表格最后一行行数
	printf("moving total %d step, ",max);
	//计算PID轮空回合
	int LK = pid_delay_time / planning_delay_time;
	printf("LK=%d ",LK);
	for (int i = 0; i <= max; i++)
		{
			TickType_t startTick = xTaskGetTickCount(); //  tick count	
			printf("step:%d ",i);
			//非空回合，计算PID值
			if (i % LK == 0 && i > 0){
				Get_error(form_planning,i);
				PID_output_1ring_v2(&pidcontrol0);
				PID_output_1ring_v2(&pidcontrol1);
			}
			//取出前馈值,+pid
			//angle0 = pidcontrol0.realtime_angle;
			//angle1 = pidcontrol1.realtime_angle;
			pid0_o = pidcontrol0.output;
			pid1_o = pidcontrol1.output;
			ff0 = get_FF_value(&pidcontrol0, form_planning, i);
			ff1 = get_FF_value(&pidcontrol1, form_planning, i);
			PID_OUTPUT_0 = get_FF_value(&pidcontrol0, form_planning, i) + pidcontrol0.output;
			PID_OUTPUT_1 = get_FF_value(&pidcontrol1, form_planning, i) + pidcontrol1.output;
			//选择通道并输出
			//printf(" pid0: FF:%f  pid:%f  ",ff0,pid0_o);
			//printf(" pid1: FF:%f  pid:%f  ",ff1,pid1_o);
			//printf(" angle0:%f  pid:%f  ",pidcontrol0.realtime_angle ,pid0_o);
			//printf(" angle1:%f  pid:%f  ",pidcontrol1.realtime_angle ,pid1_o);
			switch_channel_v2_and_output(&pidcontrol0,PID_OUTPUT_0);
			switch_channel_v2_and_output(&pidcontrol1,PID_OUTPUT_1);
			// 等待规划采样时间的倒计时信号
			if (pidstart == 0){return;}
			while ((xTaskGetTickCount() - startTick) <= pdMS_TO_TICKS(planning_delay_time))
			{
				osDelay(1);
			}
		}
	printf("remaining");
	TickType_t remain_Tick_1 = xTaskGetTickCount(); //remain tick count
	while (1)//保持在终点
	{
		TickType_t remain_Tick_2 = xTaskGetTickCount(); //remain tick count
		//非空回合，计算PID值
		Get_error(form_planning,max);
		PID_output_1ring_v2(&pidcontrol0);
		PID_output_1ring_v2(&pidcontrol1);
		//继续结束时的值,+pid
		PID_OUTPUT_0 = get_FF_value(&pidcontrol0, form_planning, max) + pidcontrol0.output;
		PID_OUTPUT_1 = get_FF_value(&pidcontrol1, form_planning, max) + pidcontrol1.output;
		//选择通道并输出
		switch_channel_v2_and_output(&pidcontrol0,PID_OUTPUT_0);
		switch_channel_v2_and_output(&pidcontrol1,PID_OUTPUT_1);
		while ((xTaskGetTickCount() - remain_Tick_2) <= pdMS_TO_TICKS(pid_delay_time))
		{
			osDelay(1);
		}
		// 保持
		if (pidstart ==0){return;}
		if ((xTaskGetTickCount() - remain_Tick_1) >= pdMS_TO_TICKS(5000)) {break;}
	}
	printf("end");
}

void test()
{
			__HAL_TIM_SET_COMPARE(ctrl_group[4].htim, ctrl_group[4].tim_channel, 200);
			osDelay(10000);
			Set_all_pressure2(80);
			Set_all_pressure2(0);
			osDelay(1500);
	
			//joint 1
			__HAL_TIM_SET_COMPARE(ctrl_group[3].htim, ctrl_group[3].tim_channel, 200);
			osDelay(1500);
			__HAL_TIM_SET_COMPARE(ctrl_group[7].htim, ctrl_group[7].tim_channel, 200);
			osDelay(1500);
			Set_all_pressure2(80);
			Set_all_pressure2(0);
			osDelay(1500);
			//...............................
			__HAL_TIM_SET_COMPARE(ctrl_group[0].htim, ctrl_group[0].tim_channel, 200);
			osDelay(1500);
			__HAL_TIM_SET_COMPARE(ctrl_group[4].htim, ctrl_group[4].tim_channel, 200);
			osDelay(1500);
			Set_all_pressure2(80);
			Set_all_pressure2(0);
			osDelay(1500);
			//...............................
			__HAL_TIM_SET_COMPARE(ctrl_group[1].htim, ctrl_group[1].tim_channel, 200);
			osDelay(1500);
			__HAL_TIM_SET_COMPARE(ctrl_group[5].htim, ctrl_group[5].tim_channel, 200);
			osDelay(1500);
			Set_all_pressure2(80);
			Set_all_pressure2(0);
			osDelay(1500);
			//...............................
			__HAL_TIM_SET_COMPARE(ctrl_group[2].htim, ctrl_group[2].tim_channel, 200);
			osDelay(1500);
			__HAL_TIM_SET_COMPARE(ctrl_group[6].htim, ctrl_group[6].tim_channel, 200);
			osDelay(1500);
			Set_all_pressure2(80);
			Set_all_pressure2(0);
			osDelay(1500);
			//...............................

}
		

void joint_2_MOVR()
{
	int max = sizeof(form_planning_2) / sizeof(form_planning_2[0]);//表格最后一行行数
	int planning_delay_time = form_planning_2[1][0] - form_planning_2[0][0];
	printf(" START! total step : %d . ",max);
	for (int i = 0; i <= max; i++)
		{
			printf(" step %d. ",i);
			TickType_t startTick = xTaskGetTickCount(); //  tick count	
			//joint 1
			__HAL_TIM_SET_COMPARE(ctrl_group[0].htim, ctrl_group[0].tim_channel, form_planning_2[i][1]);
			__HAL_TIM_SET_COMPARE(ctrl_group[1].htim, ctrl_group[1].tim_channel, form_planning_2[i][2]);
			__HAL_TIM_SET_COMPARE(ctrl_group[2].htim, ctrl_group[2].tim_channel, form_planning_2[i][3]);
			__HAL_TIM_SET_COMPARE(ctrl_group[3].htim, ctrl_group[3].tim_channel, form_planning_2[i][4]);
			//joint 2
			__HAL_TIM_SET_COMPARE(ctrl_group[4].htim, ctrl_group[4].tim_channel, form_planning_2[i][7]);
			__HAL_TIM_SET_COMPARE(ctrl_group[5].htim, ctrl_group[5].tim_channel, form_planning_2[i][8]);
			__HAL_TIM_SET_COMPARE(ctrl_group[6].htim, ctrl_group[6].tim_channel, form_planning_2[i][9]);
			__HAL_TIM_SET_COMPARE(ctrl_group[7].htim, ctrl_group[7].tim_channel, form_planning_2[i][10]);
			// 等待规划采样时间的倒计时信号
			if (pidstart == 0){return;}
			while ((xTaskGetTickCount() - startTick) <= pdMS_TO_TICKS(planning_delay_time))
			{
				osDelay(1);
			}
		}
	Set_all_pressure2(0);
	printf(" END! ");
}

void joint_2_WRITING()
{
	int max = sizeof(form_planning_writing) / sizeof(form_planning_writing[0]);//表格最后一行行数
	int planning_delay_time = form_planning_writing[1][0] - form_planning_writing[0][0];
	printf(" START! total step : %d . ",max);
	for (int i = 0; i <= max; i++)
		{
			printf(" step %d. ",i);
			TickType_t startTick = xTaskGetTickCount(); //  tick count	
			//joint 1
			__HAL_TIM_SET_COMPARE(ctrl_group[0].htim, ctrl_group[0].tim_channel, form_planning_writing[i][1]);
			__HAL_TIM_SET_COMPARE(ctrl_group[1].htim, ctrl_group[1].tim_channel, form_planning_writing[i][2]);
			__HAL_TIM_SET_COMPARE(ctrl_group[2].htim, ctrl_group[2].tim_channel, form_planning_writing[i][3]);
			__HAL_TIM_SET_COMPARE(ctrl_group[3].htim, ctrl_group[3].tim_channel, form_planning_writing[i][4]);
			//joint 2
			__HAL_TIM_SET_COMPARE(ctrl_group[4].htim, ctrl_group[4].tim_channel, form_planning_writing[i][7]);
			__HAL_TIM_SET_COMPARE(ctrl_group[5].htim, ctrl_group[5].tim_channel, form_planning_writing[i][8]);
			__HAL_TIM_SET_COMPARE(ctrl_group[6].htim, ctrl_group[6].tim_channel, form_planning_writing[i][9]);
			__HAL_TIM_SET_COMPARE(ctrl_group[7].htim, ctrl_group[7].tim_channel, form_planning_writing[i][10]);
			// 等待规划采样时间的倒计时信号
			if (pidstart == 0){return;}
			while ((xTaskGetTickCount() - startTick) <= pdMS_TO_TICKS(planning_delay_time))
			{
				osDelay(1);
			}
		}
	Set_all_pressure2(0);
	printf(" END! ");
}

void joint_2_MovOpen_StopPid(int pid_delay_time)
{
	//说明：双关节
	//使用pid 2.0拓扑版
	//路径运动中开环控制，到点后PID
	//..................................
	//0-1 获取表格最后一行行数
	int max = sizeof(form_planning_joint_2_MovOpen_StopPid) / sizeof(form_planning_joint_2_MovOpen_StopPid[0]) -1;//-1
	//0-2 计算控制间隔时间
	int planning_delay_time = form_planning_joint_2_MovOpen_StopPid[1][0] - form_planning_joint_2_MovOpen_StopPid[0][0];
	//1-1 开始
	printf(" START! total step : %d . ",max);
	//1-2 逐个执行开环表中的数据
	for (int i = 0; i <= max; i++)
		{
			printf(" step %d. ",i);
			TickType_t startTick = xTaskGetTickCount(); //  	tick count		
			//joint 1
			__HAL_TIM_SET_COMPARE(ctrl_group[0].htim, ctrl_group[0].tim_channel, form_planning_joint_2_MovOpen_StopPid[i][1]);
			__HAL_TIM_SET_COMPARE(ctrl_group[1].htim, ctrl_group[1].tim_channel, form_planning_joint_2_MovOpen_StopPid[i][2]);
			__HAL_TIM_SET_COMPARE(ctrl_group[2].htim, ctrl_group[2].tim_channel, form_planning_joint_2_MovOpen_StopPid[i][3]);
			__HAL_TIM_SET_COMPARE(ctrl_group[3].htim, ctrl_group[3].tim_channel, form_planning_joint_2_MovOpen_StopPid[i][4]);
			//joint 2
			__HAL_TIM_SET_COMPARE(ctrl_group[4].htim, ctrl_group[4].tim_channel, form_planning_joint_2_MovOpen_StopPid[i][7]);
			__HAL_TIM_SET_COMPARE(ctrl_group[5].htim, ctrl_group[5].tim_channel, form_planning_joint_2_MovOpen_StopPid[i][8]);
			__HAL_TIM_SET_COMPARE(ctrl_group[6].htim, ctrl_group[6].tim_channel, form_planning_joint_2_MovOpen_StopPid[i][9]);
			__HAL_TIM_SET_COMPARE(ctrl_group[7].htim, ctrl_group[7].tim_channel, form_planning_joint_2_MovOpen_StopPid[i][10]);
			// 等待规划采样时间的倒计时信号
			if (pidstart == 0){return;}
			while ((xTaskGetTickCount() - startTick) <= pdMS_TO_TICKS(planning_delay_time))
			{
				osDelay(1);
			}
		}
	//2-1 到终点，开始闭环控制
	printf(" remaining ");
	//2-2 外部赋值关节的output初始值
	//关节1
	joint0.output0 = form_planning_joint_2_MovOpen_StopPid[max][1];
	joint0.output1 = form_planning_joint_2_MovOpen_StopPid[max][2];
	joint0.output2 = form_planning_joint_2_MovOpen_StopPid[max][3];
	joint0.output3 = form_planning_joint_2_MovOpen_StopPid[max][4];
	//关节2
	joint1.output0 = form_planning_joint_2_MovOpen_StopPid[max][7];
	joint1.output1 = form_planning_joint_2_MovOpen_StopPid[max][8];
	joint1.output2 = form_planning_joint_2_MovOpen_StopPid[max][9];
	joint1.output3 = form_planning_joint_2_MovOpen_StopPid[max][10];
	//定义关节气压输出限制范围
	joint_output_limited_between(80);
	//2-3 每隔一个pid间隔周期pid_delay_time执行一次运算，
	//	  直到保持时间remain_Tick_1 >= 设置值，结束
	TickType_t remain_Tick_1 = xTaskGetTickCount(); //remain tick count
	while (1)
	{
		TickType_t remain_Tick_2 = xTaskGetTickCount(); //pid tick count
		//执行pid
		//每次运行前外部更新4个realtime_angle
			//注意检查关节对应数组的位置
	//		pidcontrol0.realtime_angle = ((float)angle[angle_ReadIndex][0][3]) / 8192 *180;
	//		pidcontrol1.realtime_angle = ((float)angle[angle_ReadIndex][1][3]) / 8192 *180;
	//		pidcontrol2.realtime_angle = ((float)angle[angle_ReadIndex][0][2]) / 8192 *180;
	//		pidcontrol3.realtime_angle = ((float)angle[angle_ReadIndex][1][2]) / 8192 *180;
		pidcontrol0.realtime_angle = ((float)angle[angle_ReadIndex][0][2]) / 8192 *180;
		pidcontrol1.realtime_angle = ((float)angle[angle_ReadIndex][1][2]) / 8192 *180;
		pidcontrol2.realtime_angle = ((float)angle[angle_ReadIndex][0][3]) / 8192 *180;
		pidcontrol3.realtime_angle = ((float)angle[angle_ReadIndex][1][3]) / 8192 *180;
		pid_output_v2(&pidcontrol0,&pidcontrol1,
					  &pidcontrol2,&pidcontrol3,
					  form_planning_joint_2_MovOpen_StopPid[max][5],
					  form_planning_joint_2_MovOpen_StopPid[max][6],
					  form_planning_joint_2_MovOpen_StopPid[max][11],
					  form_planning_joint_2_MovOpen_StopPid[max][12]);
		//pid的间隔延时
		while ((xTaskGetTickCount() - remain_Tick_2) <= pdMS_TO_TICKS(pid_delay_time))
		{
			osDelay(1);
		}
		//总的保持时间，时间到则停止
		if (pidstart ==0){return;}
		if ((xTaskGetTickCount() - remain_Tick_1) >= pdMS_TO_TICKS(5000)) {break;}
	}
	Set_all_pressure2(0);
	PID_INIT(2);
	printf(" END! ");
}

void joint_2_MOV_CLOSE_test()
{
	//说明：双关节轨迹闭环跟踪
	//使用pid 2.0拓扑版
	//慢速运行，路径离散点逐个用PD跟踪
	//..................................
	//0-1 获取表格最后一行行数
	int max = sizeof(form_planning_joint_2_MovOpen_StopPid) / sizeof(form_planning_joint_2_MovOpen_StopPid[0]) -1;//-1
	//0-2 计算控制间隔时间
	int planning_delay_time = form_planning_joint_2_MovOpen_StopPid[1][0] - form_planning_joint_2_MovOpen_StopPid[0][0];
	//1-1 开始
	printf(" START! total step : %d . ",max);
	//1-2 外部赋值关节的output初始值
	//关节1
	joint0.P = 0;
	joint0.Q = 0;
	joint0.output0 = 0;
	joint0.output1 = 0;
	joint0.output2 = 0;
	joint0.output3 = 0;
	//关节2
	joint1.P = 0;
	joint1.Q = 0;
	joint1.output0 = 0;
	joint1.output1 = 0;
	joint1.output2 = 0;
	joint1.output3 = 0;
	//1-3 定义关节气压输出限制范围
	joint_output_limited_between(80);
	//2 开始PID
	for (int i = 0; i <= max; i++)
		{
			printf(" step %d. ",i);
			TickType_t startTick = xTaskGetTickCount(); //  	tick count		
		//执行pid
		//每次运行前外部更新4个realtime_angle
			//注意检查关节对应数组的位置
	//		pidcontrol0.realtime_angle = ((float)angle[angle_ReadIndex][0][3]) / 8192 *180;
	//		pidcontrol1.realtime_angle = ((float)angle[angle_ReadIndex][1][3]) / 8192 *180;
	//		pidcontrol2.realtime_angle = ((float)angle[angle_ReadIndex][0][2]) / 8192 *180;
	//		pidcontrol3.realtime_angle = ((float)angle[angle_ReadIndex][1][2]) / 8192 *180;
			pidcontrol0.realtime_angle = ((float)angle[angle_ReadIndex][0][2]) / 8192 *180;
			pidcontrol1.realtime_angle = ((float)angle[angle_ReadIndex][1][2]) / 8192 *180;
			pidcontrol2.realtime_angle = ((float)angle[angle_ReadIndex][0][3]) / 8192 *180;
			pidcontrol3.realtime_angle = ((float)angle[angle_ReadIndex][1][3]) / 8192 *180;
			pid_output_v2(&pidcontrol0,&pidcontrol1,
						  &pidcontrol2,&pidcontrol3,
						  form_planning_joint_2_MovOpen_StopPid[i][5],
						  form_planning_joint_2_MovOpen_StopPid[i][6],
						  form_planning_joint_2_MovOpen_StopPid[i][11],
						  form_planning_joint_2_MovOpen_StopPid[i][12]);
			//pid的间隔延时
			if (pidstart == 0){return;}
			while ((xTaskGetTickCount() - startTick) <= pdMS_TO_TICKS(planning_delay_time))
			{
				osDelay(1);
			}
			if (pidstart ==0){return;}
		}
	//3 结束复位
	Set_all_pressure2(0);
	PID_INIT(2);
	printf(" END! ");
}