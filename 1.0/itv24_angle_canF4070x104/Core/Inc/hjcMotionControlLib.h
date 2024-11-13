#ifndef __hjcMotionControlLib_H
#define __hjcMotionControlLib_H
#include "stdio.h"	
#include "main.h"
#include "can.h"
#include "myPID.h"

void Set_all_pressure2(int pressure);
void Angle_Reset(void);//锟斤拷取一锟轿角度ｏ拷锟斤拷锟斤拷为锟斤拷始值
void MOVC_open(int max_pressure, int hz, int T);
void pressure_test(	uint8_t data[8], int max_i, int max_j, char pressure_id_1, char pressure_id_2);
void pressure_test_trend(int max_pressure, int min_pressure, int step, int hz);
void pressure_test_trend_single_step(int pressure, char pressure_id,int delay_time);
void MOV_mf_open(int hz);
void MOV_mf_1ring(int hz);
void MOV_mf_1ring_v2(int hz);
void test(void);
void joint_2_MOVR(void);
void joint_2_WRITING(void);
void joint_2_MovOpen_StopPid(int pid_delay_time);
void joint_2_MOV_CLOSE_test(void);
#endif

