#ifndef __myPID_H
#define __myPID_H
#include "stdio.h"	
#include "main.h"

//PID结构体
typedef struct{
    uint8_t id;//序号
    float feed_forward_value;//前馈值

    float realtime_angle;float realtime_speed;float realtime_pressure;//实时值：角度速度压力
    float target_angle; float target_speed;float target_pressure;//目标值：角度速度压力
    //error-----------------------
    float error_angle;   float error_sum_angle;   float error_difference_angle;   float error_last_angle;//误差 积分 差分 上一次误差
    float error_speed;   float error_sum_speed;   float error_difference_speed;   float error_last_speed;
    float error_pressure;float error_sum_pressure;float error_difference_pressure;float error_last_pressure;
    float error_sum_angle_max;float error_sum_speed_max;float error_sum_pressure_max;//积分最大值
    //Kp Ki Kd---------------------
    float Kp_angle;float Ki_angle;float Kd_angle;
    float Kp_speed;float Ki_speed;float Kd_speed;
    float Kp_pressure;float Ki_pressure;float Kd_pressure;
    //output-----------------------
    int output_max;int output_min;//输出最大最小值
    float output; //输出    
    short output_channel;    
}PID_TypeDef;

//关节输出结构体
typedef struct{
    int16_t 
	P , Q,
	output0     , output1     , output2     , output3,
	output0_min , output1_min , output2_min , output3_min,
	output0_max , output1_max , output2_max , output3_max;
}joint_TypeDef;

extern joint_TypeDef joint0;
extern joint_TypeDef joint1;

extern PID_TypeDef pidcontrol0;//第一关节1,3肌肉
extern PID_TypeDef pidcontrol1;//第一关节2,4肌肉
extern PID_TypeDef pidcontrol2;


extern PID_TypeDef pidcontrol3;
extern PID_TypeDef pidcontrol4;
extern PID_TypeDef pidcontrol5;
extern PID_TypeDef pidcontrol6;
extern PID_TypeDef pidcontrol7;
//初始化
void PID_INIT(int arm_num);
void PID_INIT_single_init(PID_TypeDef* pid_InitStruct);
void PID_Reset(PID_TypeDef* pid_InitStruct);
//计算函数
float get_FF_value(PID_TypeDef* pid_Struct, float form_planning[102][9], int form_id);
void PID_output_1ring_v2(PID_TypeDef* pid_Struct);
void switch_channel_v2_and_output(PID_TypeDef* pid_Struct, float pressure_PID_value);
void PID_output_1ring(PID_TypeDef* pid_Struct, float target_angle, float form_planning[102][9], int form_id);
void PID_output_2ring(PID_TypeDef* pid_Struct, float target_angle, float form_planning[102][9], int form_id);
void PID_output_3ring(PID_TypeDef* pid_Struct, float target_angle, float form_planning[102][9], int form_id);
void Get_error(float form_planning[100][9], int form_id);
float Get_Value(uint8_t pidnum);
float pid_amplitude_limiting(float PID_value,float limit);//限幅
float angle_PID_value(PID_TypeDef* pid_Struct);
float speed_PID_value(PID_TypeDef* pid_Struct, float angle_PID_value);
float pressure_PID_value(PID_TypeDef* pid_Struct, float speed_PID_value);
void switch_channel(PID_TypeDef* pid_Struct, float pressure_PID_value);
void pid_output_v2(PID_TypeDef* pid_Struct0, PID_TypeDef* pid_Struct1,
                   PID_TypeDef* pid_Struct2, PID_TypeDef* pid_Struct3, 
                   float target_angle0, float target_angle1, float target_angle2, float target_angle3);
void pid_iteration(PID_TypeDef* pid_Struct);
void air_output_j1(float value0,float value1,float value2,float value3);
void air_output_j2(float value0,float value1,float value2,float value3);
void joint_output_limited_between(int value);
void joint_OUTPUT_LIMIT();
float get_theta(float x,float y);
#endif
