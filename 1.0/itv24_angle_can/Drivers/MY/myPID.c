#include "main.h"
#include "myPID.h"
#include "stdio.h"
#include "can.h"
#include "usart.h"
#include <stdlib.h>
#include "math.h"
#include "spi.h"

extern uint16_t angle[2][4];
extern float form_planning[100][9];

//float watch_angle_0, watch_angle_1, watch_angle_f0, watch_angle_f1;


PID_TypeDef pidcontrol0;
PID_TypeDef pidcontrol1;
PID_TypeDef pidcontrol2;
PID_TypeDef pidcontrol3;
PID_TypeDef pidcontrol4;
PID_TypeDef pidcontrol5;
PID_TypeDef pidcontrol6;
PID_TypeDef pidcontrol7;
joint_TypeDef joint0;
joint_TypeDef joint1;

void PID_INIT(int arm_num){
  pidcontrol0.id=0;
  pidcontrol1.id=1;
  pidcontrol2.id=2;
  pidcontrol3.id=3;
  pidcontrol4.id=4;
  pidcontrol5.id=5;
  pidcontrol6.id=6;
  pidcontrol7.id=7;
  switch (arm_num)
  {
  case 1:
    PID_INIT_single_init(&pidcontrol0);
    PID_INIT_single_init(&pidcontrol1);
    break;
  case 2:
    PID_INIT_single_init(&pidcontrol0);
    PID_INIT_single_init(&pidcontrol1);
    PID_INIT_single_init(&pidcontrol2);
    PID_INIT_single_init(&pidcontrol3);
    break;
  case 3:
    PID_INIT_single_init(&pidcontrol0);
    PID_INIT_single_init(&pidcontrol1);
    PID_INIT_single_init(&pidcontrol2);
    PID_INIT_single_init(&pidcontrol3);
    PID_INIT_single_init(&pidcontrol4);
    PID_INIT_single_init(&pidcontrol5);
    break;
  case 4:
    PID_INIT_single_init(&pidcontrol0);
    PID_INIT_single_init(&pidcontrol1);
    PID_INIT_single_init(&pidcontrol2);
    PID_INIT_single_init(&pidcontrol3);
    PID_INIT_single_init(&pidcontrol4);
    PID_INIT_single_init(&pidcontrol5);
    PID_INIT_single_init(&pidcontrol6);
    PID_INIT_single_init(&pidcontrol7);
    break;
  default:
    break;
  }
}
void PID_INIT_single_init(PID_TypeDef* pid_InitStruct){
    //RT status---------------------
    pid_InitStruct->realtime_angle=0;pid_InitStruct->realtime_speed=0;pid_InitStruct->realtime_pressure=0;
    //target------------------------
    pid_InitStruct->target_angle=0;pid_InitStruct->target_speed=0;pid_InitStruct->target_pressure=0;
    //error-------------------------
    pid_InitStruct->error_angle=0;              pid_InitStruct->error_sum_angle=0;     
    pid_InitStruct->error_difference_angle=0;   pid_InitStruct->error_last_angle=0;
    pid_InitStruct->error_speed=0;              pid_InitStruct->error_sum_speed=0;     
    pid_InitStruct->error_difference_speed=0;   pid_InitStruct->error_last_speed=0;
    pid_InitStruct->error_pressure=0;           pid_InitStruct->error_sum_pressure=0;     
    pid_InitStruct->error_difference_pressure=0;pid_InitStruct->error_last_pressure=0;   
    //error_sum_max积分限幅
    pid_InitStruct->error_sum_angle_max = 200;
    pid_InitStruct->error_sum_speed_max = 100;
    pid_InitStruct->error_sum_pressure_max = 100;
    //Kp Ki Kd---------------------
    pid_InitStruct->Kp_angle=3;pid_InitStruct->Ki_angle=0;pid_InitStruct->Kd_angle=0;
    pid_InitStruct->Kp_speed=0.5;pid_InitStruct->Ki_speed=0.2;pid_InitStruct->Kd_speed=0.1;
    pid_InitStruct->Kp_pressure=0.5;pid_InitStruct->Ki_pressure=0.3;pid_InitStruct->Kd_pressure=0.5;
    //output-----------------------
	pid_InitStruct->output_max=250;pid_InitStruct->output_min=-250;
    pid_InitStruct->output=0;
    pid_InitStruct->output_channel=0;
}
void PID_Reset(PID_TypeDef* pid_InitStruct){
    PID_INIT_single_init(&pidcontrol0);
	  PID_INIT_single_init(&pidcontrol1);
    PID_INIT_single_init(&pidcontrol2);
	  PID_INIT_single_init(&pidcontrol3);
    PID_INIT_single_init(&pidcontrol4);
	  PID_INIT_single_init(&pidcontrol5);
    PID_INIT_single_init(&pidcontrol6);
	  PID_INIT_single_init(&pidcontrol7);
}

float get_FF_value(PID_TypeDef* pid_Struct, float form_planning[101][9], int form_id)
{
  float FF_value;
  switch (pid_Struct->id %2)//取id数/2的余数
  {
  case 0://id=0246,取绝对值大的数值
    //识别出P0为输出
    if (fabs(form_planning[form_id][1]) > fabs(form_planning[form_id][3])){
      FF_value = form_planning[form_id][1];
      pid_Struct->output_channel = 0;
      }
    //识别出P2为输出
    else {
      FF_value = form_planning[form_id][3];
      pid_Struct->output_channel = 2;
      }
    break;
  case 1:
    //识别出P1为输出
    if (fabs(form_planning[form_id][2]) > fabs(form_planning[form_id][4])){
      FF_value = form_planning[form_id][2];
      pid_Struct->output_channel = 1;
      }
    //识别出P3为输出
    else {
      FF_value = form_planning[form_id][4];
      pid_Struct->output_channel = 3;
      }
    break;
  }
  return FF_value;
}
void PID_output_1ring_v2(PID_TypeDef* pid_Struct)
{   
  /*
  V2版，只计算PID，前馈放在外部加
  */
	pid_Struct->output = pid_amplitude_limiting(angle_PID_value(pid_Struct),80);//前馈值+限幅后的PID输出
}
void switch_channel_v2_and_output(PID_TypeDef* pid_Struct, float pressure_PID_value)
{
	int temp_pressure;
	short temp_ch;
	//test
	if (pid_Struct -> output_channel == 2){
	printf("   CH=2!!   ");
	}
  if(pid_Struct->output >= 0)//输出端口不变
  {
	  temp_pressure = pressure_PID_value;
    __HAL_TIM_SET_COMPARE(ctrl_group[pid_Struct -> output_channel].htim, ctrl_group[pid_Struct -> output_channel].tim_channel, temp_pressure);
	//printf("out:pid%d = %d   ",pid_Struct -> output_channel, temp_pressure);
  }
  if(pressure_PID_value < 0)//输出道取反
  {
	//printf("         swtich ch         ");
    pressure_PID_value = fabs(pressure_PID_value);//取绝对值
	temp_pressure = pressure_PID_value;
    switch (pid_Struct -> output_channel)
    {
    case 0:
      temp_ch = 2;
	  //printf("out:pid%d = %d   ",temp_ch, temp_pressure);
      __HAL_TIM_SET_COMPARE(ctrl_group[0].htim, ctrl_group[0].tim_channel, 0);//原通道停止
      __HAL_TIM_SET_COMPARE(ctrl_group[temp_ch].htim, ctrl_group[temp_ch].tim_channel, temp_pressure);
      break;
    case 1:
      temp_ch = 3;
	 //printf("out:pid%d = %d   ",temp_ch, temp_pressure);
      __HAL_TIM_SET_COMPARE(ctrl_group[1].htim, ctrl_group[1].tim_channel, 0);
      __HAL_TIM_SET_COMPARE(ctrl_group[temp_ch].htim, ctrl_group[temp_ch].tim_channel, temp_pressure);
      break;
    case 2:
      temp_ch = 0;
	  //printf("out:pid%d = %d   ",temp_ch, temp_pressure);
      __HAL_TIM_SET_COMPARE(ctrl_group[2].htim, ctrl_group[2].tim_channel, 0);
      __HAL_TIM_SET_COMPARE(ctrl_group[temp_ch].htim, ctrl_group[temp_ch].tim_channel, temp_pressure);
      break;
    case 3:
      temp_ch = 1;
	  //printf("out:pid%d = %d   ",temp_ch, temp_pressure);
      __HAL_TIM_SET_COMPARE(ctrl_group[3].htim, ctrl_group[3].tim_channel, 0);
      __HAL_TIM_SET_COMPARE(ctrl_group[temp_ch].htim, ctrl_group[temp_ch].tim_channel, temp_pressure);
      break;

    }
  }

  //pid_Struct->realtime_pressure = pressure_PID_value;
  //pid_Struct->output = abs(pressure_PID_value);
}

void PID_output_1ring(PID_TypeDef* pid_Struct, float target_angle, float form_planning[101][9], int form_id)
{   
  float FFPID_output;
  pid_Struct->target_angle = target_angle;
  pid_Struct->realtime_angle=Get_Value(pid_Struct->id);
  FFPID_output = get_FF_value(pid_Struct, form_planning, form_id) +
                 pid_amplitude_limiting(angle_PID_value(pid_Struct),150);//前馈值+限幅后的PID输出
  switch_channel(pid_Struct,FFPID_output);
  __HAL_TIM_SET_COMPARE(ctrl_group[pid_Struct->output_channel].htim, 
                        ctrl_group[pid_Struct->output_channel].tim_channel, 
                                   pid_Struct->output);
}
void PID_output_2ring(PID_TypeDef* pid_Struct, float target_angle, float form_planning[101][9], int form_id)
{   
  float FFPID_output;
  pid_Struct->target_angle = target_angle;
  pid_Struct->realtime_angle=Get_Value(pid_Struct->id);
  FFPID_output = get_FF_value(pid_Struct, form_planning, form_id) +
                 pid_amplitude_limiting(pressure_PID_value(pid_Struct,angle_PID_value(pid_Struct)),150);//前馈值+限幅后的PID输出
  switch_channel(pid_Struct,FFPID_output);
  __HAL_TIM_SET_COMPARE(ctrl_group[pid_Struct->output_channel].htim, 
                        ctrl_group[pid_Struct->output_channel].tim_channel, 
                                   pid_Struct->output);
}

void PID_output_3ring(PID_TypeDef* pid_Struct, float target_angle, float form_planning[101][9], int form_id)
{      
  float FFPID_output;
  pid_Struct->target_angle = target_angle;
  pid_Struct->realtime_angle=Get_Value(pid_Struct->id);
  FFPID_output = get_FF_value(pid_Struct, form_planning, form_id) +
                 pid_amplitude_limiting(pressure_PID_value(pid_Struct,speed_PID_value(pid_Struct,angle_PID_value(pid_Struct))),150);//前馈值+限幅后的PID输出
  switch_channel(pid_Struct,FFPID_output);
  __HAL_TIM_SET_COMPARE(ctrl_group[pid_Struct->output_channel].htim, 
                        ctrl_group[pid_Struct->output_channel].tim_channel, 
                                   pid_Struct->output);  
}

void Get_error(float form_planning[100][9], int form_id){
	float joint0_rt_ang0,joint0_rt_ang1;
	float joint0_target_ang0,joint0_target_ang1;
	float a;
	float Angel0=0,Angel1=0;
	float fAngel0=0,fAngel1=0;

	joint0_rt_ang0 = ((float)angle[0][3]) / 8192 *180;
	joint0_rt_ang1 = ((float)angle[1][3]) / 8192 *180;
	joint0_target_ang0 = form_planning[form_id][5];
	joint0_target_ang1 = form_planning[form_id][6];
	Angel0 = (joint0_target_ang0 - joint0_rt_ang0)/180*3.14;
	Angel1 = (joint0_target_ang1 - joint0_rt_ang1)/180*3.14;
	
	fAngel0=asinf(1.141/2*(sinf(Angel0)+cosf(Angel0)*sinf(Angel1)));
	fAngel1=atanf(1.141/2*(tanf(Angel1)-tanf(Angel0)/cosf(Angel1)));
	pidcontrol0.error_angle = fAngel0/3.141*180;
	pidcontrol1.error_angle = fAngel1/3.141*180;
	
}
float Get_Value(uint8_t pidnum){
//	uint16_t * temp;
	int ang0=0,ang1=0;
	float a;
	float Angel0=0,Angel1=0;
	float fAngel0=0,fAngel1=0;

	ang0 = angle[0][3 - pidnum];
	ang1 = angle[1][3- pidnum];
	//printf("ang=%d",ang0);
	Angel0=(float)ang0 / 8192 * 3.141;
	Angel1=(float)ang1 / 8192 * 3.141;
	fAngel0=asinf(1.141/2*(sinf(Angel0)+cosf(Angel0)*sinf(Angel1)));
	fAngel1=atanf(1.141/2*(tanf(Angel1)-tanf(Angel0)/cosf(Angel1)));
//	ang0=(int)(fAngel0/3.141*8192);
//	ang1=(int)(fAngel1/3.141*8192);
	ang0=fAngel0/3.141*180;
	ang1=fAngel1/3.141*180;
//	printf("ang=%d",ang0);
	if(pidnum%2==0)a=ang0;
	else if(pidnum%2==1)a=ang1;
  return a;
}

float pid_amplitude_limiting(float PID_value,float limit)//限幅
{
	
  if(PID_value > limit)
  {
    return limit;
  }
  else if(PID_value < -limit)
  {
    return -limit;
  }
// TIAOSHI SHI XIAN PINGBI ,YIHOU ZAI QUDIAO ZHUSHI
//  else
//    if (PID_value < 10 && PID_value > -10)PID_value = 0;

//    return PID_value;
  else 
	  return PID_value;
}

float angle_PID_value(PID_TypeDef* pid_Struct)
{
  //pid_Struct->error_angle = pid_Struct->target_angle - pid_Struct->realtime_angle;
  pid_Struct->error_sum_angle +=  pid_Struct->error_angle;//积分
  if (fabs(pid_Struct->error_sum_angle) > pid_Struct->error_sum_angle_max)//积分限幅
  {
    if (pid_Struct->error_sum_angle > 0)
    {pid_Struct->error_sum_angle = pid_Struct->error_sum_angle_max;}
    else{pid_Struct->error_sum_angle = -pid_Struct->error_sum_angle_max;}
  }
  pid_Struct->error_difference_angle = pid_Struct->error_angle - pid_Struct->error_last_angle;//差分
  pid_Struct->error_last_angle =  pid_Struct->error_angle;//下一次循环的上一次误差
  pid_Struct->output = pid_Struct->Kp_angle * pid_Struct->error_angle + pid_Struct->Ki_angle * pid_Struct->error_sum_angle + pid_Struct->Kd_angle * pid_Struct->error_difference_angle;
  return pid_Struct->output;
}

float speed_PID_value(PID_TypeDef* pid_Struct, float angle_PID_value)
{
  pid_Struct->target_speed = angle_PID_value;
  pid_Struct->error_speed = pid_Struct->target_speed - pid_Struct->realtime_speed;
  pid_Struct->error_sum_speed +=  pid_Struct->error_speed;//积分
  pid_Struct->error_difference_speed = pid_Struct->error_speed - pid_Struct->error_last_speed;//差分
  pid_Struct->error_last_speed =  pid_Struct->error_speed;//下一次循环的上一次误差
  return pid_Struct->Kp_speed * pid_Struct->error_speed
       + pid_Struct->Ki_speed * pid_Struct->error_sum_speed
       + pid_Struct->Kd_speed * pid_Struct->error_difference_speed;
}

float pressure_PID_value(PID_TypeDef* pid_Struct, float speed_PID_value)
{
  pid_Struct->target_pressure = speed_PID_value;
  pid_Struct->error_pressure = pid_Struct->target_pressure - pid_Struct->realtime_pressure;
  pid_Struct->error_sum_pressure +=  pid_Struct->error_pressure;//积分
  pid_Struct->error_difference_pressure = pid_Struct->error_pressure - pid_Struct->error_last_pressure;//差分
  pid_Struct->error_last_pressure =  pid_Struct->error_pressure;//下一次循环的上一次误差
  return pid_Struct->Kp_pressure * pid_Struct->error_pressure
       + pid_Struct->Ki_pressure * pid_Struct->error_sum_pressure
       + pid_Struct->Kd_pressure * pid_Struct->error_difference_pressure;
}

void switch_channel(PID_TypeDef* pid_Struct, float pressure_PID_value)
{
  if(pressure_PID_value >= 0)
  {
    if (pid_Struct->id % 2 == 0) //如果是0246号pid，则输出为正时，输出给0号肌肉
    {
    pid_Struct->output_channel = 0;
    }
    else if (pid_Struct->id % 2 == 1) //如果是1357号pid，则输出为正时，输出给1号肌肉
    {
    pid_Struct->output_channel = 1;
    }
  else if(pressure_PID_value < 0)
  {
    if (pid_Struct->id % 2 == 0) //如果是0246号pid，则输出为负时，输出给2号肌肉
    {
    pid_Struct->output_channel = 2;
    }
    else if (pid_Struct->id % 2 == 1) //如果是1357号pid，则输出为正时，输出给3号肌肉
    {
    pid_Struct->output_channel = 3;
    }
  }
  }
  pid_Struct->realtime_pressure = pressure_PID_value;
  pid_Struct->output = fabs(pressure_PID_value);
}

void pid_output_v2(PID_TypeDef* pid_Struct0, PID_TypeDef* pid_Struct1,
                   PID_TypeDef* pid_Struct2, PID_TypeDef* pid_Struct3, 
                   float target_angle0, float target_angle1, float target_angle2, float target_angle3)
{
  float pidout0, pidout1, pidout2, pidout3;
  //注意
  //1.外部赋值output
  //2.每次运行前外部更新4个realtime_angle
  //
  //计算拓扑空间的误差
  //关节1
  float x1 = pid_Struct0 -> realtime_angle;
  float y1 = pid_Struct1 -> realtime_angle;
  float x0 = target_angle0;
  float y0 = target_angle1;
  float err_a = (x1 - y1 + (y0 - x0 ))/1.414;
  float err_b = (x1 + y1 - (y0 + x0 ))/1.414;
  //printf(" J1:err_a = %f err_b = %f ",err_a , err_b);
  //关节2
  float x1_j2 = pid_Struct2 -> realtime_angle;
  float y1_j2 = pid_Struct3 -> realtime_angle;
  float x0_j2 = target_angle2;
  float y0_j2 = target_angle3;
  float err_a_j2 = (x1_j2 - y1_j2 + (y0_j2 - x0_j2 ))/1.414;
  float err_b_j2 = (x1_j2 + y1_j2 - (y0_j2 + x0_j2 ))/1.414; 
  //pan duan shu chu tong dao
  float theta_temp_j0 , theta_temp_j1;
  //printf(" J2:err_a = %f err_b = %f ",err_a_j2 , err_b_j2);
  //计算pid输出
  //关节0
  pid_Struct0 -> error_angle = err_a;
  pid_Struct1 -> error_angle = err_b;
  pid_iteration(pid_Struct0);
  pid_iteration(pid_Struct1);
  pidout0 = pid_Struct0->Kp_angle * err_a 
          + pid_Struct0->Ki_angle * pid_Struct0->error_sum_angle 
          + pid_Struct0->Kd_angle * pid_Struct0->error_difference_angle;
  pidout1 = pid_Struct1->Kp_angle * err_b 
          + pid_Struct1->Ki_angle * pid_Struct1->error_sum_angle 
          + pid_Struct1->Kd_angle * pid_Struct1->error_difference_angle;
  //关节1
  pid_Struct2 -> error_angle = err_a_j2;
  pid_Struct3 -> error_angle = err_b_j2;
  pid_iteration(pid_Struct2);
  pid_iteration(pid_Struct3);  
  pidout2 = pid_Struct2->Kp_angle * pid_Struct2 -> error_angle 
          + pid_Struct2->Ki_angle * pid_Struct2->error_sum_angle 
          + pid_Struct2->Kd_angle * pid_Struct2->error_difference_angle;
  pidout3 = pid_Struct3->Kp_angle * pid_Struct3 -> error_angle 
          + pid_Struct3->Ki_angle * pid_Struct3->error_sum_angle 
          + pid_Struct3->Kd_angle * pid_Struct3->error_difference_angle;
  //PID输出限幅
  pidout0 = pid_amplitude_limiting(pidout0,80);
  pidout1 = pid_amplitude_limiting(pidout1,80);
  pidout2 = pid_amplitude_limiting(pidout2,80);
  pidout3 = pid_amplitude_limiting(pidout3,80);
  //pid值加到PQ轴上，更新PQ轴数据-------------------------------似乎是负号，所以是-=
  joint0.P -= (short)round(pidout0);
  joint0.Q -= (short)round(pidout1);
  joint1.P -= (short)round(pidout2);
  joint1.Q -= (short)round(pidout3);
  //判断输出方向
  //关节0
  if(joint0.Q >= 0){
    joint0.output0 = joint0.Q;
    joint0.output2 = 0;}
  else{
    joint0.output0 = 0;
    joint0.output2 = -joint0.Q;
  }
  if(joint0.P >= 0){
    joint0.output3 = joint0.P;
    joint0.output1 = 0;}
  else{
    joint0.output3 = 0;
    joint0.output1 = -joint0.P;
  }
  //关节1
  if(joint1.Q >= 0){
    joint1.output0 = joint1.Q;
    joint1.output2 = 0;}
  else{
    joint1.output0 = 0;
    joint1.output2 = -joint1.Q;
  }
  if(joint1.P >= 0){
    joint1.output3 = joint1.P;
    joint1.output1 = 0;}
  else{
    joint1.output3 = 0;
    joint1.output1 = -joint1.P;
  }


  
  //限幅
  //joint_OUTPUT_LIMIT();
  //输出
  air_output_j1(joint0.output0, joint0.output1, joint0.output2, joint0.output3);
  air_output_j2(joint1.output0, joint1.output1, joint1.output2, joint1.output3);
  //临时数据清零
  pidout0 = 0;pidout1 = 0;pidout2 = 0;pidout3 = 0;
}

//PID迭代赋值
void pid_iteration(PID_TypeDef* pid_Struct)
{
  //计算积分
  pid_Struct->error_sum_angle +=  pid_Struct->error_angle;
  //积分限幅
  if (fabs(pid_Struct->error_sum_angle) > pid_Struct->error_sum_angle_max)
  {
    if (pid_Struct->error_sum_angle > 0)
    {pid_Struct->error_sum_angle = pid_Struct->error_sum_angle_max;}
    else{pid_Struct->error_sum_angle = -pid_Struct->error_sum_angle_max;}
  }
  //计算差分
  pid_Struct->error_difference_angle = pid_Struct->error_angle - pid_Struct->error_last_angle;
  //下一次循环的上一次误差
  pid_Struct->error_last_angle =  pid_Struct->error_angle;
}

void air_output_j1(float value0,float value1,float value2,float value3)
{
    __HAL_TIM_SET_COMPARE(ctrl_group[0].htim, ctrl_group[0].tim_channel, value0); 
    __HAL_TIM_SET_COMPARE(ctrl_group[1].htim, ctrl_group[1].tim_channel, value1);
    __HAL_TIM_SET_COMPARE(ctrl_group[2].htim, ctrl_group[2].tim_channel, value2);
    __HAL_TIM_SET_COMPARE(ctrl_group[3].htim, ctrl_group[3].tim_channel, value3);
}

void air_output_j2(float value0,float value1,float value2,float value3)
{
    __HAL_TIM_SET_COMPARE(ctrl_group[4].htim, ctrl_group[4].tim_channel, value0); 
    __HAL_TIM_SET_COMPARE(ctrl_group[5].htim, ctrl_group[5].tim_channel, value1);
    __HAL_TIM_SET_COMPARE(ctrl_group[6].htim, ctrl_group[6].tim_channel, value2);
    __HAL_TIM_SET_COMPARE(ctrl_group[7].htim, ctrl_group[7].tim_channel, value3);
}

void joint_output_limited_between(int value)
{
	//j1
	joint0.output0_max = joint0.output0 + value;
	joint0.output1_max = joint0.output1 + value;
	joint0.output2_max = joint0.output2 + value;
	joint0.output3_max = joint0.output3 + value;
	joint0.output0_min = joint0.output0 - value;
	joint0.output1_min = joint0.output1 - value;
	joint0.output2_min = joint0.output2 - value;
	joint0.output3_min = joint0.output3 - value;
	//j2
	joint1.output0_max = joint1.output0 + value;
	joint1.output1_max = joint1.output1 + value;
	joint1.output2_max = joint1.output2 + value;
	joint1.output3_max = joint1.output3 + value;
	joint1.output0_min = joint1.output0 - value;
	joint1.output1_min = joint1.output1 - value;
	joint1.output2_min = joint1.output2 - value;
	joint1.output3_min = joint1.output3 - value;
}
void joint_OUTPUT_LIMIT()
{
	//J0 MAX
	if(joint0.output0 > joint0.output0_max){joint0.output0 = joint0.output0_max;}
	if(joint0.output1 > joint0.output1_max){joint0.output1 = joint0.output1_max;}
	if(joint0.output2 > joint0.output2_max){joint0.output2 = joint0.output2_max;}
	if(joint0.output3 > joint0.output3_max){joint0.output3 = joint0.output3_max;}
	//J0 MIN
	if(joint0.output0 < joint0.output0_min){joint0.output0 = joint0.output0_min;}
	if(joint0.output1 < joint0.output1_min){joint0.output1 = joint0.output1_min;}
	if(joint0.output2 < joint0.output2_min){joint0.output2 = joint0.output2_min;}
	if(joint0.output3 < joint0.output3_min){joint0.output3 = joint0.output3_min;}
	//J1 MAX
	if(joint1.output0 > joint1.output0_max){joint1.output0 = joint1.output0_max;}
	if(joint1.output1 > joint1.output1_max){joint1.output1 = joint1.output1_max;}
	if(joint1.output2 > joint1.output2_max){joint1.output2 = joint1.output2_max;}
	if(joint1.output3 > joint1.output3_max){joint1.output3 = joint1.output3_max;}
	//J1 MIN
	if(joint1.output0 < joint1.output0_min){joint1.output0 = joint1.output0_min;}
	if(joint1.output1 < joint1.output1_min){joint1.output1 = joint1.output1_min;}
	if(joint1.output2 < joint1.output2_min){joint1.output2 = joint1.output2_min;}
	if(joint1.output3 < joint1.output3_min){joint1.output3 = joint1.output3_min;}	
}
float get_theta(float x,float y)
{
	//0-360degree
	//
	float theta;
	theta = atan(y/x) /3.1415 *180;
	if (x > 0){
		if (y < 0){
			theta = 360 + theta;
		}
	}	
	if (x < 0){
		theta = 180 + theta;
	}	
	if (x == 0){
		if (y < 0){
			theta = 270;
		}
		if (y >= 0){
			theta = 90;
		}
		if (y == 0){
			theta = 999;//TE SHU QING KUANG
		}
	}
	return theta;
}	
