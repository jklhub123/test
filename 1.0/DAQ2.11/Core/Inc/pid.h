#ifndef __PID_H__
#define __PID_H__

#include "main.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

typedef struct
{
	int ctrlchannel;
	int ctrlvalue;
}ctrlgroup;

typedef struct 
{
	float kp,ki,kd;
	float error,last_error,pre_error;
	float send,max_limit,min_limit;
}pid_parameter;


extern pid_parameter pidm[20];
extern uint8_t a[4];


void pidinit(void);
void updateparameter(int ch,float _kp ,float _ki ,float _kd);
float anglepid(uint8_t ch,float tangle,float rangle);
void sendpressure(uint8_t ch,float target);
void sendpressure_Nopid(uint8_t ch,float presure);

#endif
