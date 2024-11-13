#include "pid.h"
#include "protocol.h"
#include "spi.h"


unsigned char cmd[8]={'A','0','2','B','0','0','0','0'};
float increase[20] = {0};
pid_parameter pidm[20];
uint8_t a[4];
/** @brief  init PID parameter.
  * @param  void. 
  * @retval void.
  */
void pidinit()
{
	//10°„ kp2.15,ki0.5,kd1.0 ∞Î√Î
//	pidm.kp = 0;
//	pidm.ki = 0;
//	pidm.kd = 0;
	for(int i =0 ;i<20;i++){
		pidm[i].max_limit = 1800;
		pidm[i].min_limit = 0;
		pidm[i].error = 0;
		pidm[i].last_error = 0;
		pidm[i].pre_error = 0;
		pidm[i].send = 0;
		increase[i] = 0;
	}
}
/** @brief  update PID parameter/1000.
  * @param  ch specifies the channel.3 specifies the fingertips.
  * @param  _kp specifies p.  
  * @param  _ki specifies i.
  * @param  _kd specifies d.
  * @retval void.
  */
void updateparameter(int ch,float _kp ,float _ki ,float _kd)
{
	pidm[ch].kp = _kp/1000.0;
	pidm[ch].ki = _ki/1000.0;
	pidm[ch].kd = _kd/1000.0;
	//printf("%f\t%f\t%f\r\n",pidm.kp,pidm.ki,pidm.kd);
}
/** @brief  PID.
  * @param  ch specifies the channel.3 specifies the fingertips.
  * @param  tangle specifies target angle.  
	* @param  rangle specifies real angle. 
  * @retval pressure to send.
  */
float anglepid(uint8_t ch,float tangle,float rangle)
{
	
	pidm[ch].error = tangle - rangle;
	//if(pidm[ch].error<=0.5&&pidm[ch].error>=(-0.5))pidm[ch].error = 0;
	increase[ch] = pidm[ch].kp*(pidm[ch].error-pidm[ch].last_error)+pidm[ch].ki*pidm[ch].error+pidm[ch].kd*(pidm[ch].error-2*pidm[ch].last_error+pidm[ch].pre_error);
	//printf("%f\t",increase);
	pidm[ch].send += increase[ch];
	if(pidm[ch].send>pidm[ch].max_limit)
	{
		pidm[ch].send = pidm[ch].max_limit;
	}
	else if(pidm[ch].send < pidm[ch].min_limit)
	{
		pidm[ch].send = pidm[ch].min_limit;
	}
	pidm[ch].pre_error = pidm[ch].last_error;
	pidm[ch].last_error = pidm[ch].error;
	
	return pidm[ch].send;
}
/** @brief  PID and sendpressure.
  * @param  ch specifies the channel.3 specifies the fingertips.
  * @param  _target specifies target angle.   
  * @retval void.
  */
void sendpressure(uint8_t ch,float _target)
{
	
	READ_Angle(0);
	int16_t angle = Mechanical_Angel[0][ch]-Angel_Zero[0][ch];
	if(angle < -8192)angle += 16384;
	if(angle > 8192)angle -= 16384;
	if(ch==1||ch==3)angle = (-angle);  
	float rangle = (float)(angle)/8192*180;
	int16_t tarangle = _target;
	//set_computer_value(SEND_FACT_CMD, 5, &tarangle, 1);
	
	uint16_t send = (uint16_t)anglepid(ch,_target,rangle);

	a[0] = ch;
	a[1] = 0x01;
	a[2] = (send>>8)&0x00ff;
	a[3] = send&0x00ff;
	// CAN_SendToFa(1,a);
//	for(int i=4;i<8;i++){
//		cmd[i]=a[i-4]+'0';
//	}
//	HAL_UART_Transmit(&huart2,cmd ,8,10); 
}
/** @brief  sendpressure.
  * @param  ch specifies the channel.3 specifies the fingertips.  
  * @param  presure specifies the presure to send.  
  * @retval void.
  */
void sendpressure_Nopid(uint8_t ch,float presure)
{
	uint16_t send = (uint16_t)presure;

	a[0] = ch;
	a[1] = 0x01;
	a[2] = (send>>8)&0x00ff;
	a[3] = send&0x00ff;
	// CAN_SendToFa(1,a);
//	for(int i=4;i<8;i++){
//		cmd[i]=a[i-4]+'0';
//	}
//	HAL_UART_Transmit(&huart2,cmd ,8,10); 
}