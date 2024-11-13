/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hjcMotionControlLib.h"
#include "myPID.h"
#include "protocol.h"
#include "gpio.h"
#include "spi.h"
extern uint8_t pidstart;
extern int16_t angle[2][4][5];
extern short angle_ReadIndex;
//关节指令
extern short GLOBAL_ANGLE_FLAG;
extern int16_t GLOBAL_ANGLE[2][3][4];
extern short GLOBAL_ANGLE_ReadIndex;
//开环气压指令
extern short GLOBAL_PRESSURE_FLAG;
extern int16_t GLOBAL_PRESSURE[2][5][4];
extern short GLOBAL_PRESSURE_ReadIndex;
//
extern int8_t GLOBAL_DEEP_MOTOR_FLAG;
extern uint16_t GLOBAL_DEEP_MOTOR[6];
extern int8_t GLOBAL_DEEP_MOTOR_ReadIndex;
extern int8_t GLOBAL_DEEP_MOTOR_ON_FLAG;
extern int8_t GLOBAL_DEEP_MOTOR_OFF_FLAG;
extern int8_t GLOBAL_BRMXZ_ACTION_1_FLAG;
//
extern uint8_t pressurech;
extern uint16_t pressurevalue;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PID */
osThreadId_t PIDHandle;
const osThreadAttr_t PID_attributes = {
  .name = "PID",
  .stack_size = 1280 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for read */
osThreadId_t readHandle;
const osThreadAttr_t read_attributes = {
  .name = "read",
  .stack_size = 1280 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for send */
osThreadId_t sendHandle;
const osThreadAttr_t send_attributes = {
  .name = "send",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask_pid(void *argument);
void StartTask_read(void *argument);
void StartTask_send(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of PID */
  PIDHandle = osThreadNew(StartTask_pid, NULL, &PID_attributes);

  /* creation of read */
  readHandle = osThreadNew(StartTask_read, NULL, &read_attributes);

  /* creation of send */
  sendHandle = osThreadNew(StartTask_send, NULL, &send_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	//printf("001");
	//receiving_process();
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    osDelay(500);
    
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask_pid */
/**
* @brief Function implementing the PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_pid */
void StartTask_pid(void *argument)
{
  /* USER CODE BEGIN StartTask_pid */
  /* Infinite loop */
  for(;;)
	{	
		//read_4_button();
    //关节指令
		if(GLOBAL_ANGLE_FLAG == 1)
			{
			MOVJ_v1_0(GLOBAL_ANGLE, GLOBAL_ANGLE_ReadIndex);
			GLOBAL_ANGLE_FLAG = 0;
			}
    //气压指令
		if(GLOBAL_PRESSURE_FLAG == 1)
			{
			MOVP_v1_0(GLOBAL_PRESSURE, GLOBAL_PRESSURE_ReadIndex);
			GLOBAL_PRESSURE_FLAG = 0;
			}
		if(GLOBAL_PRESSURE_FLAG == 2)
			{
			MOVP_v1_0_right_arm(GLOBAL_PRESSURE, GLOBAL_PRESSURE_ReadIndex);
			GLOBAL_PRESSURE_FLAG = 0;
			}
	//电机指令
		if(GLOBAL_DEEP_MOTOR_FLAG == 1)
			{
				DEEP_MOTOR_TEST_v1_0(GLOBAL_DEEP_MOTOR , GLOBAL_DEEP_MOTOR_ReadIndex , 1);
				GLOBAL_DEEP_MOTOR_FLAG = 0;
			}
		if(GLOBAL_DEEP_MOTOR_FLAG == 2)
			{
				DEEP_MOTOR_TEST_v1_0(GLOBAL_DEEP_MOTOR , GLOBAL_DEEP_MOTOR_ReadIndex , 2);
				GLOBAL_DEEP_MOTOR_FLAG = 0;
			}
		if(GLOBAL_DEEP_MOTOR_FLAG == 3)
			{
				DEEP_MOTOR_TEST_v1_0(GLOBAL_DEEP_MOTOR , GLOBAL_DEEP_MOTOR_ReadIndex , 3);
				GLOBAL_DEEP_MOTOR_FLAG = 0;
			}
		if(GLOBAL_DEEP_MOTOR_FLAG == 4)
			{
				DEEP_MOTOR_TEST_v1_0(GLOBAL_DEEP_MOTOR , GLOBAL_DEEP_MOTOR_ReadIndex , 4);
				GLOBAL_DEEP_MOTOR_FLAG = 0;
			}
	//电机开关
		if(GLOBAL_DEEP_MOTOR_ON_FLAG == 1){
		  for(short i=1;i<=4;i++)
			{
			turn_off_DEEP_Motor(i);
			osDelay(1);
			}
		  GLOBAL_DEEP_MOTOR_ON_FLAG =0;
		  printf("ENABLE DEEP_MOTOR\r\n");
		  printf("-------------------------\r\n");
		  }
		  //
		if(GLOBAL_DEEP_MOTOR_OFF_FLAG == 1){
		  for(short i=1;i<=4;i++)
			{
			turn_off_DEEP_Motor(i);
			osDelay(1);
			}
		  GLOBAL_DEEP_MOTOR_OFF_FLAG =0;
		  printf("DISABLE DEEP_MOTOR\r\n");
		  printf("-------------------------\r\n");
		  }
	if(GLOBAL_BRMXZ_ACTION_1_FLAG == 1){
      //turn_off_DEEP_Motor(1);
      GLOBAL_BRMXZ_ACTION_1_FLAG =0;
      printf("MOVE ACTION 1 \r\n");
	  printf("-------------------------\r\n");
      }
	  
	osDelay(300);
	}
  /* USER CODE END StartTask_pid */
}

/* USER CODE BEGIN Header_StartTask_read */
/**
* @brief Function implementing the read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_read */
void StartTask_read(void *argument)
{
  /* USER CODE BEGIN StartTask_read */
  /* Infinite loop */
  for(;;)
  {
    //Read_B();
    osDelay(10);
	//printf(" Task3 readBing ");
  }
  /* USER CODE END StartTask_read */
}

/* USER CODE BEGIN Header_StartTask_send */
/**
* @brief Function implementing the send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_send */
void StartTask_send(void *argument)
{
  /* USER CODE BEGIN StartTask_send */
  /* Infinite loop */
  for(;;)
  {
		osDelay(800);
		//printf("Task4 sending");
  }
  /* USER CODE END StartTask_send */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

