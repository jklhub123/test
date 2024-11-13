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
extern uint8_t pidstart;
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for read */
osThreadId_t readHandle;
const osThreadAttr_t read_attributes = {
  .name = "read",
  .stack_size = 128 * 4,
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
//  //等待�?启信�?
//	pidstart = 1;
//	while(1)
//	{
//		if(pidstart == 1){break;}
//		printf("waitting");
//		osDelay(500);
//	}
//  //�?�?
//	printf("Task2 processing");
//  //回零 初始�?
//	Set_all_pressure2(0);
//	PID_INIT(2);
//  osDelay(1000);
//  //�?始执�?
//  joint_2_MOV_CLOSE_test();
  osDelay(3000);
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
    Read_B();
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
     osDelay(500);
	 //printf("Task4 sending");
  }
  /* USER CODE END StartTask_send */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

