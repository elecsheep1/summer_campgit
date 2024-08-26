/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "RM3508.h"
#include "can_bsp.h"
#include "DR_rm3508.h"
//#include "rgb.h"
#include "ICM42688.h"
#include "filt.h"
#include "point_control.h"
#include "arm_control.h"
#include "gpio.h"

#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"

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
float x;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myReadyPosHandle;
osThreadId myPutBall1Handle;
osThreadId myPutBall2Handle;
osThreadId myGetBall1Handle;
osThreadId myGetBall2Handle;
osThreadId myJudgeHandle;
osThreadId mySetM567Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartReadyPos(void const * argument);
void StartPutBall1(void const * argument);
void StartPutBall2(void const * argument);
void StartGetBall1(void const * argument);
void StartGetBall2(void const * argument);
void StartJudge(void const * argument);
void StartSetM567(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityHigh, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myReadyPos */
  osThreadDef(myReadyPos, StartReadyPos, osPriorityNormal, 0, 128);
  myReadyPosHandle = osThreadCreate(osThread(myReadyPos), NULL);

  /* definition and creation of myPutBall1 */
  osThreadDef(myPutBall1, StartPutBall1, osPriorityNormal, 0, 128);
  myPutBall1Handle = osThreadCreate(osThread(myPutBall1), NULL);

  /* definition and creation of myPutBall2 */
  osThreadDef(myPutBall2, StartPutBall2, osPriorityNormal, 0, 128);
  myPutBall2Handle = osThreadCreate(osThread(myPutBall2), NULL);

  /* definition and creation of myGetBall1 */
  osThreadDef(myGetBall1, StartGetBall1, osPriorityNormal, 0, 128);
  myGetBall1Handle = osThreadCreate(osThread(myGetBall1), NULL);

  /* definition and creation of myGetBall2 */
  osThreadDef(myGetBall2, StartGetBall2, osPriorityNormal, 0, 128);
  myGetBall2Handle = osThreadCreate(osThread(myGetBall2), NULL);

  /* definition and creation of myJudge */
  osThreadDef(myJudge, StartJudge, osPriorityAboveNormal, 0, 128);
  myJudgeHandle = osThreadCreate(osThread(myJudge), NULL);

  /* definition and creation of mySetM567 */
  osThreadDef(mySetM567, StartSetM567, osPriorityAboveNormal, 0, 128);
  mySetM567Handle = osThreadCreate(osThread(mySetM567), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
		
		
//		if(Point_test == 1)
//		{
//			while(Robot_Pos_Move_PID_V1 (&Path_PID_Test))
//			{
//				Point_test = 0;
//				break;
//			}
//			osDelay(1);
//		}
		
		
//      RM3508_text();
//		Data_Init(K_filt1,G_yaw);
//    getICM42688data(K_filt,G_yaw);
//		ICM42688_yaw_adjust(K_filt,G_yaw);
//		ICM42688_VX_adjust(K_filt,G_yaw);
//		ICM42688_X_adjust(K_filt,G_yaw);
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartReadyPos */
/**
* @brief Function implementing the myReadyPos thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadyPos */
void StartReadyPos(void const * argument)
{
  /* USER CODE BEGIN StartReadyPos */

  /* Infinite loop */
   for(;;)
  {
		//机械臂预备位置
			 M5_Pos = M5_READY * 1638.4; //抓取方向电机
       M6_Pos = M6_READY * 1297; //大臂电机抓取位置
       M7_Pos = M7_READY * 1638.4; //小臂电机
       osDelay(1);
  }
  /* USER CODE END StartReadyPos */
}

/* USER CODE BEGIN Header_StartPutBall1 */
/**
* @brief Function implementing the myPutBall1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPutBall1 */
void StartPutBall1(void const * argument)
{
  /* USER CODE BEGIN StartPutBall1 */
  /* Infinite loop */
   for(;;)
  {
		vTaskSuspend(myPutBall1Handle); //暂停任务，等待唤醒
		//机械臂放球
		M5_Pos = -90 * 1638.4; //抓取方向电机
    M6_Pos = M6_PUT_1 * 1297; //大臂电机抓取位置
    M7_Pos = M7_PUT_1 * 1638.4; //小臂电机
		osDelay(5);
		while(1)
		{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				osDelay(1);
		 }
		osDelay(300);
		HAL_GPIO_WritePin (Arm_GPIO_Port ,Arm_Pin ,0);//电磁阀打开放球
		osDelay(700);
		vTaskResume(myReadyPosHandle);//预备位置
		PUT_BALL_1 = 0;
    osDelay(1);
  }
  /* USER CODE END StartPutBall1 */
}

/* USER CODE BEGIN Header_StartPutBall2 */
/**
* @brief Function implementing the myPutBall2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPutBall2 */
void StartPutBall2(void const * argument)
{
  /* USER CODE BEGIN StartPutBall2 */
  /* Infinite loop */
  for(;;)
  {
    vTaskSuspend(myPutBall2Handle); //暂停任务，等待唤醒
		//机械臂放球
		M5_Pos = M5_PUT_2 * 1638.4; //抓取方向电机
    M6_Pos = M6_PUT_2 * 1297; //大臂电机抓取位置
    M7_Pos = M7_PUT_2 * 1638.4; //小臂电机
		while(1)
		{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				osDelay(1);
		}
		osDelay(300);
		HAL_GPIO_WritePin (Arm_GPIO_Port ,Arm_Pin ,0);//电磁阀打开放球
		osDelay(700);
		vTaskResume(myReadyPosHandle);//预备位置
		PUT_BALL_2 = 0;
    osDelay(1);
	}
  /* USER CODE END StartPutBall2 */
}

/* USER CODE BEGIN Header_StartGetBall1 */
/**
* @brief Function implementing the myGetBall1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetBall1 */
void StartGetBall1(void const * argument)
{
  /* USER CODE BEGIN StartGetBall1 */
  /* Infinite loop */
 for(;;)
  {
    vTaskSuspend(myGetBall1Handle); //暂停任务，等待唤醒
		//机械臂放球
		M5_Pos = -90 * 1638.4; //抓取方向电机
    M6_Pos = M6_GET_1 * 1297; //大臂电机抓取位置
    M7_Pos = M7_GET_1 * 1638.4; //小臂电机
		osDelay(5);
		while(1)
		{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				osDelay(1);
		 }
		osDelay(300);
		HAL_GPIO_WritePin (Arm_GPIO_Port ,Arm_Pin ,1);//电磁阀打开放球
		osDelay(700);
		vTaskResume(myReadyPosHandle);//预备位置
		GET_BALL_1 = 0;
    osDelay(1);
	}
  /* USER CODE END StartGetBall1 */
}

/* USER CODE BEGIN Header_StartGetBall2 */
/**
* @brief Function implementing the myGetBall2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetBall2 */
void StartGetBall2(void const * argument)
{
  /* USER CODE BEGIN StartGetBall2 */
  /* Infinite loop */
  for(;;)
  {
    vTaskSuspend(myGetBall2Handle); //暂停任务，等待唤醒
		//机械臂放球
			//第一个球
			if(BOX == 5)
			{
			M5_Pos = M5_GET1_2 * 1638.4; //抓取方向电机
			M6_Pos = M6_GET1_2 * 1297; //大臂电机抓取位置
			M7_Pos = M7_GET1_2 * 1638.4; //小臂电机
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					osDelay(1);
		  }
			osDelay(300);
			HAL_GPIO_WritePin (Arm_GPIO_Port ,Arm_Pin ,1);//电磁阀关闭抓球
			osDelay(700);
		  }
			
			//第二个球
			if(BOX == 4)
			{
			M5_Pos = M5_GET1_2 * 1638.4; //抓取方向电机
			M6_Pos = M6_GET1_2 * 1297; //大臂电机抓取位置
			M7_Pos = M7_GET1_2 * 1638.4; //小臂电机
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					osDelay(1);
		  }
			osDelay(300);
			HAL_GPIO_WritePin (Arm_GPIO_Port ,Arm_Pin ,1);//电磁阀关闭抓球
			osDelay(700);
		  }
			
			//第三个球
			if(BOX == 3)
			{
			M5_Pos = M5_GET1_2 * 1638.4; //抓取方向电机
			M6_Pos = M6_GET1_2 * 1297; //大臂电机抓取位置
			M7_Pos = M7_GET1_2 * 1638.4; //小臂电机
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					osDelay(1);
		  }
			osDelay(300);
			HAL_GPIO_WritePin (Arm_GPIO_Port ,Arm_Pin ,1);//电磁阀关闭抓球
			osDelay(700);
		  }
			
			//第四个球
			if(BOX == 2)
			{
			M5_Pos = M5_GET1_2 * 1638.4; //抓取方向电机
			M6_Pos = M6_GET1_2 * 1297; //大臂电机抓取位置
			M7_Pos = M7_GET1_2 * 1638.4; //小臂电机
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					osDelay(1);
		  }
			osDelay(300);
			HAL_GPIO_WritePin (Arm_GPIO_Port ,Arm_Pin ,1);//电磁阀关闭抓球
			osDelay(700);
		  }
			
			//第五个球
			if(BOX == 1)
			{
			M5_Pos = M5_GET1_2 * 1638.4; //抓取方向电机
			M6_Pos = M6_GET1_2 * 1297; //大臂电机抓取位置
			M7_Pos = M7_GET1_2 * 1638.4; //小臂电机
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					osDelay(1);
		  }
			osDelay(300);
			HAL_GPIO_WritePin (Arm_GPIO_Port ,Arm_Pin ,1);//电磁阀关闭抓球
			osDelay(700);
		  }
			vTaskResume(myReadyPosHandle);//预备位置
		  BOX--;
		  GET_BALL_2 = 0;
      osDelay(1);
	}
  /* USER CODE END StartGetBall2 */
}

/* USER CODE BEGIN Header_StartJudge */
/**
* @brief Function implementing the myJudge thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJudge */
void StartJudge(void const * argument)
{
  /* USER CODE BEGIN StartJudge */
  /* Infinite loop */
  for(;;)
  {
		if(PUT_BALL_1)
		{
			vTaskSuspend(myReadyPosHandle);
			vTaskResume(myPutBall1Handle);
		}
    if(PUT_BALL_2)
		{
			vTaskSuspend(myReadyPosHandle);
			vTaskResume(myPutBall2Handle);
		}
    if(GET_BALL_1)
		{
			vTaskSuspend(myReadyPosHandle);
			vTaskResume(myGetBall1Handle);
		}
		if(GET_BALL_2)
		{
			vTaskSuspend(myReadyPosHandle);
			vTaskResume(myGetBall2Handle);
		}
    osDelay(1);
  }
  /* USER CODE END StartJudge */
}

/* USER CODE BEGIN Header_StartSetM567 */
/**
* @brief Function implementing the mySetM567 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSetM567 */
void StartSetM567(void const * argument)
{
  /* USER CODE BEGIN StartSetM567 */
	
	RM3508_Set_NowPos(5, 0);
	RM3508_Set_NowPos(6, 0);
	RM3508_Set_NowPos(7, 0);
//	 HAL_GPIO_WritePin (Arm_GPIO_Port ,Arm_Pin ,1);//电磁阀关闭抓球
	
  /* Infinite loop */
  for(;;)
  {

			RM3508_Set_Pos(M5_Pos, 5); //抓取方向电机
			RM3508_Set_Pos(M6_Pos, 6); //大臂电机
			RM3508_Set_Pos(M7_Pos, 7); //小臂电机
      osDelay(1);
  }
  /* USER CODE END StartSetM567 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
