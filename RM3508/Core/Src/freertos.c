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
osThreadId myTask03Handle;
osThreadId myGetBallHandle;
osThreadId myReadyGetBallHandle;
osThreadId myReadyPutBallHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartGetBall(void const * argument);
void StartReadyGetBall(void const * argument);
void StartReadyPutBall(void const * argument);

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

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myGetBall */
  osThreadDef(myGetBall, StartGetBall, osPriorityIdle, 0, 128);
  myGetBallHandle = osThreadCreate(osThread(myGetBall), NULL);

  /* definition and creation of myReadyGetBall */
  osThreadDef(myReadyGetBall, StartReadyGetBall, osPriorityNormal, 0, 128);
  myReadyGetBallHandle = osThreadCreate(osThread(myReadyGetBall), NULL);

  /* definition and creation of myReadyPutBall */
  osThreadDef(myReadyPutBall, StartReadyPutBall, osPriorityIdle, 0, 128);
  myReadyPutBallHandle = osThreadCreate(osThread(myReadyPutBall), NULL);

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

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
//		printf("speed_1,pos_1:%f,%f\n",Debug_speed_1,Debug_Pos_1);
//		printf("%f,%f,%f\n",gyro.x,gyro.y,gyro.z);
//		printf("%f,%f,%f,%f\n",Debug_speed_1,Debug_speed_2,Debug_speed_3,Debug_speed_4);
//		printf("%f,%f\n",Debug_Pos_1,Debug_Pos_2);
//		printf("%f,%f\n",Debug_Pos_3,Debug_Pos_4);
//		printf("%f\n",yaw_adj.Yaw);
//  		printf("%f\n",Debug_Pos_1);
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartGetBall */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetBall */
void StartGetBall(void const * argument)
{
  /* USER CODE BEGIN StartGetBall */
	vTaskSuspend(myGetBallHandle); //暂停任务，等待唤醒
  /* Infinite loop */
  for(;;)
  {
		//这里是控制机械臂夹球的代码
		while(1) //循环控制
		{
			RM3508_Set_Pos(-90 * 3276.8, 5); //抓取方向电机
			RM3508_Set_Pos(168 * 1297, 6); //大臂电机抓取位置
			RM3508_Set_Pos(30 * 1638.4, 7); //小臂电机
			
			if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
			{
				for(int i = 0;i<10;i++)
				{
					RM3508_Set_I(0, 5);
					RM3508_Set_I(0, 6);
					RM3508_Set_I(0, 7);
					osDelay(1);
				}
				break;
			}
			osDelay(1);
		}
			vTaskResume(myReadyGetBallHandle);
      vTaskSuspend(myGetBallHandle); //暂停任务，等待唤醒
//		vTaskResume(myStoreBallHandle); //存球
//		Ball_got = 1; //夹完球更改标签
		
    osDelay(1);
  }
  /* USER CODE END StartGetBall */
}

/* USER CODE BEGIN Header_StartReadyGetBall */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadyGetBall */
void StartReadyGetBall(void const * argument)
{
  /* USER CODE BEGIN StartReadyGetBall */
	vTaskSuspend(myReadyGetBallHandle);
	
//	RM3508_Set_NowPos(5, 0);
//	RM3508_Set_NowPos(6, 0);
//	RM3508_Set_NowPos(7, 0);
	
  /* Infinite loop */
  for(;;)
  {
		RM3508_Set_Pos(0 * 3276.8, 5); //抓取方向电机
		RM3508_Set_Pos(0 * 1297, 6); //大臂电机
		RM3508_Set_Pos(0 * 1638.4, 7); //小臂电机
		
		//调试用跳出
		if(fabs(M3508_Pos_Pid[4].Err) <= 5 && fabs(M3508_Pos_Pid[5].Err) <= 5 && fabs(M3508_Pos_Pid[6].Err) <= 5)
		{
			for(int i = 0;i<50;i++)
			{
				RM3508_Set_I(0, 5);
				RM3508_Set_I(0, 6);
				RM3508_Set_I(0, 7);
				osDelay(1);
			}
//			vTaskResume(myGetBallHandle);
//			vTaskSuspend(myReadyGetBallHandle);
		}
		
    osDelay(1);
  }
  /* USER CODE END StartReadyGetBall */
}

/* USER CODE BEGIN Header_StartReadyPutBall */
/**
* @brief Function implementing the myReadyPutBall thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadyPutBall */
void StartReadyPutBall(void const * argument)
{
  /* USER CODE BEGIN StartReadyPutBall */
  /* Infinite loop */
 
    RM3508_Set_NowPos(5, 0);
    RM3508_Set_NowPos(6, 0);
    RM3508_Set_NowPos(7, 0);
    
  /* Infinite loop */
  for(;;)
  {
        RM3508_Set_Pos(-90 * 3276.8, 5); //抓取方向电机
        RM3508_Set_Pos(130 * 1297, 6); //大臂电机
        RM3508_Set_Pos(30 * 1638.4, 7); //小臂电机
        
        //调试用跳出
        if(fabs(M3508_Pos_Pid[4].Err) <= 5 && fabs(M3508_Pos_Pid[5].Err) <= 5 && fabs(M3508_Pos_Pid[6].Err) <= 5)
        {
            for(int i = 0;i<50;i++)
            {
                RM3508_Set_I(0, 5);
                RM3508_Set_I(0, 6);
                RM3508_Set_I(0, 7);
                osDelay(1);
            }
            vTaskResume(myGetBallHandle);
           vTaskSuspend(myReadyPutBallHandle);
						
        }
        
    osDelay(1);
  }
  /* USER CODE END StartReadyPutBall */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
