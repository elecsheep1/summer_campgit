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
#include "mySerial.h"
#include "can_bsp.h"
#include "mycan_init.h"
#include "RM3508.h"
#include "rgb.h"
#include "ICM42688.h"
#include "Mecanum_wheel.h"
#include "arm_control.h"
#include "point_control.h"
//#include "TFminiPlus.h"
#include "run.h"
#include "math.h"

#include "tim.h"
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
osThreadId defaultTaskHandle;
osThreadId myMoveToBallHandle;
osThreadId myGetBallHandle;
osThreadId myReadyGetBallHandle;
osThreadId myStoreBallHandle;
osThreadId mySetM567Handle;
osThreadId myPutBall1Handle;
osThreadId myPutBall2Handle;
osThreadId myGetBall1Handle;
osThreadId myGetBall2Handle;
osThreadId myJudgeHandle;
osThreadId myReadyPos1Handle;
osThreadId myPointRunHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartMoveToBall(void const * argument);
void StartGetBall(void const * argument);
void StartReadyGetBall(void const * argument);
void StartStoreBall(void const * argument);
void StartSetM567(void const * argument);
void StartPutBall1(void const * argument);
void StartPutBall2(void const * argument);
void StartGetBall1(void const * argument);
void StartGetBall2(void const * argument);
void StartJudge(void const * argument);
void StartReadyPos1(void const * argument);
void StartPointRun(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myMoveToBall */
  osThreadDef(myMoveToBall, StartMoveToBall, osPriorityNormal, 0, 512);
  myMoveToBallHandle = osThreadCreate(osThread(myMoveToBall), NULL);

  /* definition and creation of myGetBall */
  osThreadDef(myGetBall, StartGetBall, osPriorityNormal, 0, 128);
  myGetBallHandle = osThreadCreate(osThread(myGetBall), NULL);

  /* definition and creation of myReadyGetBall */
  osThreadDef(myReadyGetBall, StartReadyGetBall, osPriorityNormal, 0, 128);
  myReadyGetBallHandle = osThreadCreate(osThread(myReadyGetBall), NULL);

  /* definition and creation of myStoreBall */
  osThreadDef(myStoreBall, StartStoreBall, osPriorityNormal, 0, 256);
  myStoreBallHandle = osThreadCreate(osThread(myStoreBall), NULL);

  /* definition and creation of mySetM567 */
  osThreadDef(mySetM567, StartSetM567, osPriorityAboveNormal, 0, 128);
  mySetM567Handle = osThreadCreate(osThread(mySetM567), NULL);

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

  /* definition and creation of myReadyPos1 */
  osThreadDef(myReadyPos1, StartReadyPos1, osPriorityIdle, 0, 128);
  myReadyPos1Handle = osThreadCreate(osThread(myReadyPos1), NULL);

  /* definition and creation of myPointRun */
  osThreadDef(myPointRun, StartPointRun, osPriorityNormal, 0, 128);
  myPointRunHandle = osThreadCreate(osThread(myPointRun), NULL);

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
//	vTaskSuspend(defaultTaskHandle); //测试用暂停，正式记得注释
	double start_m[3];	//巡线起始位置
	double target_m[3]; //巡线结束位置
	
  /* Infinite loop */
  for(;;)
  {
		//底盘标志判断
		if(mode_flag == TYPICAL && point_flag == 0)
		{
			input_motion(move_row, move_pitch, around_row);
		}
		else if(mode_flag == TEST)
		{
			triangle();
			mode_flag = TYPICAL;
		}
		else if(mode_flag == LEFT)
		{
			area_flag = LEFT;
			arm_flag = ARM_NORMAL;
			printf("%c%c%c",SRART_CHAR1, 0x01, END_CHAR);
			osDelay(100);
			printf("%c%c%c",SRART_CHAR2, 0x01, END_CHAR);
			osDelay(100);
			start_left(); //左场启动
//			Mec_Delay(2000);
			vTaskResume(myReadyGetBallHandle); //机器臂启动运行到预备夹球位置
			vTaskResume(myMoveToBallHandle);
			
			//开始沿线走代码
			start_m[0] = 3.85;start_m[1] = 0.78;start_m[2] = 0;	//巡线起始位置
			target_m[0] = 0.74;target_m[1] = 0.78;target_m[2] = 0;	//巡线结束位置
			current_target_m_main[0] = start_m[0];
			current_target_m_main[1] = start_m[1];
			while(1)
			{
				if(move_from_to(start_m, target_m, 0.003, current_target_m_main)) break;
				if(mode_flag == TYPICAL) break;	//手动切出
				if(ball_cnt == 6 && go_get_ball == 0) break; //球取完退出
				osDelay(1);
			}
			if(mode_flag != TYPICAL && ball_cnt < 6) //没收到最后一个球的信息就把最后一个球夹走
			{
				osDelay(10);
				while(arm_mode != READY) osDelay(10);
				osDelay(10);
				arm_mode = BUSY; //此时还没夹到球
				vTaskResume(myGetBallHandle);
				osDelay(10);
				while(arm_mode == BUSY) osDelay(10); //等待夹到球
			}
			mode_flag = TYPICAL;
			vTaskSuspend(myMoveToBallHandle);
		}
		else if(mode_flag == RIGHT)
		{
			area_flag = RIGHT;
			arm_flag = ARM_NORMAL;
			printf("%c%c%c",SRART_CHAR1, 0x00, END_CHAR);
			osDelay(100);
			printf("%c%c%c",SRART_CHAR2, 0x01, END_CHAR);
			osDelay(100);
			start_right(); //右场启动
			vTaskResume(myReadyGetBallHandle); //机器臂启动运行到预备夹球位置
			vTaskResume(myMoveToBallHandle);
			
			start_m[0] = 1.63;start_m[1] = 0.78;start_m[2] = 0;	//巡线起始位置
			target_m[0] = 4.88;target_m[1] = 0.78;target_m[2] = 0;	//巡线结束位置
			current_target_m_main[0] = start_m[0];
			current_target_m_main[1] = start_m[1];
			while(1)
			{
				if(move_from_to(start_m, target_m, 0.003, current_target_m_main)) break;
				if(mode_flag == TYPICAL) break;		//手动切出
				if(ball_cnt == 6 && go_get_ball == 0) break; //球取完退出
				osDelay(1);
			}
			if(mode_flag != TYPICAL && ball_cnt < 6) //没收到最后一个球的信息就把最后一个球夹走
			{
				osDelay(10);
				while(arm_mode != READY) osDelay(10);
				osDelay(10);
				arm_mode = BUSY; //此时还没夹到球
				vTaskResume(myGetBallHandle);
				osDelay(10);
				while(arm_mode == BUSY) osDelay(10); //等待夹到球
			}
			mode_flag = TYPICAL;
			vTaskSuspend(myMoveToBallHandle);
		}
		
		osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMoveToBall */
/**
* @brief Function implementing the myMoveToBall thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMoveToBall */
void StartMoveToBall(void const * argument)
{
  /* USER CODE BEGIN StartMoveToBall */
	vTaskSuspend(myMoveToBallHandle);
	double start_m[3];
	double target_m[3];
	
  /* Infinite loop */
  for(;;)
  {
		if(go_get_ball)
		{
			//挂起主巡线
			vTaskSuspend(defaultTaskHandle);
			//停车
			Mec_Delay(500);
			
			//判断开场方向
			
			//左场
			if(mode_flag == LEFT)
			{
				//前往夹球位置
				target_m[0] = -0.25* ball_pos[ball_cnt-go_get_ball] + 3.74; target_m[1] = 0.78; target_m[2] = 0;		//这里替换成夹球位置
				for(int i=0;i<30;)
				{
					if(move_to(target_m)) i++;
					if(mode_flag == TYPICAL) break;		//手动切出
					osDelay(1);
				}
				Mec_Delay(500);
				
				//夹球
				if(mode_flag == LEFT){
					osDelay(10);
					while(arm_mode != READY) osDelay(10);
					osDelay(10);
					arm_mode = BUSY; //此时还没夹到球
					vTaskResume(myGetBallHandle);
					osDelay(10);
					while(arm_mode == BUSY) osDelay(10); //等待夹到球
				}
				
				//更新主巡线继续位置
				current_target_m_main[0] = -0.25* ball_pos[ball_cnt-go_get_ball] + 3.74;
				current_target_m_main[1] = 0.78;
			}
			
			//右场
			else if(mode_flag == RIGHT)
			{
				target_m[0] = 0.25* ball_pos[ball_cnt-go_get_ball] + 1.88; target_m[1] = 0.78; target_m[2] = 0;		//这里替换成夹球位置
//				if(ball_pos[ball_cnt-go_get_ball] <= 2){
//					target_m[0] = 0.25* ball_pos[ball_cnt-go_get_ball] + 1.80; target_m[1] = 0.78; target_m[2] = 0;		//这里替换成夹球位置
//				}
//				else{
//					target_m[0] = 0.25* ball_pos[ball_cnt-go_get_ball] + 1.85; target_m[1] = 0.78; target_m[2] = 0;		//这里替换成夹球位置
//				}
				for(int i=0;i<30;)
				{
					if(move_to(target_m)) i++;
					if(mode_flag == TYPICAL) break;		//手动切出
					osDelay(1);
				}
				Mec_Delay(500);
				
				//夹球
				if(mode_flag == RIGHT){
					osDelay(10);
					while(arm_mode != READY) osDelay(10);
					osDelay(10);
					arm_mode = BUSY; //此时还没夹到球
					vTaskResume(myGetBallHandle);
					osDelay(10);
					while(arm_mode == BUSY) osDelay(10); //等待夹到球
				}
				
				//更新主巡线继续位置
				current_target_m_main[0] = 0.25* ball_pos[ball_cnt-go_get_ball] + 1.88;
				current_target_m_main[1] = 0.78;
			}
			//恢复主寻线
			if(--go_get_ball == 0)
				vTaskResume(defaultTaskHandle);
		}
    osDelay(1);
  }
  /* USER CODE END StartMoveToBall */
}

/* USER CODE BEGIN Header_StartGetBall */
/**
* @brief Function implementing the myGetBall thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetBall */
void StartGetBall(void const * argument)
{
  /* USER CODE BEGIN StartGetBall */
	
	//测试用代码
//	int l=0;
	
  /* Infinite loop */
  for(;;)
  {
		//测试用代码
//		osDelay(5000);
//		if(l++ == 6) vTaskSuspend(myGetBallHandle);
//		vTaskSuspend(myReadyGetBallHandle);
		
		vTaskSuspend(myGetBallHandle); //暂停任务，等待唤醒 //测试时注释
		
		vTaskSuspend(myReadyGetBallHandle); //挂起机械臂预备位置任务
		
		//这里是控制机械臂夹球的代码
		M5_Pos = -90 * 1638.4; //抓取方向电机
		osDelay(10);
		while(1) //循环控制
		{
			if(fabs(M3508_Pos_Pid[4].Err) <= 10) //到达位置跳出
				break;
			
			osDelay(1);
		}
		
		M6_Pos = 164 * 1297; //大臂电机抓取位置
		M7_Pos = 50 * 1638.4; //小臂电机
		osDelay(10);
		while(1) //循环控制
		{
			if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
				break;
			
			osDelay(1);
		}
		
		//控制气缸夹球
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		osDelay(500);
		
		//抬起机械臂
		//将大小臂抬起
		M6_Pos = 80 * 1297; //大臂电机
		M7_Pos = 100 * 1638.4; //小臂电机
		osDelay(10);
		while(1){
			if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) break; //判断跳出
			osDelay(1);
		}
		
		arm_mode = FREE; //夹完球更改标签，可以开始返回巡航位置
		vTaskResume(myStoreBallHandle); //存球
		
    osDelay(1);
  }
  /* USER CODE END StartGetBall */
}

/* USER CODE BEGIN Header_StartReadyGetBall */
/**
* @brief Function implementing the myReadyGetBall thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadyGetBall */
void StartReadyGetBall(void const * argument)
{
  /* USER CODE BEGIN StartReadyGetBall */
	
	vTaskSuspend(myReadyGetBallHandle); //测试时注释
	
  /* Infinite loop */
  for(;;)
  {
		//移动机械臂
		M6_Pos = 80 * 1297; //大臂电机
		M7_Pos = 50 * 1638.4; //小臂电机
		osDelay(5);
		if(fabs(M3508_Pos_Pid[6].Err) <= 10 && fabs(M3508_Pos_Pid[7].Err) <= 10)
		{
			M5_Pos = -90 * 1638.4; //抓取方向电机
			osDelay(5);
			if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10 && fabs(M3508_Pos_Pid[7].Err) <= 10 && arm_mode == FREE)
				arm_mode = READY; //标志机械臂已准备
		}
		
    osDelay(1);
  }
  /* USER CODE END StartReadyGetBall */
}

/* USER CODE BEGIN Header_StartStoreBall */
/**
* @brief Function implementing the myStoreBall thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStoreBall */
void StartStoreBall(void const * argument)
{
  /* USER CODE BEGIN StartStoreBall */
	uint8_t store_ball_num = 0; //记录存储的球数，以决策球要放在哪
	
  /* Infinite loop */
  for(;;)
  {
		vTaskSuspend(myStoreBallHandle);
		
		//测试程序
//		if(store_ball_num == 1) store_ball_num++;
		
		if(store_ball_num == 0){
			//移动到1号框
			M5_Pos = 144 * 1638.4; //抓取方向电机
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			M6_Pos = 133 * 1297; //大臂电机下降
			M7_Pos = 58 * 1638.4; //小臂电机
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			osDelay(800);
			//控制气缸松开夹子
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			osDelay(700);
			
			//等待后夹子移动到预备位置
			vTaskResume(myReadyGetBallHandle);
			
		}
		else if(store_ball_num == 1){
			//移动到2号框
			M5_Pos = 110 * 1638.4; //抓取方向电机
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			M7_Pos = 25 * 1638.4; //小臂电机下降
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			
			M6_Pos = 148 * 1297; //大臂电机下降
			M7_Pos = 85 * 1638.4; //小臂电机前伸
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			osDelay(300);
			//控制气缸松开夹子
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			osDelay(700);
			
			//等待后夹子移动到预备位置
			vTaskResume(myReadyGetBallHandle);
		}
		else if(store_ball_num == 2){
			//移动到3号框
			M5_Pos = 124 * 1638.4; //抓取方向电机
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			M6_Pos = 112 * 1297; //大臂电机下降
			M7_Pos = 15 * 1638.4; //小臂电机下降
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			osDelay(300);
			//控制气缸松开夹子
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			osDelay(700);
			
			//等待后夹子移动到预备位置
			vTaskResume(myReadyGetBallHandle);
		}
		else if(store_ball_num == 3){
			//移动到4号框
			M5_Pos = 84 * 1638.4; //抓取方向电机
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			M7_Pos = 28 * 1638.4; //小臂电机下降
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			M6_Pos = 117 * 1297; //大臂电机下降
			M7_Pos = 30 * 1638.4; //小臂电机下降
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			osDelay(300);
			//控制气缸松开夹子
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			osDelay(700);
			
			//等待后夹子移动到预备位置
			vTaskResume(myReadyGetBallHandle);
		}
		else if(store_ball_num == 4){
			//移动到5号框
			M5_Pos = -183 * 1638.4; //抓取方向电机
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			
			M6_Pos = 125 * 1297; //大臂电机
			M7_Pos = 30 * 1638.4; //小臂电机
			osDelay(5);
			while(1) //循环控制
			{
				if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				
				osDelay(1);
			}
			
			osDelay(300);
			//控制气缸松开夹子
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			osDelay(700);
			
			//等待后夹子移动到预备位置
			vTaskResume(myReadyGetBallHandle);
		}
		else if(store_ball_num == 5){
			//最后一个球，机械臂直接收起休眠
			vTaskResume(myReadyGetBallHandle);
		}
		store_ball_num++; //存球计数加一
		
    osDelay(1);
  }
  /* USER CODE END StartStoreBall */
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
	
	//测试用代码
//	vTaskSuspend(mySetM567Handle);
	
	RM3508_Set_NowPos(5, 2);
	RM3508_Set_NowPos(6, 0);
	RM3508_Set_NowPos(7, 0);
	
  /* Infinite loop */
  for(;;)
  {
		if(arm_flag  != ARM_NONE){
			RM3508_Set_Pos(M5_Pos, 5); //抓取方向电机
			RM3508_Set_Pos(M6_Pos, 6); //大臂电机
			RM3508_Set_Pos(M7_Pos, 7); //小臂电机
		}
		else{
			RM3508_Set_I(0, 5); //抓取方向电机
			RM3508_Set_I(0, 6); //大臂电机
			RM3508_Set_I(0, 7); //小臂电机
		}
		
    osDelay(1);
  }
  /* USER CODE END StartSetM567 */
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
				if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}		
				osDelay(1);
		 }
		osDelay(300);
		HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,0);//电磁阀打开放球
		osDelay(700);
		if(adjust){
			vTaskResume(myReadyGetBallHandle);//预备位置
			PUT_BALL_1 = 0;
			adjust = 0;
		}
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
		osDelay(5); 
		while(1)
		{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
					break;
				if(STOP_FLAG == 1)
					{
						STOP_FLAG = 0;
						break;
					}		
				osDelay(1);
		}
		osDelay(300);
		HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,0);//电磁阀打开放球
		osDelay(700);
		vTaskResume(myReadyGetBallHandle);//预备位置
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
				if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}		
				osDelay(1);
		 }
		osDelay(300);
		HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//电磁阀打开放球
		osDelay(700);
		vTaskResume(myReadyPos1Handle);//预备位置
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
			//先转方向电机
			M5_Pos = M5_GET1_2 * 1638.4; //抓取方向电机
			osDelay (5);
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 ) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			M6_Pos = M6_GET1_2 * 1297; //大臂电机抓取位置
			M7_Pos = M7_GET1_2 * 1638.4; //小臂电机
			osDelay (5);
			while(1)
			{
					if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
			osDelay(300);
			HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//电磁阀关闭抓球
			osDelay(700);
		  }
			
			//第二个球
			if(BOX == 4)
			{
			//先转方向电机
			M5_Pos = M5_GET2_2 * 1638.4; //抓取方向电机
			osDelay (5);				
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 ) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			M6_Pos = M6_GET2_2 * 1297; //大臂电机抓取位置
			M7_Pos = M7_GET2_2 * 1638.4; //小臂电机
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
			osDelay(300);
			HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//电磁阀关闭抓球
			osDelay(700);
		  }
			
			//第三个球
			if(BOX == 3)
			{
			//先转方向电机
			M5_Pos = M5_GET3_2 * 1638.4; //抓取方向电机
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 ) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
			M7_Pos = M7_GET3_2 * 1638.4; //小臂电机
			osDelay (5);	
			while(1)
			{
					if(  fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
			M6_Pos = M6_GET3_2 * 1297; //大臂电机抓取位置
			osDelay (5);	
			while(1)
			{
					if( fabs(M3508_Pos_Pid[5].Err) <= 10) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			osDelay(300);
			HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//电磁阀关闭抓球
			osDelay(700);
		  }
			
			//第四个球
			if(BOX == 2)
			{
			//先转方向电机
			M5_Pos = M5_GET4_2 * 1638.4; //抓取方向电机
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 ) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			M6_Pos = M6_GET4_2 * 1297; //大臂电机抓取位置
			M7_Pos = M7_GET4_2 * 1638.4; //小臂电机
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
			osDelay(300);
			HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//电磁阀关闭抓球
			osDelay(700);
			
			//收臂, 先收大臂
			M6_Pos = M6_READY * 1297; //大臂电机抓取位置
			osDelay(5);
			while(1)
			{
			if(fabs(M3508_Pos_Pid[5].Err) <= 10)
				break;
			  if(STOP_FLAG == 1)
				{
					break;
					STOP_FLAG = 0;
				}		
				osDelay(1);
		  }
			//再收小臂和方向
			M5_Pos = M5_READY * 1297; //大臂电机抓取位置
			M7_Pos = M7_READY * 1638.4; //小臂电机
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
		  }
			
			//第五个球
			if(BOX == 1)
			{
			//先转方向电机
			M5_Pos = M5_GET5_2 * 1638.4; //抓取方向电机
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 ) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			M6_Pos = 110 * 1297; 
	    M7_Pos = 33 *  1638.4;
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			osDelay (500);
			M6_Pos = M6_GET5_2 * 1297; //大臂电机抓取位置
			M7_Pos = M7_GET5_2 * 1638.4; //小臂电机
			osDelay (5);	
			while(1)
			{
					if( fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			osDelay(300);
			HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//电磁阀关闭抓球
			osDelay(700);
			
			
			M7_Pos = 150 *  1638.4;
			osDelay (5);	
			while(1)
			{
					if( fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
					
			M6_Pos = M6_READY * 1297; //大臂电机抓取位置
			M7_Pos = M7_READY * 1638.4; //小臂电机
			osDelay (5);	
			while(1)
			{
					if( fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //到达位置跳出
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }		
		  }
		  }
		vTaskResume(myReadyGetBallHandle);//预备位置
		GET_BALL_2 = 0;
			BOX--;
			if(BOX == 0) BOX = 5;
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
			PUT_BALL_2 = 0;
			GET_BALL_1 = 0;
			GET_BALL_2 = 0;
			vTaskSuspend(myReadyGetBallHandle);
			vTaskSuspend(myReadyGetBallHandle);
			vTaskResume(myPutBall1Handle);
		}
    if(PUT_BALL_2)
		{
			PUT_BALL_1 = 0;
			GET_BALL_1 = 0;
			GET_BALL_2 = 0;
			vTaskSuspend(myReadyGetBallHandle);
			vTaskSuspend(myReadyPos1Handle);
			vTaskResume(myPutBall2Handle);
		}
    if(GET_BALL_1)
		{
			PUT_BALL_1 = 0;
			PUT_BALL_2 = 0;
			GET_BALL_2 = 0;
			vTaskSuspend(myReadyGetBallHandle);
			vTaskSuspend(myReadyGetBallHandle);
			vTaskResume(myGetBall1Handle);
		}
		if(GET_BALL_2)
		{
			PUT_BALL_1 = 0;
			PUT_BALL_2 = 0;
			GET_BALL_1 = 0;
			vTaskSuspend(myReadyGetBallHandle);
			vTaskSuspend(myReadyGetBallHandle);
			vTaskResume(myGetBall2Handle);
		}
    osDelay(1);
  }
  /* USER CODE END StartJudge */
}

/* USER CODE BEGIN Header_StartReadyPos1 */
/**
* @brief Function implementing the myReadyPos1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadyPos1 */
void StartReadyPos1(void const * argument)
{
  /* USER CODE BEGIN StartReadyPos1 */
	vTaskSuspend(myReadyPos1Handle); //暂停任务，等待唤醒
  /* Infinite loop */
  for(;;)
  {
		//机械臂预备位置
			 M5_Pos = M5_READY1 * 1638.4; //抓取方向电机
       M6_Pos = M6_READY1 * 1297; //大臂电机抓取位置
       M7_Pos = M7_READY1 * 1638.4; //小臂电机
       osDelay(1);
			 vTaskSuspend(myReadyGetBallHandle); //暂停任务，等待唤醒
  }
  /* USER CODE END StartReadyPos1 */
}

/* USER CODE BEGIN Header_StartPointRun */
/**
* @brief Function implementing the myPointRun thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPointRun */
void StartPointRun(void const * argument)
{
  /* USER CODE BEGIN StartPointRun */
  /* Infinite loop */
  for(;;)
  {
		if(area_flag == LEFT)
		{
			switch(point_flag)
			{
				case 1:
					if(Robot_Pos_Move_PID_V2(&Path1_PID_31))
						point_flag = 0;
					break;
				case 2:
					if(Robot_Pos_Move_PID_V2(&Path1_PID_32))
						point_flag = 0;
					break;
				case 3:
					if(Robot_Pos_Move_PID_V2(&Path1_PID_33))
						point_flag = 0;
					break;
			}
		}
		else if(area_flag == RIGHT)
		{
			switch(point_flag)
			{
				case 1:
					if(Robot_Pos_Move_PID_V1(&Path_PID_31))
						point_flag = 0;
					break;
				case 2:
					if(Robot_Pos_Move_PID_V1(&Path_PID_32))
						point_flag = 0;
					break;
				case 3:
					if(Robot_Pos_Move_PID_V1(&Path_PID_33))
						point_flag = 0;
					break;
			}
		}
		
    osDelay(1);
  }
  /* USER CODE END StartPointRun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
