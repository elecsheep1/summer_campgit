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
//	vTaskSuspend(defaultTaskHandle); //��������ͣ����ʽ�ǵ�ע��
	double start_m[3];	//Ѳ����ʼλ��
	double target_m[3]; //Ѳ�߽���λ��
	
  /* Infinite loop */
  for(;;)
  {
		//���̱�־�ж�
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
			start_left(); //������
//			Mec_Delay(2000);
			vTaskResume(myReadyGetBallHandle); //�������������е�Ԥ������λ��
			vTaskResume(myMoveToBallHandle);
			
			//��ʼ�����ߴ���
			start_m[0] = 3.85;start_m[1] = 0.78;start_m[2] = 0;	//Ѳ����ʼλ��
			target_m[0] = 0.74;target_m[1] = 0.78;target_m[2] = 0;	//Ѳ�߽���λ��
			current_target_m_main[0] = start_m[0];
			current_target_m_main[1] = start_m[1];
			while(1)
			{
				if(move_from_to(start_m, target_m, 0.003, current_target_m_main)) break;
				if(mode_flag == TYPICAL) break;	//�ֶ��г�
				if(ball_cnt == 6 && go_get_ball == 0) break; //��ȡ���˳�
				osDelay(1);
			}
			if(mode_flag != TYPICAL && ball_cnt < 6) //û�յ����һ�������Ϣ�Ͱ����һ�������
			{
				osDelay(10);
				while(arm_mode != READY) osDelay(10);
				osDelay(10);
				arm_mode = BUSY; //��ʱ��û�е���
				vTaskResume(myGetBallHandle);
				osDelay(10);
				while(arm_mode == BUSY) osDelay(10); //�ȴ��е���
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
			start_right(); //�ҳ�����
			vTaskResume(myReadyGetBallHandle); //�������������е�Ԥ������λ��
			vTaskResume(myMoveToBallHandle);
			
			start_m[0] = 1.63;start_m[1] = 0.78;start_m[2] = 0;	//Ѳ����ʼλ��
			target_m[0] = 4.88;target_m[1] = 0.78;target_m[2] = 0;	//Ѳ�߽���λ��
			current_target_m_main[0] = start_m[0];
			current_target_m_main[1] = start_m[1];
			while(1)
			{
				if(move_from_to(start_m, target_m, 0.003, current_target_m_main)) break;
				if(mode_flag == TYPICAL) break;		//�ֶ��г�
				if(ball_cnt == 6 && go_get_ball == 0) break; //��ȡ���˳�
				osDelay(1);
			}
			if(mode_flag != TYPICAL && ball_cnt < 6) //û�յ����һ�������Ϣ�Ͱ����һ�������
			{
				osDelay(10);
				while(arm_mode != READY) osDelay(10);
				osDelay(10);
				arm_mode = BUSY; //��ʱ��û�е���
				vTaskResume(myGetBallHandle);
				osDelay(10);
				while(arm_mode == BUSY) osDelay(10); //�ȴ��е���
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
			//������Ѳ��
			vTaskSuspend(defaultTaskHandle);
			//ͣ��
			Mec_Delay(500);
			
			//�жϿ�������
			
			//��
			if(mode_flag == LEFT)
			{
				//ǰ������λ��
				target_m[0] = -0.25* ball_pos[ball_cnt-go_get_ball] + 3.74; target_m[1] = 0.78; target_m[2] = 0;		//�����滻�ɼ���λ��
				for(int i=0;i<30;)
				{
					if(move_to(target_m)) i++;
					if(mode_flag == TYPICAL) break;		//�ֶ��г�
					osDelay(1);
				}
				Mec_Delay(500);
				
				//����
				if(mode_flag == LEFT){
					osDelay(10);
					while(arm_mode != READY) osDelay(10);
					osDelay(10);
					arm_mode = BUSY; //��ʱ��û�е���
					vTaskResume(myGetBallHandle);
					osDelay(10);
					while(arm_mode == BUSY) osDelay(10); //�ȴ��е���
				}
				
				//������Ѳ�߼���λ��
				current_target_m_main[0] = -0.25* ball_pos[ball_cnt-go_get_ball] + 3.74;
				current_target_m_main[1] = 0.78;
			}
			
			//�ҳ�
			else if(mode_flag == RIGHT)
			{
				target_m[0] = 0.25* ball_pos[ball_cnt-go_get_ball] + 1.88; target_m[1] = 0.78; target_m[2] = 0;		//�����滻�ɼ���λ��
//				if(ball_pos[ball_cnt-go_get_ball] <= 2){
//					target_m[0] = 0.25* ball_pos[ball_cnt-go_get_ball] + 1.80; target_m[1] = 0.78; target_m[2] = 0;		//�����滻�ɼ���λ��
//				}
//				else{
//					target_m[0] = 0.25* ball_pos[ball_cnt-go_get_ball] + 1.85; target_m[1] = 0.78; target_m[2] = 0;		//�����滻�ɼ���λ��
//				}
				for(int i=0;i<30;)
				{
					if(move_to(target_m)) i++;
					if(mode_flag == TYPICAL) break;		//�ֶ��г�
					osDelay(1);
				}
				Mec_Delay(500);
				
				//����
				if(mode_flag == RIGHT){
					osDelay(10);
					while(arm_mode != READY) osDelay(10);
					osDelay(10);
					arm_mode = BUSY; //��ʱ��û�е���
					vTaskResume(myGetBallHandle);
					osDelay(10);
					while(arm_mode == BUSY) osDelay(10); //�ȴ��е���
				}
				
				//������Ѳ�߼���λ��
				current_target_m_main[0] = 0.25* ball_pos[ball_cnt-go_get_ball] + 1.88;
				current_target_m_main[1] = 0.78;
			}
			//�ָ���Ѱ��
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
	
	//�����ô���
//	int l=0;
	
  /* Infinite loop */
  for(;;)
  {
		//�����ô���
//		osDelay(5000);
//		if(l++ == 6) vTaskSuspend(myGetBallHandle);
//		vTaskSuspend(myReadyGetBallHandle);
		
		vTaskSuspend(myGetBallHandle); //��ͣ���񣬵ȴ����� //����ʱע��
		
		vTaskSuspend(myReadyGetBallHandle); //�����е��Ԥ��λ������
		
		//�����ǿ��ƻ�е�ۼ���Ĵ���
		M5_Pos = -90 * 1638.4; //ץȡ������
		osDelay(10);
		while(1) //ѭ������
		{
			if(fabs(M3508_Pos_Pid[4].Err) <= 10) //����λ������
				break;
			
			osDelay(1);
		}
		
		M6_Pos = 164 * 1297; //��۵��ץȡλ��
		M7_Pos = 50 * 1638.4; //С�۵��
		osDelay(10);
		while(1) //ѭ������
		{
			if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
				break;
			
			osDelay(1);
		}
		
		//�������׼���
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		osDelay(500);
		
		//̧���е��
		//����С��̧��
		M6_Pos = 80 * 1297; //��۵��
		M7_Pos = 100 * 1638.4; //С�۵��
		osDelay(10);
		while(1){
			if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) break; //�ж�����
			osDelay(1);
		}
		
		arm_mode = FREE; //��������ı�ǩ�����Կ�ʼ����Ѳ��λ��
		vTaskResume(myStoreBallHandle); //����
		
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
	
	vTaskSuspend(myReadyGetBallHandle); //����ʱע��
	
  /* Infinite loop */
  for(;;)
  {
		//�ƶ���е��
		M6_Pos = 80 * 1297; //��۵��
		M7_Pos = 50 * 1638.4; //С�۵��
		osDelay(5);
		if(fabs(M3508_Pos_Pid[6].Err) <= 10 && fabs(M3508_Pos_Pid[7].Err) <= 10)
		{
			M5_Pos = -90 * 1638.4; //ץȡ������
			osDelay(5);
			if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10 && fabs(M3508_Pos_Pid[7].Err) <= 10 && arm_mode == FREE)
				arm_mode = READY; //��־��е����׼��
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
	uint8_t store_ball_num = 0; //��¼�洢���������Ծ�����Ҫ������
	
  /* Infinite loop */
  for(;;)
  {
		vTaskSuspend(myStoreBallHandle);
		
		//���Գ���
//		if(store_ball_num == 1) store_ball_num++;
		
		if(store_ball_num == 0){
			//�ƶ���1�ſ�
			M5_Pos = 144 * 1638.4; //ץȡ������
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			M6_Pos = 133 * 1297; //��۵���½�
			M7_Pos = 58 * 1638.4; //С�۵��
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			osDelay(800);
			//���������ɿ�����
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			osDelay(700);
			
			//�ȴ�������ƶ���Ԥ��λ��
			vTaskResume(myReadyGetBallHandle);
			
		}
		else if(store_ball_num == 1){
			//�ƶ���2�ſ�
			M5_Pos = 110 * 1638.4; //ץȡ������
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			M7_Pos = 25 * 1638.4; //С�۵���½�
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			
			M6_Pos = 148 * 1297; //��۵���½�
			M7_Pos = 85 * 1638.4; //С�۵��ǰ��
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			osDelay(300);
			//���������ɿ�����
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			osDelay(700);
			
			//�ȴ�������ƶ���Ԥ��λ��
			vTaskResume(myReadyGetBallHandle);
		}
		else if(store_ball_num == 2){
			//�ƶ���3�ſ�
			M5_Pos = 124 * 1638.4; //ץȡ������
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			M6_Pos = 112 * 1297; //��۵���½�
			M7_Pos = 15 * 1638.4; //С�۵���½�
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			osDelay(300);
			//���������ɿ�����
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			osDelay(700);
			
			//�ȴ�������ƶ���Ԥ��λ��
			vTaskResume(myReadyGetBallHandle);
		}
		else if(store_ball_num == 3){
			//�ƶ���4�ſ�
			M5_Pos = 84 * 1638.4; //ץȡ������
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			M7_Pos = 28 * 1638.4; //С�۵���½�
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			M6_Pos = 117 * 1297; //��۵���½�
			M7_Pos = 30 * 1638.4; //С�۵���½�
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			osDelay(300);
			//���������ɿ�����
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			osDelay(700);
			
			//�ȴ�������ƶ���Ԥ��λ��
			vTaskResume(myReadyGetBallHandle);
		}
		else if(store_ball_num == 4){
			//�ƶ���5�ſ�
			M5_Pos = -183 * 1638.4; //ץȡ������
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			
			M6_Pos = 125 * 1297; //��۵��
			M7_Pos = 30 * 1638.4; //С�۵��
			osDelay(5);
			while(1) //ѭ������
			{
				if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
					break;
				
				osDelay(1);
			}
			
			osDelay(300);
			//���������ɿ�����
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			osDelay(700);
			
			//�ȴ�������ƶ���Ԥ��λ��
			vTaskResume(myReadyGetBallHandle);
		}
		else if(store_ball_num == 5){
			//���һ���򣬻�е��ֱ����������
			vTaskResume(myReadyGetBallHandle);
		}
		store_ball_num++; //���������һ
		
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
	
	//�����ô���
//	vTaskSuspend(mySetM567Handle);
	
	RM3508_Set_NowPos(5, 2);
	RM3508_Set_NowPos(6, 0);
	RM3508_Set_NowPos(7, 0);
	
  /* Infinite loop */
  for(;;)
  {
		if(arm_flag  != ARM_NONE){
			RM3508_Set_Pos(M5_Pos, 5); //ץȡ������
			RM3508_Set_Pos(M6_Pos, 6); //��۵��
			RM3508_Set_Pos(M7_Pos, 7); //С�۵��
		}
		else{
			RM3508_Set_I(0, 5); //ץȡ������
			RM3508_Set_I(0, 6); //��۵��
			RM3508_Set_I(0, 7); //С�۵��
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
		vTaskSuspend(myPutBall1Handle); //��ͣ���񣬵ȴ�����
		//��е�۷���
		M5_Pos = -90 * 1638.4; //ץȡ������
    M6_Pos = M6_PUT_1 * 1297; //��۵��ץȡλ��
    M7_Pos = M7_PUT_1 * 1638.4; //С�۵��
		osDelay(5);
		while(1)
		{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
					break;
				if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}		
				osDelay(1);
		 }
		osDelay(300);
		HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,0);//��ŷ��򿪷���
		osDelay(700);
		if(adjust){
			vTaskResume(myReadyGetBallHandle);//Ԥ��λ��
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
    vTaskSuspend(myPutBall2Handle); //��ͣ���񣬵ȴ�����
		//��е�۷���
		M5_Pos = M5_PUT_2 * 1638.4; //ץȡ������
    M6_Pos = M6_PUT_2 * 1297; //��۵��ץȡλ��
    M7_Pos = M7_PUT_2 * 1638.4; //С�۵��
		osDelay(5); 
		while(1)
		{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
					break;
				if(STOP_FLAG == 1)
					{
						STOP_FLAG = 0;
						break;
					}		
				osDelay(1);
		}
		osDelay(300);
		HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,0);//��ŷ��򿪷���
		osDelay(700);
		vTaskResume(myReadyGetBallHandle);//Ԥ��λ��
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
    vTaskSuspend(myGetBall1Handle); //��ͣ���񣬵ȴ�����
		//��е�۷���
		M5_Pos = -90 * 1638.4; //ץȡ������
    M6_Pos = M6_GET_1 * 1297; //��۵��ץȡλ��
    M7_Pos = M7_GET_1 * 1638.4; //С�۵��
		osDelay(5);
		while(1)
		{
				if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
					break;
				if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}		
				osDelay(1);
		 }
		osDelay(300);
		HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//��ŷ��򿪷���
		osDelay(700);
		vTaskResume(myReadyPos1Handle);//Ԥ��λ��
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
    vTaskSuspend(myGetBall2Handle); //��ͣ���񣬵ȴ�����
		//��е�۷���
			//��һ����
			if(BOX == 5)
			{
			//��ת������
			M5_Pos = M5_GET1_2 * 1638.4; //ץȡ������
			osDelay (5);
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 ) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			M6_Pos = M6_GET1_2 * 1297; //��۵��ץȡλ��
			M7_Pos = M7_GET1_2 * 1638.4; //С�۵��
			osDelay (5);
			while(1)
			{
					if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
			osDelay(300);
			HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//��ŷ��ر�ץ��
			osDelay(700);
		  }
			
			//�ڶ�����
			if(BOX == 4)
			{
			//��ת������
			M5_Pos = M5_GET2_2 * 1638.4; //ץȡ������
			osDelay (5);				
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 ) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			M6_Pos = M6_GET2_2 * 1297; //��۵��ץȡλ��
			M7_Pos = M7_GET2_2 * 1638.4; //С�۵��
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
			osDelay(300);
			HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//��ŷ��ر�ץ��
			osDelay(700);
		  }
			
			//��������
			if(BOX == 3)
			{
			//��ת������
			M5_Pos = M5_GET3_2 * 1638.4; //ץȡ������
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 ) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
			M7_Pos = M7_GET3_2 * 1638.4; //С�۵��
			osDelay (5);	
			while(1)
			{
					if(  fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
			M6_Pos = M6_GET3_2 * 1297; //��۵��ץȡλ��
			osDelay (5);	
			while(1)
			{
					if( fabs(M3508_Pos_Pid[5].Err) <= 10) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			osDelay(300);
			HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//��ŷ��ر�ץ��
			osDelay(700);
		  }
			
			//���ĸ���
			if(BOX == 2)
			{
			//��ת������
			M5_Pos = M5_GET4_2 * 1638.4; //ץȡ������
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 ) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			M6_Pos = M6_GET4_2 * 1297; //��۵��ץȡλ��
			M7_Pos = M7_GET4_2 * 1638.4; //С�۵��
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
			osDelay(300);
			HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//��ŷ��ر�ץ��
			osDelay(700);
			
			//�ձ�, ���մ��
			M6_Pos = M6_READY * 1297; //��۵��ץȡλ��
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
			//����С�ۺͷ���
			M5_Pos = M5_READY * 1297; //��۵��ץȡλ��
			M7_Pos = M7_READY * 1638.4; //С�۵��
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			
		  }
			
			//�������
			if(BOX == 1)
			{
			//��ת������
			M5_Pos = M5_GET5_2 * 1638.4; //ץȡ������
			osDelay (5);	
			while(1)
			{
					if(fabs(M3508_Pos_Pid[4].Err) <= 10 ) //����λ������
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
					if(fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			osDelay (500);
			M6_Pos = M6_GET5_2 * 1297; //��۵��ץȡλ��
			M7_Pos = M7_GET5_2 * 1638.4; //С�۵��
			osDelay (5);	
			while(1)
			{
					if( fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
		  }
			
			osDelay(300);
			HAL_GPIO_WritePin (GPIOB ,GPIO_PIN_5 ,1);//��ŷ��ر�ץ��
			osDelay(700);
			
			
			M7_Pos = 150 *  1638.4;
			osDelay (5);	
			while(1)
			{
					if( fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
						break;
					if(STOP_FLAG == 1)
					{
						break;
						STOP_FLAG = 0;
					}						
					osDelay(1);
					
			M6_Pos = M6_READY * 1297; //��۵��ץȡλ��
			M7_Pos = M7_READY * 1638.4; //С�۵��
			osDelay (5);	
			while(1)
			{
					if( fabs(M3508_Pos_Pid[5].Err) <= 10 && fabs(M3508_Pos_Pid[6].Err) <= 10) //����λ������
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
		vTaskResume(myReadyGetBallHandle);//Ԥ��λ��
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
	vTaskSuspend(myReadyPos1Handle); //��ͣ���񣬵ȴ�����
  /* Infinite loop */
  for(;;)
  {
		//��е��Ԥ��λ��
			 M5_Pos = M5_READY1 * 1638.4; //ץȡ������
       M6_Pos = M6_READY1 * 1297; //��۵��ץȡλ��
       M7_Pos = M7_READY1 * 1638.4; //С�۵��
       osDelay(1);
			 vTaskSuspend(myReadyGetBallHandle); //��ͣ���񣬵ȴ�����
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
