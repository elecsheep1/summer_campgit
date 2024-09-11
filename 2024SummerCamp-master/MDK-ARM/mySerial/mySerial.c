#include "mySerial.h"
#include "usart.h"
#include "tim.h"
#include <string.h>
#include <stdlib.h>

#include "RM3508.h"
#include "Mecanum_wheel.h"
#include "run.h"
#include "arm_control.h"
#include "point_control.h"
#include "cmsis_os.h"
//#include "TFminiPlus.h"

/* 初始化 */
void mySerial_Init()
{
	HAL_TIM_Base_Stop_IT(TIM_PORT);
	__HAL_TIM_SET_COUNTER(TIM_PORT, 0);
	HAL_TIM_Base_Stop_IT(TIM2_PORT);
	__HAL_TIM_SET_COUNTER(TIM2_PORT, 0);
	HAL_UART_Receive_IT(SERIAL_PORT, (uint8_t *)&aRxBuffer1, 1);   //开启UART接收
	HAL_UART_Receive_IT(SERIAL2_PORT, (uint8_t *)&aRxBuffer2, 1);
}

/*  串口接收  */
uint8_t aRxBuffer1;			//接收中断缓冲
uint8_t Uart1_RxBuff[256];		//接收缓冲
uint8_t Uart1_Rx_Cnt = 0;		//接收缓冲计数
volatile uint8_t Uart1_RxFlag = 0;

uint8_t aRxBuffer2;			//接收中断缓冲
uint8_t Uart2_RxBuff[256];		//接收缓冲
uint8_t Uart2_Rx_Cnt = 0;		//接收缓冲计数
volatile uint8_t Uart2_RxFlag = 0;
//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	
	if(huart == SERIAL_PORT)
	{
		Uart1_RxBuff[Uart1_Rx_Cnt++] = aRxBuffer1;   //接收数据转存
		HAL_TIM_Base_Start_IT(TIM_PORT);
		__HAL_TIM_SET_COUNTER(TIM_PORT, 0);
		
		if(aRxBuffer1 == END_CHAR)
		{
			Uart1_RxFlag = 1;
			ArrangeSerialList();
		}
		
		HAL_UART_Receive_IT(SERIAL_PORT, &aRxBuffer1, 1);   //再开启接收中断
	}
	
	if(huart == SERIAL2_PORT)
	{
		Uart2_RxBuff[Uart2_Rx_Cnt++] = aRxBuffer2;   //接收数据转存
		HAL_TIM_Base_Start_IT(TIM2_PORT);
		__HAL_TIM_SET_COUNTER(TIM2_PORT, 0);
		HAL_UART_Receive_IT(SERIAL2_PORT, &aRxBuffer2, 1);   //再开启接收中断
	}
	
//	TFminiPlus_Serial_Callback(huart);
		
}

/* 串口接收计时中断（用以判断传输完毕） */
void Serial_TIM_Period_Callback(TIM_HandleTypeDef *htim)
{
	if(htim == TIM_PORT){
		Uart1_RxFlag = 1;
		HAL_TIM_Base_Stop_IT(TIM_PORT);
		__HAL_TIM_SET_COUNTER(TIM_PORT, 0);
		
		ArrangeSerialList();
	}
	
	if(htim == TIM2_PORT){
		Uart2_RxFlag = 1;
		HAL_TIM_Base_Stop_IT(TIM2_PORT);
		__HAL_TIM_SET_COUNTER(TIM2_PORT, 0);
		
		ArrangeSerialList();
	}
}

//// mySerial.h中未启用
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	Serial_TIM_Period_Callback(htim);
//}


/* 串口重定向 */
int fputc(int ch, FILE *f){
 uint8_t temp[1] = {ch};
 HAL_UART_Transmit(SERIAL_PORT, temp, 1, 0xffff);
return ch;
}

int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(SERIAL_PORT,&ch, 1, 0xffff);
  return ch;
}

/* 数据处理 */
//摇杆数据&控制数据
int around_row = 0;
int around_pitch = 0;
int move_row = 0;
int move_pitch = 0;
int row, pitch;
uint8_t mode_flag = TYPICAL;
uint8_t arm_flag = ARM_NONE;
uint8_t run_ball_flag = 0;
uint8_t run_ball_num = 0;
//视觉返回数据
uint8_t ball_cnt = 0; //计数视觉看到的紫球数
uint8_t ball_pos[6] = {0};
uint8_t go_get_ball = 0; //取球中断标志
double camera_delay = 0;
uint8_t area_flag; //场地标志
uint8_t point_flag; //跑点标志位
int DEBUG = 0;
void ArrangeSerialList(){
	
	//视觉串口传递数据
	if(Uart1_RxFlag){
		int ball_receive = 0;
		if(Uart1_RxBuff[0] == SRART_CHAR1) //判断是否为位置帧
		{
			sscanf((char *)Uart1_RxBuff+1, "%lf %lf %lf", &current_meter[0], &current_meter[1], &current_meter[2]);
			DEBUG++;
		}
		else if(Uart1_RxBuff[0] == SRART_CHAR2) //传输的是视觉找球帧
		{
			ball_receive = Uart1_RxBuff[1];
			if(ball_receive != 0 && ball_cnt < 6) //视觉发现球
			{
				if(ball_cnt == 0) //未发现过球
				{
//					if(mode_flag ==LEFT){
						ball_pos[ball_cnt++] = ball_receive;
//					}
//					else if(mode_flag ==RIGHT){
//						ball_pos[5 - ball_cnt++] = ball_receive;
//					}
					go_get_ball += 1;
				}
				else //记录过球
				{
//					if(mode_flag == LEFT){
						if(ball_pos[ball_cnt-1] != ball_receive) //发现的是新球
						{
							ball_pos[ball_cnt++] = ball_receive;
							go_get_ball += 1;
						}
//					}
//					else if(mode_flag == RIGHT){
//						
//						if(ball_pos[5 - (ball_cnt-1)] != ball_receive) //发现的是新球
//						{
//							ball_pos[5 - ball_cnt++] = ball_receive;
//							go_get_ball += 1;
//						}
//					}
				}
			}
		}
		memset(Uart1_RxBuff,0x00,sizeof(Uart1_RxBuff));//读完清空以防干扰下一组数据
		Uart1_Rx_Cnt = 0;
		Uart1_RxFlag = 0;
   }
	
	 //wifi模块传递数据
	if(Uart2_RxFlag){
		char sign = Uart2_RxBuff[0];	// 数据标志位：	l:旋转摇杆 r:平移摇杆 x:手柄控制器 k:键盘
		if(sign == 'l' || sign == 'r')
		{
			sscanf((char *)Uart2_RxBuff+1,"%d;%d",&row,&pitch);
			switch(sign)
			{
				case 'l':
					around_row = row;
					around_pitch = pitch;
					break;
				case 'r':
					move_row = row;
					move_pitch = pitch;
					break;
			}
		}
		else if(sign == 'x')
		{
			switch(Uart2_RxBuff[1])
			{
				case 'A':
					mode_flag = TYPICAL;
					point_flag = 0;
					break;
				case 'B':
					arm_flag = ARM_NONE;
					break;
				case 'X':
					adjust = 1;
					break;
				case 'Y':
					//未定义
					break;
				case '-':
					printf("%c%c%c",SRART_CHAR1, 0x01, END_CHAR);
					area_flag = LEFT;
					break;
				case '+':
					printf("%c%c%c",SRART_CHAR1, 0x00, END_CHAR);
					area_flag = RIGHT;
					break;
				case '<':
					arm_flag = ARM_NORMAL;
					mode_flag = LEFT; //左场出发
					break;
				case '>':
					arm_flag = ARM_NORMAL;
					mode_flag = RIGHT; //右场出发
					break;
				case 'U':
					arm_flag = ARM_NORMAL;
					PUT_BALL_2 = 1; //三区放球
					break;
				case 'D':
					arm_flag = ARM_NORMAL;
					PUT_BALL_1 = 1; //二区放球
					break;
				case 'L':
					arm_flag = ARM_NORMAL;
					GET_BALL_1 = 1; //二区取球
					break;
				case 'R':
					arm_flag = ARM_NORMAL;
					GET_BALL_2 = 1; //背上取球
					break;
			}
		}
		else if(sign == 'k')
		{
			switch(Uart2_RxBuff[1])
			{
//				case '<':
//					run_ball_flag = LEFT;
//					run_ball_num = Uart2_RxBuff[2] - '0';
//					mode_flag = TEST4;
//					break;
//				case '>':
//					run_ball_flag = RIGHT;
//					run_ball_num = Uart2_RxBuff[2] - '0';
//					mode_flag = TEST4;
//					break;
				case '1':
					point_flag = 1;
					printf("%c%c%c",SRART_CHAR2, 0x09, END_CHAR);
					break;
				case '2':
					point_flag = 2;
					printf("%c%c%c",SRART_CHAR2, 0x09, END_CHAR);
					break;
				case '3':
					point_flag = 3;
					printf("%c%c%c",SRART_CHAR2, 0x09, END_CHAR);
					break;
				case 'Q':
					point_flag = 4;
					break;
				case 'W':
					point_flag = 5;
					break;
				case 'E':
					point_flag = 6;
					break;
				case 'R':
					point_flag = 7;
					break;
				case 'T':
					point_flag = 8;
					break;
				case 'Y':
					point_flag = 9;
					break;
				case 'A':
					point_flag = 10;
					break;
				case 'S':
					point_flag = 11;
					break;
				case 'D':
					point_flag = 12;
					break;
				case 'F':
					point_flag = 13;
					break;
				case 'G':
					point_flag = 14;
					break;
				case 'H':
					point_flag = 15;
					break;
			}
		}
		memset(Uart2_RxBuff,0x00,sizeof(Uart2_RxBuff));//读完清空以防干扰下一组数据
		Uart2_Rx_Cnt = 0;
		Uart2_RxFlag = 0;
//		printf("lr,lp,rr,rp:%d,%d,%d,%d\n", left_row, left_pitch, right_row, right_pitch);
	}
		
}