#ifndef __mySerial_H
#define __mySerial_H

/*
使用说明
在cubemx中配置htim14，预分频选择42-1(将定时器中断调整至ms级别即可)，记得开启htim14的定时器中断
在定时器中断的中断回调中加上Serial_TIM_Period_Callback(htim);
在主函数while前调用mySerial_Init();
程序中运行一次ArrangeSerialList();解析一次串口数据（已经把这个函数添加到Serial_TIM_Period_Callback中了）
*/

#include "main.h"
#include <stdio.h>

#define SERIAL_PORT &huart5
#define TIM_PORT &htim13
#define TIM2_PORT &htim14
#define SERIAL2_PORT &huart4

#define SRART_CHAR1 0xAA
#define SRART_CHAR2 0xBB
#define END_CHAR 0x55


#define TYPICAL		0
#define TEST	1
#define TEST2	2
#define TEST3 3
#define TEST4	4

#define LEFT	'l'
#define RIGHT	'r'
#define ARM_GET 'U'
#define ARM_SLEEP 'D'
#define ARM_READY 'L'
#define ARM_CATCH 'R'
#define ARM_NONE 'N'
#define ARM_NORMAL 'W'


extern uint8_t area_flag; //场地标志
extern uint8_t point_flag; //跑点标志位

extern uint8_t ball_cnt; //计数视觉看到的紫球数
extern uint8_t ball_pos[6];
extern uint8_t go_get_ball; //取球中断标志
extern double camera_delay;

extern uint8_t run_ball_flag;	//寻球标签
extern uint8_t run_ball_num;	//左右位移计数
extern uint8_t mode_flag;			//底盘模式标签
extern uint8_t arm_flag;			//机械臂模式标签

extern uint8_t aRxBuffer1;			//接收中断缓冲
extern uint8_t Uart1_RxBuff[256];		//接收缓冲
extern uint8_t Uart1_Rx_Cnt;		//接收缓冲计数
extern volatile uint8_t Uart1_RxFlag;

extern uint8_t aRxBuffer2;			//接收中断缓冲
extern uint8_t Uart2_RxBuff[256];		//接收缓冲
extern uint8_t Uart2_Rx_Cnt;		//接收缓冲计数
extern volatile uint8_t Uart2_RxFlag;

//__weak void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Serial_TIM_Period_Callback(TIM_HandleTypeDef *htim);

// 串口重定向
int fputc(int ch, FILE *f);
int fgetc(FILE * f);

void mySerial_Init();
void ArrangeSerialList();

extern int around_row;
extern int around_pitch;
extern int move_row;
extern int move_pitch;

#endif