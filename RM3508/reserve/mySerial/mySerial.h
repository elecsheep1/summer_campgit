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

#define HEAD		0
#define NOHEAD	1

extern uint8_t nohead_flag;			//无头模式标签

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

extern int left_row;
extern int left_pitch;
extern int right_row;
extern int right_pitch;

#endif