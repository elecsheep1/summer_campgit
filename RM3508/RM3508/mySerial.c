#include "mySerial.h"
#include "usart.h"
#include "tim.h"
#include <string.h>
#include <stdlib.h>
#include "i_Serial.h"

#include "RM3508.h"
#include "Mecanum_wheel.h"
#include "point_control.h"
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
int left_row = 0;
int left_pitch = 0;
int right_row = 0;
int right_pitch = 0;

int row, pitch;
uint8_t nohead_flag = HEAD;
void ArrangeSerialList(){
	
	if(Uart2_RxFlag){
		
		char sign = Uart2_RxBuff[0];// l/r
		if(sign == 'l' || sign == 'r')
		{
			sscanf((char *)Uart2_RxBuff+1,"%d;%d",&row,&pitch);
			switch(sign)
			{
				case 'l':
					left_row = row;
					left_pitch = pitch;
					break;
				case 'r':
					right_row = row;
					right_pitch = pitch;
					break;
			}
		}
		else if(sign == 'n')
		{
			nohead_flag = NOHEAD;
		}
		else if(sign == 'h')
		{
			nohead_flag = HEAD;
		}
		else if(sign == 'q')
		{
			Point_test = 1;
		}
		memset(Uart2_RxBuff,0x00,sizeof(Uart2_RxBuff));//读完清空以防干扰下一组数据
		Uart2_Rx_Cnt = 0;
		Uart2_RxFlag = 0;
//		printf("lr,lp,rr,rp:%d,%d,%d,%d\n", left_row, left_pitch, right_row, right_pitch);
	}
	
if(Uart1_RxFlag){        //视觉串口传递数据
        int ball_receive = 0;
        sscanf((char *)Uart1_RxBuff, "%f %f %f", &Cur_Gyro.X_Pos, &Cur_Gyro.Y_Pos, &Cur_Gyro.Z_Pos);
 
    }
  memset(Uart1_RxBuff,0x00,sizeof(Uart1_RxBuff));//读完清空以防干扰下一组数据
  Uart1_Rx_Cnt = 0;
  Uart1_RxFlag = 0;
}

