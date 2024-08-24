#include "i_Serial.h"
#include "usart.h"
#include "tim.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


#include "RM3508.h"
#include "Mecanum_wheel.h"
#include "point_control.h"
//#include "TFminiPlus.h"


uint8_t Uart1_RxBuff1[lengthofdata];

void i_Serial_Init()
{
HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Uart1_RxBuff1, lengthofdata);
}

void HAL_UARTEx_RxEventCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == huart5.Instance)
		{
			sscanf((char *)Uart1_RxBuff1, "%f %f %f", &Cur_Gyro.X_Pos, &Cur_Gyro.Y_Pos, &Cur_Gyro.Z_Pos);
			memset(Uart1_RxBuff1,0x00,sizeof(Uart1_RxBuff1));//读完清空以防干扰下一组数据
			
		}
}