#include "TFminiPlus.h"
#include "tim.h"
#include "usart.h"
#include "mySerial.h"

uint16_t TF_distance = 0;             // TF_测距数据
uint16_t TF_strength = 0;             // TF_信号强度
uint16_t TF_temperature = 0;          // TF_温度
uint16_t TF_checksum = 0;             // TF_自检
uint8_t TF_flag = 0;              		// TF_测距状态
uint8_t TF_Buff[9] = {0};							// TF_原始数据储存
uint8_t TF_Cnt = 0;												// TF_接收计数
void TFminiPlus_Serial_Callback(UART_HandleTypeDef *huart)
{
	if(huart == SERIAL_PORT)
	{
		TF_Buff[TF_Cnt] = aRxBuffer1;
		
		if(TF_Cnt)
		{
				if(TF_Buff[1] != TF_HEADER) TF_Cnt = 0;	//2、若第二个数不是 0x59，则TF_Cnt清零，下一次重新检测第一个数
				else  TF_Cnt++;																//3、第一和第二个数均满足要求，开始将获得的数，逐位赋给TF_Buff[2~8]
		}
		if(TF_Buff[0] != TF_HEADER) TF_Cnt = 0;
		else if(TF_Cnt==0) TF_Cnt = 1;							//1、若第一个数满足要求0x59，则下一次接收并检测第二个函数

		if (TF_Cnt==9)//获得完9位数后
		{
				TF_Cnt = 0;
				TF_checksum = TF_Buff[0]+TF_Buff[1]+TF_Buff[2]+TF_Buff[3]+TF_Buff[4]+
												TF_Buff[5]+TF_Buff[6]+TF_Buff[7];

				if((TF_checksum % 256) == TF_Buff[8])//TF_Buff[8]是自检位，满足要求后根据公式计算距离等
				{
					TF_distance = TF_Buff[3]<<8 | TF_Buff[2];
					TF_strength = TF_Buff[5]<<8 | TF_Buff[4];
					TF_temperature  = TF_Buff[7]<<8 | TF_Buff[6];
					TF_flag = 1;
				}
		}
		
		HAL_UART_Receive_IT(SERIAL_PORT, &aRxBuffer1, 1);
	}
}