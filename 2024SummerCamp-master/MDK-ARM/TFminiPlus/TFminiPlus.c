#include "TFminiPlus.h"
#include "tim.h"
#include "usart.h"
#include "mySerial.h"

uint16_t TF_distance = 0;             // TF_�������
uint16_t TF_strength = 0;             // TF_�ź�ǿ��
uint16_t TF_temperature = 0;          // TF_�¶�
uint16_t TF_checksum = 0;             // TF_�Լ�
uint8_t TF_flag = 0;              		// TF_���״̬
uint8_t TF_Buff[9] = {0};							// TF_ԭʼ���ݴ���
uint8_t TF_Cnt = 0;												// TF_���ռ���
void TFminiPlus_Serial_Callback(UART_HandleTypeDef *huart)
{
	if(huart == SERIAL_PORT)
	{
		TF_Buff[TF_Cnt] = aRxBuffer1;
		
		if(TF_Cnt)
		{
				if(TF_Buff[1] != TF_HEADER) TF_Cnt = 0;	//2�����ڶ��������� 0x59����TF_Cnt���㣬��һ�����¼���һ����
				else  TF_Cnt++;																//3����һ�͵ڶ�����������Ҫ�󣬿�ʼ����õ�������λ����TF_Buff[2~8]
		}
		if(TF_Buff[0] != TF_HEADER) TF_Cnt = 0;
		else if(TF_Cnt==0) TF_Cnt = 1;							//1������һ��������Ҫ��0x59������һ�ν��ղ����ڶ�������

		if (TF_Cnt==9)//�����9λ����
		{
				TF_Cnt = 0;
				TF_checksum = TF_Buff[0]+TF_Buff[1]+TF_Buff[2]+TF_Buff[3]+TF_Buff[4]+
												TF_Buff[5]+TF_Buff[6]+TF_Buff[7];

				if((TF_checksum % 256) == TF_Buff[8])//TF_Buff[8]���Լ�λ������Ҫ�����ݹ�ʽ��������
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