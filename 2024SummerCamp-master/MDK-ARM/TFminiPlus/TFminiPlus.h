#ifndef __TFMINIBUS__
#define __TFMINIBUS__

#include "main.h"

#define TFMINIPLUS_UART &huart5

#define TF_HEADER 0x59

extern uint16_t TF_distance;			// TF_�������
extern uint16_t TF_strength;			// TF_�ź�ǿ��
extern uint16_t TF_temperature;		// TF_�¶�
extern uint16_t TF_checksum;			// TF_�Լ�
extern uint8_t TF_flag;						// TF_���״̬
extern uint8_t TF_Buff[9];				// TF_ԭʼ���ݴ���
extern uint8_t TF_Cnt;						// TF_���ռ���

/**
 * @brief TFminiPlus���ڻص�����
 * @param ���ڱ��
 * @note  ���˺������봮���ж���(Ҫ�����ҵ�mySerial.c��)
 */
void TFminiPlus_Serial_Callback(UART_HandleTypeDef *huart);

#endif