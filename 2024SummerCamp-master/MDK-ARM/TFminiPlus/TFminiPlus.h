#ifndef __TFMINIBUS__
#define __TFMINIBUS__

#include "main.h"

#define TFMINIPLUS_UART &huart5

#define TF_HEADER 0x59

extern uint16_t TF_distance;			// TF_测距数据
extern uint16_t TF_strength;			// TF_信号强度
extern uint16_t TF_temperature;		// TF_温度
extern uint16_t TF_checksum;			// TF_自检
extern uint8_t TF_flag;						// TF_测距状态
extern uint8_t TF_Buff[9];				// TF_原始数据储存
extern uint8_t TF_Cnt;						// TF_接收计数

/**
 * @brief TFminiPlus串口回调函数
 * @param 串口编号
 * @note  将此函数加入串口中断中(要更改我的mySerial.c库)
 */
void TFminiPlus_Serial_Callback(UART_HandleTypeDef *huart);

#endif