#ifndef MYCAN_INIT_H
#define MYCAN_INIT_H

#include "main.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

/* PIDµ÷ÊÔ */
extern uint8_t test_mode;
extern uint32_t test_speed;
extern uint32_t test_pos;

#endif