#include "mycan_init.h"
#include "can.h"
#include "RM3508.h"
#include "mySerial.h"

uint8_t test_mode = 0;
uint32_t test_speed = 0;
uint32_t test_pos = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
        if(hcan->Instance == hcan1.Instance)
        {
                CAN_RxHeaderTypeDef RxMessage;
                uint8_t RxData[8] = {0};
        
                HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage, RxData);        

                if(RxMessage.IDE==CAN_ID_STD)
                {
                        if(RxMessage.StdId>=0x201&&RxMessage.StdId<=0x208)
                        {
                                RM3508_Get_Feedback(RxMessage.StdId, RxData);
                        }
                }
                /*自己写解析函数*/
        }
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
        if(hcan->Instance == hcan2.Instance)
        {
                CAN_RxHeaderTypeDef RxMessage;
                uint8_t RxData[8] = {0};
        
                HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxMessage, RxData);        

                if(RxMessage.IDE==CAN_ID_STD)
                {
                        if(RxMessage.StdId>=0x201&&RxMessage.StdId<=0x208)
                        {
                                RM3508_Get_Feedback(RxMessage.StdId, RxData);
													
                        }
                }
                /*自己写解析函数*/
        }
}