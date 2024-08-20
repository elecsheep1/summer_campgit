#include "DR_rm3508.h"
#include "RM3508.h"
#include "can_bsp.h"


#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"


#include "main.h"
#include "can.h"
//#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

/**************************调试用变量***********************/
float Debug_speed_1 = 0;
float Debug_Pos_1 = 0;

float Debug_speed_2 = 0;
float Debug_Pos_2 = 0;

float Debug_speed_3 = 0;
float Debug_Pos_3 = 0;

float Debug_speed_4 = 0;
float Debug_Pos_4 = 0;
/**************************速度/角度变量***********************/
//注意速度设置的范围是-15000~15000
float speed_1 = 0;
float Pos_1 = 0;

float speed_2 = 0;
float Pos_2 = 0;

float speed_3 = 0;
float Pos_3 = 0;

float speed_4 = 0;
float Pos_4 = 0;

float speed_5 = 0;
float Pos_5 = 0;
/**************************模式变量***********************/
uint8_t mode = 0;





/**************************串口重定向***********************/
//int fputc(int c,FILE  *stream)
//{
//	uint8_t ch[1]={c};
//	HAL_UART_Transmit (&huart5 ,ch,1,0xFFFF);
//	return c;
//}




/**************************CAN中断函数***********************/
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
								Debug_speed_1 = RM3508_Get_Speed(1);
								Debug_speed_2 = RM3508_Get_Speed(2);
								Debug_speed_3 = RM3508_Get_Speed(3);
								Debug_speed_4 = RM3508_Get_Speed(4);
								Debug_Pos_1 = RM3508_Get_Pos(1);
								Debug_Pos_2 = RM3508_Get_Pos(2);
								Debug_Pos_3 = RM3508_Get_Pos(3);
								Debug_Pos_4 = RM3508_Get_Pos(4);
//								printf("speed:%d\n",speed);
								
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

/**************************电机测试函数***********************/
void RM3508_text()
{
		if(mode == 0){
    RM3508_Set_Speed(speed_1,1);
    RM3508_Set_Speed(speed_2,2);
    RM3508_Set_Speed(speed_3,3);
		RM3508_Set_Speed(speed_4,4);	
		RM3508_Set_Speed(speed_5,5);	
		}
		if(mode == 1){
			RM3508_Set_Pos(Pos_1,1);
			RM3508_Set_Pos(Pos_2,2);	
			RM3508_Set_Pos(Pos_3,3);
     	RM3508_Set_Pos(Pos_4,4);
      RM3508_Set_Pos(Pos_5,5);				
		}
}


/**************************运动函数***********************/
void Move(float speed1,float speed2,float speed3,float speed4)
{
	RM3508_Set_Speed(speed1,1);
	RM3508_Set_Speed(speed2,2);
	RM3508_Set_Speed(speed3,3);
	RM3508_Set_Speed(speed4,4);
}




