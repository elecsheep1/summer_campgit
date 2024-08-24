#include "arm_control.h"
#include "RM3508.h"
#include "can_bsp.h"
#include "PID.h"

#include "main.h"
#include "cmsis_os.h"

#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"



uint8_t Box = 4;


ARM Arm_Pos = 
{
	.ARM_Pos_5 = 0  /360 * 8192,
	.ARM_Pos_6 = 0  /360 * 8192,
	.ARM_Pos_7 = 0  /360 * 8192,
};
/*********************************************************************************
 *@  name      : Arm_Pos_PID_V1 (ARM* arm)
 *@  function  : 判断是否到点
 *@  input     : 跑点结构体
 *@  output    : 是否跑到点
 *********************************************************************************/
uint8_t Arm_count = 0;
bool Arm_Pos_PID_V1 (ARM* arm)
{
    RM3508_Set_Pos(arm->ARM_Pos_5,5);
    RM3508_Set_Pos(arm->ARM_Pos_6,6);
    RM3508_Set_Pos(arm->ARM_Pos_7,7);
    if (M3508_Pos_Pid[6].Err< arm->Arrive_Error&&M3508_Pos_Pid[7].Err< arm->Arrive_Error&&M3508_Pos_Pid[5].Err< arm->Arrive_Error)
			{
        if(Arm_count++ == 5)
        return true;
        }
			else
        return false;
}

/*********************************************************************************
 *@  name      : Arm_Pos_Init (ARM* arm)
*@  function  : 机械臂返回0位置
 *@  input     : 跑点结构体
 *@  output    : 是否跑到点
 *********************************************************************************/
void Arm_Pos_Init (ARM* arm)
{
	
}


/*********************************************************************************
 *@  name      : Box_Open ()
*@  function  : 机械臂返回0位置
 *@  input     : 跑点结构体
 *@  output    : 是否跑到点
 *********************************************************************************/
//注意door1要常高电平
void Box_Open ()
{
	if(Box == 4)
	{
		HAL_GPIO_WritePin (Box__door1_GPIO_Port,Box__door1_Pin ,0);
		HAL_GPIO_WritePin (Box_Push_GPIO_Port,Box_Push_Pin  ,1);
		//延时后关闭
		osDelay(500);
		HAL_GPIO_WritePin (Box__door1_GPIO_Port,Box__door1_Pin ,1);
	}
	if(Box == 3)
	{
		HAL_GPIO_WritePin (Box__door1_GPIO_Port,Box__door1_Pin ,0);
		HAL_GPIO_WritePin (Box_Push_GPIO_Port,Box_Push_Pin  ,1);
		//延时后关闭
		osDelay(500);
		HAL_GPIO_WritePin (Box__door1_GPIO_Port,Box__door1_Pin ,1);
		HAL_GPIO_WritePin (Box__door2_GPIO_Port,Box__door2_Pin ,0);
		osDelay(500);
		HAL_GPIO_WritePin (Box__door2_GPIO_Port,Box__door2_Pin ,1);
	}
		if(Box == 2)
	{
    HAL_GPIO_WritePin (Box__door1_GPIO_Port,Box__door1_Pin ,0);
		osDelay(500);
		HAL_GPIO_WritePin (Box__door1_GPIO_Port,Box__door1_Pin ,1);
	}
		if(Box == 1)
	{
    HAL_GPIO_WritePin (Box__door1_GPIO_Port,Box__door1_Pin ,0);
		osDelay(500);
		HAL_GPIO_WritePin (Box__door1_GPIO_Port,Box__door1_Pin ,1);
	}
}