#include "arm_control.h"
#include "RM3508.h"
#include "can_bsp.h"
#include "PID.h"

#include "main.h"

#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"


/*********************************************************************************
 *@  name      : Arm_Pos_PID_V1 (ARM* arm)
 *@  function  : 判断是否到点
 *@  input     : 跑点结构体
 *@  output    : 是否跑到点
 *********************************************************************************/
uint8_t Arm_count = 0;
bool Arm_Pos_PID_V1 (ARM* arm)
{
    RM3508_Set_Pos(arm->A3508_Pos,5);
    RM3508_Set_Pos(arm->A2006_Pos_1,6);
    RM3508_Set_Pos(arm->A2006_Pos_2,7);
    if (M3508_Pos_Pid[6].Err< arm->Arrive_Error&&M3508_Pos_Pid[7].Err< arm->Arrive_Error&&M3508_Pos_Pid[5].Err< arm->Arrive_Error)
			{
        if(Arm_count++ == 5)
        return true;
        }
			else
        return false;
}

