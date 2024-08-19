#ifndef __POINT_CONTROL_H
#define __POINT_CONTROL_H

#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"


//底盘位置环PID结构体
#define Control_Point 4//从初始位置到目标位置总共的节点个数
typedef struct
{
	uint16_t Point_Order;//当前点的编号
	uint16_t Total_Order;//所有点的个数
	//目标位置xyr
	int16_t Target_X[Control_Point];
    int16_t Target_Y[Control_Point];
    int16_t Target_R[Control_Point];
    //目标位置死区
    float Arrive_Err[Control_Point];
    //PID相关参数
    float X_Kp[Control_Point];
    float X_Ki[Control_Point];
    float X_Kd[Control_Point];

    float Y_Kp[Control_Point];
    float Y_Ki[Control_Point];
    float Y_Kd[Control_Point];

    float R_Kp[Control_Point];
    float R_Ki[Control_Point];
    float R_Kd[Control_Point];
    //输出限幅
    float X_Max[Control_Point];
    float Y_Max[Control_Point];
    float R_Max[Control_Point];
    //积分限幅
    float X_I_Limit[Control_Point];
	float Y_I_Limit[Control_Point];
	float R_I_Limit[Control_Point];
}Path_PID_V1_t;


//该结构体用于串联位置信息和速度信息
typedef struct
{
    //当前位置信息
    float Current_X;
    float Current_Y;
    float Current_R;
    //目标位置
    float Target_X;
    float Target_Y;
    float Target_R;
    //当前速度信息
    float Velocity_X;
    float Velocity_Y;
    float Velocity_R;
    //目标速度
    float Target_Velocity_X;
    float Target_Velocity_Y;
    float Target_Velocity_R;
    //底盘合速度
    float Speed_vehicle; 
    //底盘合速度偏转角
	float Angle_vehicle; 
}Robot_t;


typedef struct
{
	float X_Pos;
	float Y_Pos;
	float Z_Pos;
	
}GYRO;



bool Robot_Pos_Move_PID_V1 (Path_PID_V1_t *path);
#endif
