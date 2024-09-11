#ifndef __POINT_CONTROL_H
#define __POINT_CONTROL_H

#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"
//底盘位置环PID结构体
#define Control_Point 1//从初始位置到目标位置总共的节点个数
//#define SRART_CHAR1 0xBB
//#define SRART_CHAR2
//#define END_CHAR 0x55
typedef struct
{
	  uint16_t Point_Order;//当前点的编号
	  uint16_t Total_Order;//所有点的个数
	  //目标位置xyr
	  double Target_X[Control_Point];
    double Target_Y[Control_Point];
    double Target_R[Control_Point];
    //目标位置死区
    double Arrive_Err[Control_Point];
	  double Angle_Err[Control_Point];
    //PID相关参数
    double X_Kp[Control_Point];
    double X_Ki[Control_Point];
    double X_Kd[Control_Point];

    double Y_Kp[Control_Point];
    double Y_Ki[Control_Point];
    double Y_Kd[Control_Point];

    double R_Kp[Control_Point];
    double R_Ki[Control_Point];
    double R_Kd[Control_Point];
    //输出限幅
    double X_Max[Control_Point];
    double Y_Max[Control_Point];
    double R_Max[Control_Point];
    //积分限幅
    double X_I_Limit[Control_Point];
	double Y_I_Limit[Control_Point];
	double R_I_Limit[Control_Point];
}Path_PID_V1_t;


//该结构体用于串联位置信息和速度信息
typedef struct
{
    //当前位置信息
    double Current_X;
    double Current_Y;
    double Current_R;
    //目标位置
    double Target_X;
    double Target_Y;
    double Target_R;
    //当前速度信息
    double Velocity_X;
    double Velocity_Y;
    double Velocity_R;
    //目标速度
    double Target_Velocity_X;
    double Target_Velocity_Y;
    double Target_Velocity_R;
    //底盘合速度
    double Speed_vehicle; 
    //底盘合速度偏转角
	double Angle_vehicle; 
}Robot_t;


typedef struct
{
	float X_Pos;
	float Y_Pos;
	float Z_Pos;
}CUR_GYRO;



extern uint8_t Point_test;
extern Path_PID_V1_t Path_PID_31;
extern Path_PID_V1_t Path_PID_32;
extern Path_PID_V1_t Path_PID_33;

extern Path_PID_V1_t Path1_PID_31;
extern Path_PID_V1_t Path1_PID_32;
extern Path_PID_V1_t Path1_PID_33;

extern CUR_GYRO Cur_Gyro;
//extern double current_meter[3];


void Robot_Run_Point_Pid_V1();
void Robot_Run_Point_Pid_V2();
bool Robot_Pos_Move_PID_V1 (Path_PID_V1_t *path);
bool Robot_Pos_Move_PID_V2 (Path_PID_V1_t *path);
#endif
