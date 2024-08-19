#include "point_control.h"
#include "RM3508.h"
#include "can_bsp.h"
#include "PID.h"

#include "main.h"

#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"

#define PI 3.14

float Cur_Gyro_ZPos = 0;//偏航角
float Point_Err = 0;//与目标点距离误差，用于判断是否到点
float Point_Err_r = 0;//与目标点角度误差，用于判断是否到点

Robot_t Robot;
GYRO Cur_Gyro;
/*********************************************************************************
 *@  name      : Robot_Run_Point_Pid_V1()
 *@  function  : 跑点函数
 *@  input     : none
 *@  output    : none
 *********************************************************************************/
void Robot_Run_Point_Pid_V1()
{
	//底盘位置环转换成车体平动速度
  Robot.Velocity_X = PID_Calculate (&PID_X,Robot.Current_X ,Robot.Target_X );
  Robot.Velocity_Y = PID_Calculate (&PID_Y,Robot.Current_Y ,Robot.Target_Y );
	//底盘位置环转换成车体旋转速度
	Robot.Velocity_R = PID_Calculate (&PID_R,Robot.Current_R ,Robot.Target_R );
	
	
	//车体运动
	Robot.Speed_vehicle  = sqrtf(Robot.Velocity_X * Robot.Velocity_X + Robot.Velocity_Y *Robot.Velocity_Y );
	Robot.Angle_vehicle  = atan2(Robot.Velocity_X , Robot.Velocity_Y);
	if(Robot.Angle_vehicle < 0)
		Robot.Angle_vehicle += 6.28f;//2pi
	//再用Speed_vehicle和Angle_vehicle更新Velocity_X和Velocity_Y
	Robot.Velocity_X = Robot.Speed_vehicle * sin(Robot.Angle_vehicle - Cur_Gyro.Z_Pos /180 * PI );
	Robot.Velocity_Y = Robot.Speed_vehicle * cos(Robot.Angle_vehicle - Cur_Gyro.Z_Pos /180 * PI );
	
	RM3508_Set_Speed(-(Robot.Velocity_Y - Robot.Velocity_X - Robot.Velocity_R), 1);
	RM3508_Set_Speed(Robot.Velocity_Y + Robot.Velocity_X + Robot.Velocity_R, 2);
	RM3508_Set_Speed(Robot.Velocity_Y - Robot.Velocity_X + Robot.Velocity_R, 3);
	RM3508_Set_Speed(-(Robot.Velocity_Y + Robot.Velocity_X - Robot.Velocity_R), 4);
	
	Point_Err = (PID_X.Error * PID_X .Error + PID_Y.Error * PID_Y.Error );
	Point_Err_r = PID_R.Error ;
	
}

/*********************************************************************************
 *@  name      : Robot_Pos_Move_PID_V1 (Path_PID_V1_t *path)
 *@  function  : 判断是否到点
 *@  input     : 跑点结构体
 *@  output    : 是否跑到点
 *********************************************************************************/
uint8_t Point_Count = 0; // 到点后误差在死区内 次数累计代替时间
bool Robot_Pos_Move_PID_V1 (Path_PID_V1_t *path)
{
	//判断是否跑完所有节点
		if (path->Point_Order == path->Total_Order)
	{
		path->Point_Order = 0;
		return true;
	}
	//设置目标位置
	Robot.Current_X = Cur_Gyro.X_Pos;
	Robot.Target_X = path->Target_X[path->Point_Order];

	Robot.Current_Y = Cur_Gyro.Y_Pos;
	Robot.Target_Y = path->Target_Y[path->Point_Order];

	Robot.Current_R = Cur_Gyro.Z_Pos;
	Robot.Target_R = path->Target_R[path->Point_Order];
	
	
		// PID参数初始化
	PID_X.K_p = path->X_Kp[path->Point_Order]; // 现在这个控制点_X的PID参数
	PID_X.K_i = path->X_Ki[path->Point_Order];
	PID_X.K_d = path->X_Kd[path->Point_Order];
	PID_X.Output_Max = path->X_Max[path->Point_Order];
	PID_X.IntegralLimit = path->X_I_Limit[path->Point_Order];

	PID_Y.K_p = path->Y_Kp[path->Point_Order]; // 现在这个控制点_Y的PID参数
	PID_Y.K_i = path->Y_Ki[path->Point_Order];
	PID_Y.K_d = path->Y_Kd[path->Point_Order];
	PID_Y.Output_Max = path->Y_Max[path->Point_Order];
	PID_Y.IntegralLimit = path->Y_I_Limit[path->Point_Order];

	PID_R.K_p = path->R_Kp[path->Point_Order]; // 现在这个控制点的PID参数
	PID_R.K_i = path->R_Ki[path->Point_Order];
	PID_R.K_d = path->R_Kd[path->Point_Order];
	PID_R.Output_Max = path->R_Max[path->Point_Order];
	PID_R.IntegralLimit = path->R_I_Limit[path->Point_Order];
	
	//执行跑点
	Robot_Run_Point_Pid_V1();
	
	//判断是否到节点
	if (Point_Err < path->Arrive_Err[path->Point_Order])
		if (Point_Count++ == 5)
		{
			path->Point_Order++; // 下一个控制点

			PID_X.I_Out = 0;
			PID_Y.I_Out = 0;
			PID_X.I_Out = 0; // 清零重新启动！

			Point_Count = 0;
		}

	return false;
	
}



