#include "point_control.h"
#include "RM3508.h"
#include "can_bsp.h"
#include "PID.h"
#include "run.h"

#include "main.h"

#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"

#define PI 3.14

//��־λ
 uint8_t Point_test = 0;



float Cur_Gyro_ZPos = 0;//ƫ����
float Point_Err = 0;//��Ŀ�������������ж��Ƿ񵽵�
float Point_Err_r = 0;//��Ŀ���Ƕ��������ж��Ƿ񵽵�

Path_PID_V1_t Path_PID_32 = {
	.Point_Order = 0,
	.Total_Order = 1,
	.Target_X[0] = 1.892,
	.Target_Y[0] = 0.405,
	.Target_R[0] = -2,
	.Arrive_Err[0] = 0.01,
	.Angle_Err[0] = 1,
	.X_Kp[0] = 4000,
	.X_Ki[0] = 0.5,
	.X_Kd[0] = 0.02,
  .Y_Kp[0] = 4000,
	.Y_Ki[0] = 0.5,
	.Y_Kd[0] = 0.02,
	.R_Kp[0] = 50,
	.R_Ki[0] = 0.3,
	.R_Kd[0] = 0,
	.X_Max[0] = 1000,
	.Y_Max[0] = 1000,
	.R_Max[0] = 500,
	.X_I_Limit[0] = 100,
	.Y_I_Limit[0] = 100,
	.R_I_Limit[0] = 800,
};


Path_PID_V1_t Path_PID_31 = {
	.Point_Order = 0,
	.Total_Order = 1,
	.Target_X[0] = 0.34,
	.Target_Y[0] = 0.405,
	.Target_R[0] = -3,
	.Arrive_Err[0] = 0.01,
	.Angle_Err[0] = 1,
	.X_Kp[0] = 4000,
	.X_Ki[0] = 0.5,
	.X_Kd[0] = 0.02,
  .Y_Kp[0] = 4000,
	.Y_Ki[0] = 0.5,
	.Y_Kd[0] = 0.02,
	.R_Kp[0] = 50,
	.R_Ki[0] = 0.3,
	.R_Kd[0] = 0,
	.X_Max[0] = 1000,
	.Y_Max[0] = 1000,
	.R_Max[0] = 500,
	.X_I_Limit[0] = 100,
	.Y_I_Limit[0] = 100,
	.R_I_Limit[0] = 800,
};


Path_PID_V1_t Path_PID_33 = {
	.Point_Order = 0,
	.Total_Order = 1,
	.Target_X[0] = 3.39,
	.Target_Y[0] = 0.435,
	.Target_R[0] = 0,
	.Arrive_Err[0] = 0.01,
	.Angle_Err[0] = 1,
	.X_Kp[0] = 4000,
	.X_Ki[0] = 0.5,
	.X_Kd[0] = 0.02,
  .Y_Kp[0] = 4000,
	.Y_Ki[0] = 0.5,
	.Y_Kd[0] = 0.02,
	.R_Kp[0] = 50,
	.R_Ki[0] = 0.3,
	.R_Kd[0] = 0,
	.X_Max[0] = 1000,
	.Y_Max[0] = 1000,
	.R_Max[0] = 500,
	.X_I_Limit[0] = 100,
	.Y_I_Limit[0] = 100,
	.R_I_Limit[0] = 800,
};

Path_PID_V1_t Path1_PID_32 = {
	.Point_Order = 0,
	.Total_Order = 1,
	.Target_X[0] = 2.19,
	.Target_Y[0] = 0.405,
	.Target_R[0] = 1,
	.Arrive_Err[0] = 0.01,
	.Angle_Err[0] = 1,
	.X_Kp[0] = 4000,
	.X_Ki[0] = 0.5,
	.X_Kd[0] = 0.02,
  .Y_Kp[0] = 4000,
	.Y_Ki[0] = 0.5,
	.Y_Kd[0] = 0.02,
	.R_Kp[0] = 50,
	.R_Ki[0] = 0.3,
	.R_Kd[0] = 0,
	.X_Max[0] = 1000,
	.Y_Max[0] = 1000,
	.R_Max[0] = 500,
	.X_I_Limit[0] = 100,
	.Y_I_Limit[0] = 100,
	.R_I_Limit[0] = 800,
};

Path_PID_V1_t Path1_PID_31 = {
	.Point_Order = 0,
	.Total_Order = 1,
	.Target_X[0] = 0.68,
	.Target_Y[0] = 0.405,
	.Target_R[0] = 0,
	.Arrive_Err[0] = 0.01,
	.Angle_Err[0] = 1,
	.X_Kp[0] = 4000,
	.X_Ki[0] = 0.5,
	.X_Kd[0] = 0.02,
  .Y_Kp[0] = 4000,
	.Y_Ki[0] = 0.5,
	.Y_Kd[0] = 0.02,
	.R_Kp[0] = 50,
	.R_Ki[0] = 0.3,
	.R_Kd[0] = 0,
	.X_Max[0] = 1000,
	.Y_Max[0] = 1000,
	.R_Max[0] = 500,
	.X_I_Limit[0] = 100,
	.Y_I_Limit[0] = 100,
	.R_I_Limit[0] = 800,
};

Path_PID_V1_t Path1_PID_33 = {
	.Point_Order = 0,
	.Total_Order = 1,
	.Target_X[0] = 3.73,
	.Target_Y[0] = 0.4053,
	.Target_R[0] = -2,
	.Arrive_Err[0] = 0.01,
	.Angle_Err[0] = 1,
	.X_Kp[0] = 4000,
	.X_Ki[0] = 0.5,
	.X_Kd[0] = 0.02,
  .Y_Kp[0] = 4000,
	.Y_Ki[0] = 0.5,
	.Y_Kd[0] = 0.02,
	.R_Kp[0] = 50,
	.R_Ki[0] = 0.3,
	.R_Kd[0] = 0,
	.X_Max[0] = 1000,
	.Y_Max[0] = 1000,
	.R_Max[0] = 500,
	.X_I_Limit[0] = 100,
	.Y_I_Limit[0] = 100,
	.R_I_Limit[0] = 800,
};


Robot_t Robot;
CUR_GYRO Cur_Gyro;

//double current_meter[3];
/*********************************************************************************
 *@  name      : Point_Init
 *@  function  : ��ʼ����λ����
 *@  input     : none
 *@  output    : none
 *********************************************************************************/
//void Point_Init()
//{
//	Path_PID_Test.Point_Order = 0;//��ǰ��ı��
//	Path_PID_Test.Total_Order = 1;//���е�ĸ���
//	  //Ŀ��λ��xyr
//	Path_PID_Test.Target_X[0] = 2;
//  Path_PID_Test.Target_Y[0] = 0.382;
//  Path_PID_Test.Target_R[0] = 0;
//    //Ŀ��λ������
//  Path_PID_Test.Arrive_Err[0] = 0.01;
//    //PID��ز���
//  Path_PID_Test.X_Kp[0] = 8000;
//  Path_PID_Test.X_Ki[0] = 1;
//  Path_PID_Test.X_Kd[0] = 0.02;

//  Path_PID_Test.Y_Kp[0] = 8000;
//  Path_PID_Test.Y_Ki[0] = 1;
//  Path_PID_Test.Y_Kd[0] = 0.02;

//  Path_PID_Test.R_Kp[0] = 20;
//  Path_PID_Test.R_Ki[0] = 0.05;
//  Path_PID_Test.R_Kd[0] = 0;
//    //����޷�
//  Path_PID_Test.X_Max[0] = 1000;
//  Path_PID_Test.Y_Max[0] = 1000;
//  Path_PID_Test.R_Max[0] = 500;
//    //�����޷�
//  Path_PID_Test.X_I_Limit[0] = 100;
//	Path_PID_Test.Y_I_Limit[0] = 100;
//	Path_PID_Test.R_I_Limit[0] = 100;
//}





/*********************************************************************************
 *@  name      : Robot_Run_Point_Pid_V1()
 *@  function  : �ܵ㺯��
 *@  input     : none
 *@  output    : none
 *********************************************************************************/
void Robot_Run_Point_Pid_V1()
{
	//����λ�û�ת���ɳ���ƽ���ٶ�
  Robot.Velocity_X = PID_Calculate (&PID_X,Robot.Current_X ,Robot.Target_X );
  Robot.Velocity_Y = PID_Calculate (&PID_Y,Robot.Current_Y ,Robot.Target_Y );
	//����λ�û�ת���ɳ�����ת�ٶ�
	Robot.Velocity_R = PID_Calculate (&PID_R,Robot.Current_R ,Robot.Target_R );
	
	
//	//�����˶�
//	Robot.Speed_vehicle  = sqrtf(Robot.Velocity_X * Robot.Velocity_X + Robot.Velocity_Y *Robot.Velocity_Y );
//	Robot.Angle_vehicle  = atan2(Robot.Velocity_X , Robot.Velocity_Y);
//	if(Robot.Angle_vehicle < 0)
//		Robot.Angle_vehicle += 6.28f;//2pi
//	//����Speed_vehicle��Angle_vehicle����Velocity_X��Velocity_Y
	Robot.Velocity_X = Robot.Velocity_X * cos(current_meter[2]*PI/180) + Robot.Velocity_Y * sin(current_meter[2]*PI/180);
	Robot.Velocity_Y = Robot.Velocity_X * sin(current_meter[2]*PI/180) + Robot.Velocity_Y * cos(current_meter[2]*PI/180);
//	
	RM3508_Set_Speed((-Robot.Velocity_Y + Robot.Velocity_X + Robot.Velocity_R)*1.5, 1);
	RM3508_Set_Speed((-Robot.Velocity_Y - Robot.Velocity_X + Robot.Velocity_R)*1.5, 2);
	RM3508_Set_Speed((-(-Robot.Velocity_Y - Robot.Velocity_X - Robot.Velocity_R))*1.5, 3);
	RM3508_Set_Speed((-(-Robot.Velocity_Y + Robot.Velocity_X - Robot.Velocity_R))*1.5, 4);
	
	Point_Err = (PID_X.Error * PID_X .Error + PID_Y.Error * PID_Y.Error );
	Point_Err_r = PID_R.Error ;
	
}

/*********************************************************************************
 *@  name      : Robot_Run_Point_Pid_V1()
 *@  function  : �ܵ㺯��
 *@  input     : none
 *@  output    : none
 *********************************************************************************/
void Robot_Run_Point_Pid_V2()
{
	//����λ�û�ת���ɳ���ƽ���ٶ�
  Robot.Velocity_X = PID_Calculate (&PID_X,Robot.Current_X ,Robot.Target_X );
  Robot.Velocity_Y = PID_Calculate (&PID_Y,Robot.Current_Y ,Robot.Target_Y );
	//����λ�û�ת���ɳ�����ת�ٶ�
	Robot.Velocity_R = PID_Calculate (&PID_R,Robot.Current_R ,Robot.Target_R );
	
	
//	//�����˶�
//	Robot.Speed_vehicle  = sqrtf(Robot.Velocity_X * Robot.Velocity_X + Robot.Velocity_Y *Robot.Velocity_Y );
//	Robot.Angle_vehicle  = atan2(Robot.Velocity_X , Robot.Velocity_Y);
//	if(Robot.Angle_vehicle < 0)
//		Robot.Angle_vehicle += 6.28f;//2pi
//	//����Speed_vehicle��Angle_vehicle����Velocity_X��Velocity_Y
	Robot.Velocity_X = Robot.Velocity_X * cos(current_meter[2]*PI/180) + Robot.Velocity_Y * sin(current_meter[2]*PI/180);
	Robot.Velocity_Y = Robot.Velocity_X * sin(current_meter[2]*PI/180) + Robot.Velocity_Y * cos(current_meter[2]*PI/180);
//	
	RM3508_Set_Speed((-Robot.Velocity_Y - Robot.Velocity_X + Robot.Velocity_R)*1.5, 1);
	RM3508_Set_Speed((-Robot.Velocity_Y + Robot.Velocity_X + Robot.Velocity_R)*1.5, 2);
	RM3508_Set_Speed((-(-Robot.Velocity_Y + Robot.Velocity_X - Robot.Velocity_R))*1.5, 3);
	RM3508_Set_Speed((-(-Robot.Velocity_Y - Robot.Velocity_X - Robot.Velocity_R))*1.5, 4);
	
	Point_Err = (PID_X.Error * PID_X .Error + PID_Y.Error * PID_Y.Error );
	Point_Err_r = PID_R.Error ;
	
}


/*********************************************************************************
 *@  name      : Robot_Pos_Move_PID_V1 (Path_PID_V1_t *path)
 *@  function  : �ж��Ƿ񵽵�
 *@  input     : �ܵ�ṹ��
 *@  output    : �Ƿ��ܵ���
 *********************************************************************************/
uint8_t Point_Count = 0; // ���������������� �����ۼƴ���ʱ��
bool Robot_Pos_Move_PID_V1 (Path_PID_V1_t *path)
{
	//�ж��Ƿ��������нڵ�
		if (path->Point_Order == path->Total_Order)
	{
		path->Point_Order = 0;
		return true;
	}
	//����Ŀ��λ��
	Robot.Current_X =4-current_meter[1];
	Robot.Target_X = path->Target_X[path->Point_Order];

	Robot.Current_Y =5.875- current_meter[0];
	Robot.Target_Y = path->Target_Y[path->Point_Order];
	
//	if(current_meter[2] <0)
//	{
//		Angle_Data = current_meter[2] +360.0 ;
//	}
	Robot.Current_R = current_meter[2]  ;
	Robot.Target_R = path->Target_R[path->Point_Order];
	
	
		// PID������ʼ��
	PID_X.K_p = path->X_Kp[path->Point_Order]; // ����������Ƶ�_X��PID����
	PID_X.K_i = path->X_Ki[path->Point_Order];
	PID_X.K_d = path->X_Kd[path->Point_Order];
	PID_X.Output_Max = path->X_Max[path->Point_Order];
	PID_X.IntegralLimit = path->X_I_Limit[path->Point_Order];

	PID_Y.K_p = path->Y_Kp[path->Point_Order]; // ����������Ƶ�_Y��PID����
	PID_Y.K_i = path->Y_Ki[path->Point_Order];
	PID_Y.K_d = path->Y_Kd[path->Point_Order];
	PID_Y.Output_Max = path->Y_Max[path->Point_Order];
	PID_Y.IntegralLimit = path->Y_I_Limit[path->Point_Order];

	PID_R.K_p = path->R_Kp[path->Point_Order]; // ����������Ƶ��PID����
	PID_R.K_i = path->R_Ki[path->Point_Order];
	PID_R.K_d = path->R_Kd[path->Point_Order];
	PID_R.Output_Max = path->R_Max[path->Point_Order];
	PID_R.IntegralLimit = path->R_I_Limit[path->Point_Order];
	
	//ִ���ܵ�
	Robot_Run_Point_Pid_V1();

	
	//�ж��Ƿ񵽽ڵ�
	if (Point_Err < path->Arrive_Err[path->Point_Order])
		if(Point_Err_r <path->Angle_Err[path->Point_Order])
		if (Point_Count++ == 100)
		{
			path->Point_Order++; // ��һ�����Ƶ�

			PID_X.I_Out = 0;
			PID_Y.I_Out = 0;
			PID_X.I_Out = 0; // ��������������
			Point_Count = 0;
		}

	return false;
	
}


/*********************************************************************************
 *@  name      : Robot_Pos_Move_PID_V2 (Path_PID_V1_t *path)
 *@  function  : �ж��Ƿ񵽵�
 *@  input     : �ܵ�ṹ��
 *@  output    : �Ƿ��ܵ���
 *********************************************************************************/
double Angle_Data = 0;
bool Robot_Pos_Move_PID_V2 (Path_PID_V1_t *path)
{
	//�ж��Ƿ��������нڵ�
		if (path->Point_Order == path->Total_Order)
	{
		path->Point_Order = 0;
		return true;
	}
	//����Ŀ��λ��
	Robot.Current_X =4-current_meter[1];
	Robot.Target_X = path->Target_X[path->Point_Order];

	Robot.Current_Y =5.875- current_meter[0];
	Robot.Target_Y = path->Target_Y[path->Point_Order];
	
//	if(current_meter[2] <0)
//	{
//		Robot.Current_R = current_meter[2] +360.0 ;
//	}
	Robot.Current_R =  current_meter[2] ;
	Robot.Target_R = path->Target_R[path->Point_Order];
	
	
		// PID������ʼ��
	PID_X.K_p = path->X_Kp[path->Point_Order]; // ����������Ƶ�_X��PID����
	PID_X.K_i = path->X_Ki[path->Point_Order];
	PID_X.K_d = path->X_Kd[path->Point_Order];
	PID_X.Output_Max = path->X_Max[path->Point_Order];
	PID_X.IntegralLimit = path->X_I_Limit[path->Point_Order];

	PID_Y.K_p = path->Y_Kp[path->Point_Order]; // ����������Ƶ�_Y��PID����
	PID_Y.K_i = path->Y_Ki[path->Point_Order];
	PID_Y.K_d = path->Y_Kd[path->Point_Order];
	PID_Y.Output_Max = path->Y_Max[path->Point_Order];
	PID_Y.IntegralLimit = path->Y_I_Limit[path->Point_Order];

	PID_R.K_p = path->R_Kp[path->Point_Order]; // ����������Ƶ��PID����
	PID_R.K_i = path->R_Ki[path->Point_Order];
	PID_R.K_d = path->R_Kd[path->Point_Order];
	PID_R.Output_Max = path->R_Max[path->Point_Order];
	PID_R.IntegralLimit = path->R_I_Limit[path->Point_Order];
	
	//ִ���ܵ�
	Robot_Run_Point_Pid_V2();

	
	//�ж��Ƿ񵽽ڵ�
	if (Point_Err < path->Arrive_Err[path->Point_Order])
		if(Point_Err_r <path->Angle_Err[path->Point_Order])
		if (Point_Count++ == 100)
		{
			path->Point_Order++; // ��һ�����Ƶ�

			PID_X.I_Out = 0;
			PID_Y.I_Out = 0;
			PID_X.I_Out = 0; // ��������������
			Point_Count = 0;
		}

	return false;
	
}








