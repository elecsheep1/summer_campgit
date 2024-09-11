#ifndef __PID_H
#define __PID_H

#define DeadBand(x, limit) ((x >= limit || x <= -limit) ? x : 0)
#define Limit(x, limit) (x >= limit ? limit : (x <= -limit ? -limit : x))

typedef struct
{
		float IntegralLimit;
		float DeadBand;

		float K_p;
		float K_i;
		float K_d;

		float Error;
		float Error_Last;
		float Error_Sum;

		float I_Start;
		
		float P_Out;
		float I_Out;
		float D_Out;
		
		float PID_Out;
		float Output_Max;

}PID_t;



extern PID_t PID_X;
extern PID_t PID_Y;
extern PID_t PID_R;

float PID_Calculate (PID_t *pid , float Current_Value , float Target_Value);

#endif

