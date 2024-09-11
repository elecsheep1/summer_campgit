#include "PID.h"
#include "math.h"
PID_t PID_X = {
      .K_p = 20,
      .K_i = 0,
			.K_d=0.01,
			.IntegralLimit = 0,
			.DeadBand = 0,
			.I_Start = 0,
			.Output_Max = 1000
};



PID_t PID_Y = {
      .K_p = 20,
      .K_i = 0,
			.K_d=0.01,
			.IntegralLimit = 0,
			.DeadBand = 0,
			.I_Start = 0,
			.Output_Max = 1000
};



PID_t PID_R = {
      .K_p = 15,
      .K_i = 0.1,
			.K_d=0.01,
			.IntegralLimit = 10,
			.DeadBand = 0,
			.I_Start = 0,
			.Output_Max = 200
};



float PID_Calculate (PID_t *pid , float Current_Value , float Target_Value)
{
		pid->Error_Last = pid->Error;
		pid->Error = Target_Value - Current_Value;  
		pid->Error = DeadBand(pid->Error,pid->DeadBand);
		
    //P
		pid->P_Out = pid->K_p * pid->Error;
    //I
		if(fabsf(pid->Error) < pid->I_Start && pid->Error != 0 )
		{
			pid->I_Out += pid->K_i * pid->Error;
			pid->I_Out = Limit(pid->I_Out , pid->IntegralLimit);
		}  
		else 
			pid->I_Out = 0; 
    //D
		pid->D_Out = pid->K_d * (pid->Error - pid->Error_Last);
    //PID
		pid->PID_Out = Limit (pid->P_Out + pid->I_Out + pid->D_Out , pid->Output_Max);
		
		return pid->PID_Out;
}

