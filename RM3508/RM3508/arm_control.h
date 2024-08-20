#ifndef __ARM_CONTROL_H
#define __ARM_CONTROL_H

#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"


typedef struct
{
   float A3508_Pos;
   float A2006_Pos_1;
   float A2006_Pos_2;   
   float Arrive_Error;
}ARM;


bool Arm_Pos_PID_V1 (ARM* arm);
#endif
