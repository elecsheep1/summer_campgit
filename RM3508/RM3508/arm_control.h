#ifndef __ARM_CONTROL_H
#define __ARM_CONTROL_H

#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"


#define PI 3.14

typedef struct
{
   float ARM_Pos_5;
   float ARM_Pos_6;
   float ARM_Pos_7;   
   float Arrive_Error;
}ARM;


bool Arm_Pos_PID_V1 (ARM* arm);
#endif
