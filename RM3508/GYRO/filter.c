#include "filter.h"

First_Filter First_Flt =
{0.004f, 0.0f, 0.0f};

/**
 * @brief  None
 * @note  None
 * @param  None
 * @retval None
  */
void Filter_Init(void)
{

}

/**
 * @brief  一阶低通滤波
 * @note  None
 * @param  None
 * @retval None
  */
float First_Order_Filter(First_Filter* First_Struct, float Input)
{
    First_Struct->Current_Output = First_Struct->K * (Input - First_Struct->Last_Output) + First_Struct->Last_Output;
    First_Struct->Last_Output = First_Struct->Current_Output;

    return First_Struct->Current_Output;
}

/**
 * @brief  卡尔曼滤波
 * @note  None
 * @param  None
 * @retval None
  */
float Kalman_Filter(Kal_Filter* K_Flt, float Input)
{
    /*量测更新，3组方程*/
    K_Flt->Input = Input;
    K_Flt->K = (K_Flt->C_last) / (K_Flt->C_last + K_Flt->R);
    K_Flt->X  = K_Flt->X_last + K_Flt->K * (K_Flt->Input - K_Flt->X_last);
    K_Flt->C =  (1 - K_Flt->K) * (K_Flt->C_last);

    /*时间更新，2组方程*/
    K_Flt->X_last = K_Flt->X;
    K_Flt->C_last = K_Flt->C + K_Flt->Q;

    return K_Flt->X;
}
