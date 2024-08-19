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
 * @brief  һ�׵�ͨ�˲�
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
 * @brief  �������˲�
 * @note  None
 * @param  None
 * @retval None
  */
float Kalman_Filter(Kal_Filter* K_Flt, float Input)
{
    /*������£�3�鷽��*/
    K_Flt->Input = Input;
    K_Flt->K = (K_Flt->C_last) / (K_Flt->C_last + K_Flt->R);
    K_Flt->X  = K_Flt->X_last + K_Flt->K * (K_Flt->Input - K_Flt->X_last);
    K_Flt->C =  (1 - K_Flt->K) * (K_Flt->C_last);

    /*ʱ����£�2�鷽��*/
    K_Flt->X_last = K_Flt->X;
    K_Flt->C_last = K_Flt->C + K_Flt->Q;

    return K_Flt->X;
}
