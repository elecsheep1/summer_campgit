#ifndef FILTER_H
#define FILTER_H

/*一维kalman滤波结构体定义,A=1,B=0,H=1*/
typedef struct Kalman_filter
{
	float C_last;				    /*上次预测过程协方差矩阵 C(k|k-1)*/
	float X_last;				    /*系统状态预测矩阵，列矩阵*/
	
	float Q;						/*过程噪声协方差*/
	float R;						/*量测噪声协方差*/
	
	float K;						/*卡尔曼增益，列矩阵*/
	float X;						/*最优估计输出矩阵，列矩阵*/
	float C;						/*最优估计协方差矩阵C(k|k)*/
                            
	float Input;				    /*量测值，即Z(k)*/
} Kal_Filter;

typedef struct
{
    float K;								//x[n]系数
    float Current_Output;		            //y[n]
    float Last_Output;			            //y[n-1]
} First_Filter;

void Filter_Init(void);
float First_Order_Filter(First_Filter* First_Struct, float Input);
float Kalman_Filter(Kal_Filter* K_Flt, float Input);


#endif
