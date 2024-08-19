#ifndef __FILT_H
#define __FILT_H

typedef struct Kalman_filter
{
	float p ;				    /*上次预测过程协方差矩阵 */
	float prevData ;				    /*上一次系统状态预测矩阵，列矩阵*/
	
	float q;						/*过程噪声协方差*/
	float r;						/*量测噪声协方差*/
	
	float kGain;						/*卡尔曼增益，列矩阵*/
//	float X;						/*最优估计输出矩阵，列矩阵*/
//	float C;						/*最优估计协方差矩阵C(k|k)*/
                            
	float inData;				    /*量测值，即Z(k)*/
} Kal_Filter;

extern Kal_Filter k_filt;
extern Kal_Filter *K_filt;

extern Kal_Filter k_filt1;
extern Kal_Filter *K_filt1;

float KalmanFilter(Kal_Filter* K_Filt,float inData);
//int averageFilter(int N);
//int firstOrderFilter(int newValue, int oldValue, float a);
#endif


