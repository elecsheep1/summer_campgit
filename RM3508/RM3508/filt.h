#ifndef __FILT_H
#define __FILT_H

typedef struct Kalman_filter
{
	float p ;				    /*�ϴ�Ԥ�����Э������� */
	float prevData ;				    /*��һ��ϵͳ״̬Ԥ������о���*/
	
	float q;						/*��������Э����*/
	float r;						/*��������Э����*/
	
	float kGain;						/*���������棬�о���*/
//	float X;						/*���Ź�����������о���*/
//	float C;						/*���Ź���Э�������C(k|k)*/
                            
	float inData;				    /*����ֵ����Z(k)*/
} Kal_Filter;

extern Kal_Filter k_filt;
extern Kal_Filter *K_filt;

extern Kal_Filter k_filt1;
extern Kal_Filter *K_filt1;

float KalmanFilter(Kal_Filter* K_Filt,float inData);
//int averageFilter(int N);
//int firstOrderFilter(int newValue, int oldValue, float a);
#endif


