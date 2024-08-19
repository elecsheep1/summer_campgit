#ifndef FILTER_H
#define FILTER_H

/*һάkalman�˲��ṹ�嶨��,A=1,B=0,H=1*/
typedef struct Kalman_filter
{
	float C_last;				    /*�ϴ�Ԥ�����Э������� C(k|k-1)*/
	float X_last;				    /*ϵͳ״̬Ԥ������о���*/
	
	float Q;						/*��������Э����*/
	float R;						/*��������Э����*/
	
	float K;						/*���������棬�о���*/
	float X;						/*���Ź�����������о���*/
	float C;						/*���Ź���Э�������C(k|k)*/
                            
	float Input;				    /*����ֵ����Z(k)*/
} Kal_Filter;

typedef struct
{
    float K;								//x[n]ϵ��
    float Current_Output;		            //y[n]
    float Last_Output;			            //y[n-1]
} First_Filter;

void Filter_Init(void);
float First_Order_Filter(First_Filter* First_Struct, float Input);
float Kalman_Filter(Kal_Filter* K_Flt, float Input);


#endif
