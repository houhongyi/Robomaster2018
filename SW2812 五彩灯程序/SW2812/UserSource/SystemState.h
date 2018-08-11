#ifndef __SysState_H__
#define __SysState_H__

#include "stm32f1xx_hal.h"
#include "tim.h"
#include "MyCom.h"
//===============================================================ϵͳģʽ����


#define SetSystemState(x) SystemState.State=SystemState.State|x //����ϵͳ״̬
#define ClearSystemState(x) SystemState.State=SystemState.State&(~x) //���ϵͳ״̬
#define GetSystemState(x) (SystemState.State&x)//��ȡϵͳ״̬



//==============================================================�����ת����

#define OutLine_Time 50 //���߼��ʱ��

typedef enum
{
	Beep_Silence, //��Ĭ
	Beep_Beeping, //����
}BEEPMode;




typedef struct{
	short Mode;//����ģʽ
	short Enable;//״̬
	short State;//״̬
	short Task;//����
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//ʱ����������
	long OutLine_Flag;//���߱�־
	
}SystemStateDef;


typedef enum
{
	MainCtr_Heart,
	
	DeviceTotal_No	
}DeviceX_NoDEF;


extern SystemStateDef SystemState;


int SystemState_Inite(void);//SystemState��ʼ��
inline void RefreshSysTime(void);//ˢ��ϵͳʱ�䣨mm��
float GetSystemTimer(void);//��ȡϵͳ��ǰ׼ȷʱ��
void MesureTimePeriod(void);//�������м��

void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//ˢ������ͨ��ʱ��ʱ������
void OutLine_Check(void);//���߼����
#endif
