#ifndef __SysState_H__
#define __SysState_H__

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
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
	float Dist1;
	float Dist2;
	float Dist3;
	float Dist4;
	float Dist5;
	float Dist6;
	
}RobotDistDef;

typedef struct{
	short Mode;//����ģʽ
	short Enable;//״̬
	short State;//״̬
	short Task;//����
	BEEPMode Beep;//������
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//ʱ����������
	long OutLine_Flag;//���߱�־
	
	RobotDistDef RobotDist;//�����˲���
}SystemStateDef;


typedef enum
{
	MainCtr_Heart,
	Dist1_No,
	Dist2_No,
	Dist3_No,
	Dist4_No,
	Dist5_No,
	Dist6_No,
	
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
