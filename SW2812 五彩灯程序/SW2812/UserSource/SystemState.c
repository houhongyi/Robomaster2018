#include "SystemState.h"
#include <string.h>
#include <math.h>

#include "WS2812.h"

SystemStateDef SystemState={0};
float g_TimePer[100]={0};


float g_Time_DeviceOutLine[DeviceTotal_No]={0};//�������һ��ͨ��ʱ������

//=====================================================
//							  �ڲ�����
//
//====================================================


//=====================================================
//							  �ⲿ����
//
//====================================================
//ϵͳ��ʼ�� �ɹ� ����1 ʧ�ܷ���0

int SystemState_Inite()
{
	SystemState.Enable=0;
	SystemState.State=0;
	SystemState.Task=0;
	SystemState.Time=0;
	SystemState.htim=&htim4;//��ʱ�����趨 ÿ 10us ��һ����  ����ֵΪ 1000-1 (1ms)  ���� Timer3 ��Ƶ168M Ԥ��Ƶ (840-1) ����ֵ (100-1)
	HAL_TIM_Base_Start_IT(SystemState.htim);//����ʱ�������
	return 1;
}

//�ж�ˢ���е��� ����ϵͳʱ�� ms
 inline void RefreshSysTime()
{
		SystemState.Time+=1;
}

//���ϵͳʱ��
inline float GetSystemTimer()
{
	return SystemState.htim->Instance->CNT/1000 +SystemState.Time;
}


//ˢ������ͨ��ʱ��ʱ������
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No)
{
	g_Time_DeviceOutLine[DevX_No]=GetSystemTimer();
}

//�������м�� ��g_TimePer�б����100�μ��
void MesureTimePeriod()
{
	static float pre_time=0;
	float time=GetSystemTimer();
	int i=100;
	for(i=99;i>0;i--)
	{
		g_TimePer[i]=g_TimePer[i-1];
	}
	g_TimePer[0]=time-pre_time;
	pre_time=time;
}

//**************************************************************************
//**************************************************************************
//����ΪTIM�����ж�ǿ����  ��Ҫ�������������������ö�ʱ���ж���ӵ��˴�*****
//**************************************************************************
//**************************************************************************

//ϵͳʱ���ʱ����ʱ�ж�  ����ʱ�䣨1mm �жϣ� 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim->Instance==SystemState.htim->Instance)
	{
		RefreshSysTime();//����ʱ��
	}
}

