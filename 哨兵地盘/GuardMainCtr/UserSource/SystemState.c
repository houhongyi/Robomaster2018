#include "SystemState.h"
#include <string.h>

//======= other Lib
#include "bsp_motor.h"

SystemStateDef SystemState={0};

float g_Time_MotorStall[MotorTotal_No]={0};//�����תʱ������
float g_Time_MotorOutLine[MotorTotal_No]={0};//������һ��ͨ��ʱ������
float g_Time_DeviceOutLine[DeviceTotal_No]={0};//�������һ��ͨ��ʱ������
float g_Time_TaskPeriod[TaskTotal_No]={0};//���������м��
float g_TimePre_Taskstart[TaskTotal_No]={0};//��������������ʱ��
float g_TimePer[100]={0};

MotorXDef* Motors[MotorTotal_No]={&Motor1,&Motor2};

//=====================================================
//							  �ڲ�����
//
//====================================================
//���߼����
void OutLine_Check()
{
	short num=0;//��ʱ�����ۼ���
	float time=GetSystemTime();//��ǰϵͳʱ��

	for(num=0;num<MotorTotal_No;num++)
	{
		if(time-g_Time_MotorOutLine[num]>50)
		{
			MyFlagSet(SystemState.OutLine_Flag,num);//���ö��߱�־
		}
		else 
		{
			MyFlagClear(SystemState.OutLine_Flag,num);//������߱�־
		}
	}
	
	for(num=0;num<DeviceTotal_No;num++)
	{
		if(time-g_Time_DeviceOutLine[num]>OutLine_Time)
		{
			MyFlagSet(SystemState.OutLine_Flag,(MotorTotal_No+num));//���ö��߱�־
		}
		else
		{
			MyFlagClear(SystemState.OutLine_Flag,(MotorTotal_No+num));//������߱�־
		}
	}
}

//�����ת�����
void MotorStall_Check()
{
	short num=0;//��ʱ�����ۼ���
	float time=GetSystemTime();//��ǰϵͳʱ��
	char MotorStall_Flag=0;//�����ת��־
	for(num=0;num<MotorTotal_No;num++)
	{
		if(my_abs(Motors[num]->State.Spd)<Motor_Stall_Spd && my_abs(Motors[num]->Motor_output)>Motor_Stall_Output)//���һ������������2500 ���ٶ�С��5mm/s ��Ϊ��ת
		{
			if(!g_Time_MotorStall[num])
				g_Time_MotorStall[num]=GetSystemTime();//��¼��תʱ��
		}
		else
			g_Time_MotorStall[num]=0;
		
		if(g_Time_MotorStall[num])
		{
			if(time-g_Time_MotorStall[num]>Motor_Stall_Time)//�����תʱ����� 200ms
			{
				SetMotorStall(Motors[num]->State.State);//���ö�ת��־λ
				MotorStall_Flag++;///��ת��־�Ӽ�
			}
			else
			{
				ClearMotorStall(Motors[num]->State.State);//�����ת��־
			}
		}
		else
			ClearMotorStall(Motors[num]->State.State);//�����ת��־
		
		if(MotorStall_Flag)
			SetSystemState(SystemState_MotorStall);//�趨��ת״̬
		else 
			ClearSystemState(SystemState_MotorStall);//�����ת״̬
	}
}

//�ж�ˢ���е��� ����ϵͳʱ�� ms
inline void RefreshSysTime()
{	
	SystemState.Time+=1;
}

//���߼��
void vOutLineCheck_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		OutLine_Check();//���߼��
		MotorStall_Check();//�����ת���
		vTaskDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);
	}
}



//=====================================================
//							  �ⲿ����
//
//=====================================================
//ϵͳ��ʼ�� �ɹ� ����1 ʧ�ܷ���0
int SystemState_Inite()
{
	SystemState.Enable=System_Disable;
	SystemState.Robote.Mode=Mode_Cruise;//Ѳ��ģʽ
	SystemState.Robote.RunState=State_AutoCTR;//�Զ�����״̬
	SystemState.Robote.GarderFire=Fire_Permite;//������Ȩ
	SystemState.Task=0;
	SystemState.Beep=Beep_Silence;
	SystemState.Time=0;
	SystemState.Wifi_ctrTime=0;//�ϴ�WIFI����ʱ��
	SystemState.htim=&htim6;//��ʱ�����趨 ÿ 10us ��һ����  ����ֵΪ 100-1 (1ms)  ���� Timer3 ��Ƶ168M Ԥ��Ƶ (840-1) ����ֵ (100-1)
	HAL_TIM_Base_Start_IT(SystemState.htim);//����ʱ�������
	if(!xTaskCreate( vOutLineCheck_Task, "OutLineCheck_Task", 200, NULL, 3, NULL ))return 0;
	return 1;
}


//���ϵͳʱ��
inline float GetSystemTime()
{	
	return SystemState.htim->Instance->CNT/100.0 +SystemState.Time;
}

//ˢ�µ��ͨ��ʱ������
void RefreshMotorOutLineTime(MotorX_NoDEF Mot_No)
{
	g_Time_MotorOutLine[Mot_No]=GetSystemTime();
}

//ˢ������ͨ��ʱ��ʱ������
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No)
{
	g_Time_DeviceOutLine[DevX_No]=GetSystemTime();
}

//��ȡ�������������ڼ��
void GetTaskPeriod(TaskX_NoDEF No)
{
	g_Time_TaskPeriod[No]=0.02f*(GetSystemTime()-g_TimePre_Taskstart[No])+0.98f*g_Time_TaskPeriod[No];
	g_TimePre_Taskstart[No]=GetSystemTime();
}

//  ��ȡ��100�����м��
void MesureTimePeriod()
{
	static float pre_time=0;
	float time=GetSystemTime();
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
	//===============

}
