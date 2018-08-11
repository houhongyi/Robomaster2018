#include "SystemState.h"
#include <string.h>
#include <math.h>

SystemStateDef SystemState={0};
float g_TimePer[100]={0};


float g_Time_DeviceOutLine[DeviceTotal_No]={0};//外设最近一次通信时间数组

//=====================================================
//							  内部函数
//
//====================================================

//断线检测检测
void OutLine_Check()
{
	short num=0;//临时变量累加用
	float time=GetSystemTimer();//当前系统时间

	for(num=0;num<DeviceTotal_No;num++)
	{
		if(time-g_Time_DeviceOutLine[num]>OutLine_Time)
		{
			MyFlagSet(SystemState.OutLine_Flag,(num));//设置断线标志
		}
		else
		{
			MyFlagClear(SystemState.OutLine_Flag,(num));//清除断线标志
		}
	}
}

//断线检测
void vOutLineCheck_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		OutLine_Check();//断线检测
		vTaskDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);
	}
}

//=====================================================
//							  外部调用
//
//====================================================
//系统初始化 成功 返回1 失败返回0

int SystemState_Inite()
{
	SystemState.Enable=0;
	SystemState.State=0;
	SystemState.Task=0;
	SystemState.Beep=Beep_Silence;
	SystemState.Time=0;
	SystemState.htim=&htim6;//计时器请设定 每 10us 记一个数  重载值为 100-1 (1ms)  例如 Timer3 主频168M 预分频 (840-1) 重载值 (100-1)
	HAL_TIM_Base_Start_IT(SystemState.htim);//启动时间计数器
	
	if(!xTaskCreate( vOutLineCheck_Task, "OutLineCheck_Task", 200, NULL, 3, NULL ))return 0;
	return 1;
}

//中断刷新中调用 更新系统时间 ms
 inline void RefreshSysTime()
{
		SystemState.Time+=1;
}

//获得系统时间
inline float GetSystemTimer()
{
	return SystemState.htim->Instance->CNT/100.0 +SystemState.Time;
}


//刷新外设通信时间时间数组
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No)
{
	g_Time_DeviceOutLine[DevX_No]=GetSystemTimer();
}

//测量运行间隔 在g_TimePer中保存近100次间隔
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
//本处为TIM更新中断强引用  需要将本工程内其他弱引用定时器中断添加到此处*****
//**************************************************************************
//**************************************************************************

//系统时间计时器定时中断  更新时间（1mm 中断） 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==SystemState.htim->Instance)
	{
		RefreshSysTime();//更新时间
	}
}

