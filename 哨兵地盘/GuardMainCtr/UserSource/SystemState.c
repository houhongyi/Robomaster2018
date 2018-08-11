#include "SystemState.h"
#include <string.h>

//======= other Lib
#include "bsp_motor.h"

SystemStateDef SystemState={0};

float g_Time_MotorStall[MotorTotal_No]={0};//电机堵转时间数组
float g_Time_MotorOutLine[MotorTotal_No]={0};//电机最近一次通信时间数组
float g_Time_DeviceOutLine[DeviceTotal_No]={0};//外设最近一次通信时间数组
float g_Time_TaskPeriod[TaskTotal_No]={0};//各任务运行间隔
float g_TimePre_Taskstart[TaskTotal_No]={0};//各任务运行启动时间
float g_TimePer[100]={0};

MotorXDef* Motors[MotorTotal_No]={&Motor1,&Motor2};

//=====================================================
//							  内部函数
//
//====================================================
//断线检测检测
void OutLine_Check()
{
	short num=0;//临时变量累加用
	float time=GetSystemTime();//当前系统时间

	for(num=0;num<MotorTotal_No;num++)
	{
		if(time-g_Time_MotorOutLine[num]>50)
		{
			MyFlagSet(SystemState.OutLine_Flag,num);//设置断线标志
		}
		else 
		{
			MyFlagClear(SystemState.OutLine_Flag,num);//清除断线标志
		}
	}
	
	for(num=0;num<DeviceTotal_No;num++)
	{
		if(time-g_Time_DeviceOutLine[num]>OutLine_Time)
		{
			MyFlagSet(SystemState.OutLine_Flag,(MotorTotal_No+num));//设置断线标志
		}
		else
		{
			MyFlagClear(SystemState.OutLine_Flag,(MotorTotal_No+num));//清除断线标志
		}
	}
}

//电机堵转检测检测
void MotorStall_Check()
{
	short num=0;//临时变量累加用
	float time=GetSystemTime();//当前系统时间
	char MotorStall_Flag=0;//电机堵转标志
	for(num=0;num<MotorTotal_No;num++)
	{
		if(my_abs(Motors[num]->State.Spd)<Motor_Stall_Spd && my_abs(Motors[num]->Motor_output)>Motor_Stall_Output)//如果一个电机输出大于2500 而速度小于5mm/s 认为堵转
		{
			if(!g_Time_MotorStall[num])
				g_Time_MotorStall[num]=GetSystemTime();//记录堵转时间
		}
		else
			g_Time_MotorStall[num]=0;
		
		if(g_Time_MotorStall[num])
		{
			if(time-g_Time_MotorStall[num]>Motor_Stall_Time)//如果堵转时间大于 200ms
			{
				SetMotorStall(Motors[num]->State.State);//设置堵转标志位
				MotorStall_Flag++;///堵转标志加加
			}
			else
			{
				ClearMotorStall(Motors[num]->State.State);//清除堵转标志
			}
		}
		else
			ClearMotorStall(Motors[num]->State.State);//清除堵转标志
		
		if(MotorStall_Flag)
			SetSystemState(SystemState_MotorStall);//设定堵转状态
		else 
			ClearSystemState(SystemState_MotorStall);//清除堵转状态
	}
}

//中断刷新中调用 更新系统时间 ms
inline void RefreshSysTime()
{	
	SystemState.Time+=1;
}

//断线检测
void vOutLineCheck_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		OutLine_Check();//断线检测
		MotorStall_Check();//电机堵转检测
		vTaskDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);
	}
}



//=====================================================
//							  外部调用
//
//=====================================================
//系统初始化 成功 返回1 失败返回0
int SystemState_Inite()
{
	SystemState.Enable=System_Disable;
	SystemState.Robote.Mode=Mode_Cruise;//巡航模式
	SystemState.Robote.RunState=State_AutoCTR;//自动运行状态
	SystemState.Robote.GarderFire=Fire_Permite;//开火授权
	SystemState.Task=0;
	SystemState.Beep=Beep_Silence;
	SystemState.Time=0;
	SystemState.Wifi_ctrTime=0;//上次WIFI控制时间
	SystemState.htim=&htim6;//计时器请设定 每 10us 记一个数  重载值为 100-1 (1ms)  例如 Timer3 主频168M 预分频 (840-1) 重载值 (100-1)
	HAL_TIM_Base_Start_IT(SystemState.htim);//启动时间计数器
	if(!xTaskCreate( vOutLineCheck_Task, "OutLineCheck_Task", 200, NULL, 3, NULL ))return 0;
	return 1;
}


//获得系统时间
inline float GetSystemTime()
{	
	return SystemState.htim->Instance->CNT/100.0 +SystemState.Time;
}

//刷新电机通信时间数组
void RefreshMotorOutLineTime(MotorX_NoDEF Mot_No)
{
	g_Time_MotorOutLine[Mot_No]=GetSystemTime();
}

//刷新外设通信时间时间数组
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No)
{
	g_Time_DeviceOutLine[DevX_No]=GetSystemTime();
}

//获取各任务运行周期间隔
void GetTaskPeriod(TaskX_NoDEF No)
{
	g_Time_TaskPeriod[No]=0.02f*(GetSystemTime()-g_TimePre_Taskstart[No])+0.98f*g_Time_TaskPeriod[No];
	g_TimePre_Taskstart[No]=GetSystemTime();
}

//  获取近100次运行间隔
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
	//===============

}
