#ifndef __SysState_H__
#define __SysState_H__

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "MyCom.h"
//===============================================================系统模式定义


#define SetSystemState(x) SystemState.State=SystemState.State|x //设置系统状态
#define ClearSystemState(x) SystemState.State=SystemState.State&(~x) //清除系统状态
#define GetSystemState(x) (SystemState.State&x)//获取系统状态



//==============================================================电机堵转定义

#define OutLine_Time 50 //断线检测时间

typedef enum
{
	Beep_Silence, //沉默
	Beep_Beeping, //长鸣
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
	short Mode;//运行模式
	short Enable;//状态
	short State;//状态
	short Task;//任务
	BEEPMode Beep;//蜂鸣器
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//时间计数器句柄
	long OutLine_Flag;//断线标志
	
	RobotDistDef RobotDist;//机器人测量
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


int SystemState_Inite(void);//SystemState初始化
inline void RefreshSysTime(void);//刷新系统时间（mm）
float GetSystemTimer(void);//获取系统当前准确时间
void MesureTimePeriod(void);//测量运行间隔

void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//刷新外设通信时间时间数组
void OutLine_Check(void);//断线检测检测
#endif
