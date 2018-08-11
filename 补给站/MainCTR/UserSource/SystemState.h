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

#define OutLine_Time 100 //断线检测时间

typedef enum
{
	Beep_Silence, //沉默
	Beep_Beeping, //长鸣
}BEEPMode;

typedef struct
{
	float Engneer_communi_t;
	float Hero_communi_t;
}NIR_Commui_tDef;

typedef struct
{
	char Cylinder_UL;
	char Cylinder_UR;
	char Cylinder_L;
	char Cylinder_R;
	char Cylinder_M;
}CylindeDef;


typedef struct{
	short Mode;//运行模式
	short Enable;//状态
	short State;//状态
	short Task;//任务
	BEEPMode Beep;//蜂鸣器
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//时间计数器句柄
	long OutLine_Flag;//断线标志
	CylindeDef Cylinde;//气缸状态
	NIR_Commui_tDef NIR_Commui_t;//最近一次红外通信时间
}SystemStateDef;


typedef enum
{
	MosCtr_Heart,
	Motor_Lift,
	NFC_Reader_L,
	NFC_Reader_R,
	NFC_Reader_LCard,
	NFC_Reader_RCard,
	NIR_Reader,
	
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
