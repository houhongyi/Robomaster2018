#ifndef __SysState_H
#define __SysState_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "MyCom.h"

//==============================================================电机堵转定义
#define Motor_Stall_Time 200
#define Motor_Stall_Spd 5
#define Motor_Stall_Output 2500  //电机堵转输出阈值
#define OutLine_Time 50 //断线检测时间

//===============================================================系统模式定义
#define System_Disable        0 //失能模式
//#define SystemMode_Set_Zero       1 //自动清零
//#define SystemMode_RemoteControl  2 //遥控操作
//#define SystemMode_WIFI_Control   3 //wifi操作模式
//#define SystemMode_Auto_Control   4 //自动操作模式*/

#define SystemState_Enable  1 //使能位 


#define SystemState_Normal  0 //状态正常
#define SystemState_MotorStall  1 //电机堵转
#define SystemState_MotorOutline  2 //电机断线

#define SystemTask_GetBullet  0 //获取子弹任务 第0位

#define SetSystemState(x) SystemState.State=SystemState.State|x //设置系统状态
#define ClearSystemState(x) SystemState.State=SystemState.State&(~x) //清除系统状态
#define GetSystemState(x) (SystemState.State&x)//获取系统状态


	#ifndef __MyCom_H
		#define my_abs(x) ((x)>0?(x):-(x)) //ABS宏定义
		#define MyFlagSet(x,y) x=x|(0x00000001<<y) //设置标志位  y第几位
		#define MyFlagClear(x,y) x=x&~(0x00000001<<y)
		#define MyFlagGet(x,y) (x&(0x00000001<<y))
	#endif

typedef enum
{
	Beep_Silence, //沉默
	Beep_Beeping, //长鸣
}BEEPMode;

typedef enum
{
	Mode_Cali,//校准模式
	Mode_Cruise,//巡航
	Mode_Attack,//攻击模式
	Mode_Foolish,//定点攻击模式
	Mode_Defense,//防御模式
	Mode_Crazy,//暴走模式
	Mode_YuntaiCTR,//云台控制模式模式
}GarderMode;//哨兵运行模式

typedef enum
{
	State_AutoCTR,//自动运行
	State_RemoteCTR,//遥控器运行
	State_WifiCTR,//WIFI控制模式
}GarderState;//哨兵控制状态

typedef enum
{
	Enemy_Finding=0,//正在找敌军
	Enemy_Finded,//找到敌军
}GarderFindEnemyDef;//哨兵敌军扫描状态

typedef enum
{
	Fire_Deny=0,//禁止开火
	Fire_Permite=1,//允许开火
}GarderFireDef;//哨兵开火授权

typedef enum
{
	YuntaiCTR_Deny=0,//云台拒绝控制
	YuntaiCTR_Applay=1,//云台申请控制
}YuntaiApplayDef;//云台申请


typedef struct{
	GarderMode Mode;//运行模式
	GarderState RunState;//运行状态
	GarderFindEnemyDef FindEnemy;//敌军扫描状态
	GarderFireDef GarderFire;//敌军开火授权
	YuntaiApplayDef YuntaiApplay;//云台申请控制
	short Limite_Right;//右限位
	short Limite_Left;//左限位
	short Yuntai_CTR_APP_SPD;//云台控制申请速度
	short M2006_A_Current;
	short M2006_B_Current;
}RoboteDef;


typedef struct{
	
	char State;//状态
	short Enable;
	short Task;//任务
	BEEPMode Beep;//蜂鸣器
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//时间计数器句柄
	long OutLine_Flag;//断线标志
	float Wifi_ctrTime;//WIFI上次发送命令时间
	
	RoboteDef Robote;//机器人
}SystemStateDef;

typedef enum
{
	Motor1_No=0,
	Motor2_No,

	MotorTotal_No
}MotorX_NoDEF;

typedef enum
{
	RemoteCtr_No=0,//遥控
	YUNTAICtrBoard_No,//云台板
	WIFIBoard_No,//Wifi板
	Judgment_No,//裁判系统
	Judgment_hurt_No,//裁判系统
	Motor2006CTR_No,//2006拨弹电机透传
	YAW6623_No,//Yaw6623
	
	DeviceTotal_No	
}DeviceX_NoDEF;

typedef enum
{
	PID_Task_No,
	RobotCtr_Task_No,
	MotorCheck_Task_No,
	LED_Task_No,
	Beep_Task_No,
	Test_Task_No,
	TaskTotal_No	
}TaskX_NoDEF;

extern SystemStateDef SystemState;

float GetSystemTime(void);//获取当前系统时间
void RefreshSysTime(void);//更新时间 中断刷新中调用
int SystemState_Inite(void);//系统初始化
void RefreshMotorOutLineTime(MotorX_NoDEF Mot_No);//刷新电机通信数组
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//刷新外设通信数组
void MesureTimePeriod(void);
#endif

