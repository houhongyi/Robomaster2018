#ifndef __SysState_H__
#define __SysState_H__

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "bsp_motor.h"
#include "MyCom.h"
//===============================================================系统模式定义
#define SystemMode_Remote_MoveControl  0 //遥控操作地盘模式
#define SystemMode_Remote_LiftControl  1 //遥控操作升降模式
#define SystemMode_KeyBoard_Control  2 //键盘操作模式
#define SystemMode_Auto_Control  3 //自动操作模式
#define SystemMode_Set_Zero      4 //自动清零
#define SystemMode_Disable       5 //失能模式

#define SystemState_Enable  1 //使能位 
#define SystemState_Enable_Move  2 //地盘使能位
#define SystemState_Enable_Lift  4 //抬升使能位

#define SystemState_Normal  0 //状态正常
#define SystemState_MotorStall  1 //电机堵转
#define SystemState_MotorOutline  2 //电机断线

#define SystemTask_GetBullet  0 //获取子弹任务 第0位

#define SetSystemState(x) SystemState.State=SystemState.State|x //设置系统状态
#define ClearSystemState(x) SystemState.State=SystemState.State&(~x) //清除系统状态
#define GetSystemState(x) (SystemState.State&x)//获取系统状态



//==============================================================电机堵转定义
#define Motor_Stall_Time 200
#define Motor_Stall_Spd 5
#define Motor_Stall_Output 4000  //电机堵转输出阈值
#define OutLine_Time 50 //断线检测时间

typedef enum
{
	Beep_Silence, //沉默
	Beep_Beeping, //长鸣
}BEEPMode;

typedef struct{
	float Camera_X;
	float Camera_Y;
	float Camera_Z;
}SensorDex;

typedef struct{
	short Catch;//夹子
	short Pump;//真空泵
	short Magazine;//弹仓
	short Magazine_Shell;//弹仓盖
	short ReliefFrame;//救援架
	short Electromagnet;//电磁铁
	short Camara_LED;//灯光
	short AddHPCard;//加血卡
	float Move_X_SPD;//横移速度 右为正
	float Move_Y_SPD;//前进速度 前为正
	float Move_Z_SPD;//旋转速度 顺时针为正
	
	short Lift_Catch;//放松为0 夹取为1
	float Lift_Z_Loc;//提升机构高度
	float Lift_X_Loc;//提升机构前进位置
	float Lift_A_Loc;//弹箱角度
	
}RobotDef;

typedef struct{
	short Mode;//运行模式
	short Enable;//状态
	short State;//状态
	short Task;//任务
	BEEPMode Beep;//蜂鸣器
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//时间计数器句柄
	long OutLine_Flag;//断线标志
	
	SensorDex Sensor;//传感器数据
	RobotDef Robot;//机器人控制
}SystemStateDef;

typedef enum
{
	Motor1_No,
	Motor2_No,
	Motor3_No,
	Motor4_No,
	MotorA_No,
	MotorB_No,
	MotorC_No,
	MotorD_No,
	MotorE_No,
	MotorF_No,
	
	MotorTotal_No
}MotorX_NoDEF;

typedef enum
{
	RemoteCtr_No,//遥控
	ExtendCtr_No,//扩展版
	ExtendCtr2_No,//扩展版2
	
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


extern float g_Time_MotorStall[MotorTotal_No];
extern float g_Time_MotorOutLine[MotorTotal_No];
extern float g_Time_TaskPeriod[TaskTotal_No];//各任务运行间隔
extern float g_TimePre_Taskstart[TaskTotal_No];//各任务运行启动时间

extern SystemStateDef SystemState;
extern MotorXDef* Motors[MotorTotal_No];

int SystemState_Inite(void);//SystemState初始化
inline void RefreshSysTime(void);//刷新系统时间（mm）
float GetSystemTimer(void);//获取系统当前准确时间
void MesureTimePeriod(void);//测量运行间隔
void GetTaskPeriod(TaskX_NoDEF No);//测量任务运行间隔




void RefreshMotorOutLineTime(MotorX_NoDEF Mot_No);//刷新电机通信时间数组
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//刷新外设通信时间时间数组
void MotorStall_Check(void);//电机堵转检测检测
void OutLine_Check(void);//断线检测检测
#endif
