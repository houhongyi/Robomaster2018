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
#define SystemTask_UpIsland  1  //上岛任务 第1位
#define SystemTask_DownIsland  2 //下岛任务 第2位
#define SystemTask_AutoMove  3 //自动移动 第3位

#define SetSystemState(x) SystemState.State=SystemState.State|x //设置系统状态
#define ClearSystemState(x) SystemState.State=SystemState.State&(~x) //清除系统状态
#define GetSystemState(x) (SystemState.State&x)//获取系统状态



//==============================================================电机堵转定义
#define Motor_Stall_Time 200
#define Motor_Stall_Spd 5
#define OutLine_Time 50 //断线检测时间

typedef enum
{
	None=0,
	Forward=1,//前进
	Backward=-1,//后退
}Direct_enum;//方厦毒衮

typedef enum
{
	Beep_Silence, //沉默
	Beep_Beeping, //长鸣
}BEEPMode;

typedef enum
{
	UDMode_None=0,
	UDMode_UpIsland,
	UDMode_DownIsland,
}UDIslanMode;

typedef struct{
	float Camera_X;
	float Camera_Y;
	float Camera_Z;
}SensorDex;

typedef struct{
	short State_Yaw;
	short State_Pich;
	float Com_Yaw;
	float Com_Pich;
	Direct_enum Direct;
}YunTaiDex;

typedef struct{
	short Dis_LF;
	short Dis_RF;
	short Dis_LB;
	short Dis_RB;
	short Dis_X1;
	short Dis_X2;
}DistSensorDex;

typedef struct{
	TaskHandle_t hTask_UpIsland;
	TaskHandle_t hTask_DownIsland;
	TaskHandle_t hTask_getbullet;
	TaskHandle_t hTask_automove;
}TaskHandleDex;

typedef struct{
	short Catch;//夹子
	short Pump;//真空泵
	short Magazine;//弹仓
	short ReliefFrame;//救援架
	short ReliefFrame2;//救援架 备用
	short ReliefFinger;//救援手指
	short Camara_LED;//灯光
	short AddHPCard;//加血卡
	short EleBreak;//电磁制动器
	short Supplyer;//
	short GetBulletFrame;//获取子弹提升架
	float Move_X_SPD;//横移速度 右为正
	float Move_Y_SPD;//前进速度 前为正
	float Move_Z_SPD;//旋转速度 顺时针为正
	
	//short Lift_Catch;//放松为0 夹取为1
	float Lift_Z_Loc;//提升架高度
	float Lift_A_Loc;//弹箱旋转角度
	float Lift_Y_Loc;//悬架前进距离
	float Lift_Y_SPD;//悬架前进速度
	float Guide_Frame;//导流架角度
	YunTaiDex YunTai;
}RobotDef;

typedef struct{
	short Mode;//运行模式
	short Enable;//状态
	short State;//状态
	short Task;//任务
	short GetBulletFinish;//是否完成取弹 用于控制车体自动移动
	BEEPMode Beep;//蜂鸣器
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//时间计数器句柄
	long OutLine_Flag;//断线标志
	
	UDIslanMode UDMode;//上下岛模式
	DistSensorDex DistSensor;//距离传感器数据
	SensorDex Sensor;//传感器数据
	RobotDef Robot;//机器人控制
	TaskHandleDex TaskHandle;//任务句柄
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
	
	Motor5_No,
	Motor6_No,
	
	MotorTotal_No
}MotorX_NoDEF;

typedef enum
{
	RemoteCtr_No,//遥控
	ExtendCtr_No,//扩展版
	ExtendCtr2_No,//扩展版2
	DisMesur_No,//测距板
	YuntaiCtr_No,//云台控制板
	
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

void YuntaiMsg_GetState(unsigned char* s);//获取云台当前角度

#endif
