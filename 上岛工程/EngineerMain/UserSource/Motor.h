#ifndef __Motor_H
#define __Motor_H

#define Motor_En_Disable 0 //电机失能
#define Motor_En_Enable 1 //电机使能

#define Motor_State_SpdNear 1 //电机速度达到设定附近
#define Motor_State_LocNear 2 //电机位置达到设定附近

#define PID_Type_NULL -1 //空PID
#define PID_Type_Positional 0 //位置式PID
#define PID_Type_Incremental 1 //增量式PID

#define SetMotorSpdNear(x) x=x|Motor_State_SpdNear//设定电机速度接近
#define ClearMotorSpdNear(x) x=x&(~Motor_State_SpdNear)//清除电机速度接近
#define GetMotorSpdNear(x) x&Motor_State_SpdNear//获取电机速度接近状态
#define SetMotorLocNear(x) x=x|Motor_State_LocNear//设定电机位置接近
#define ClearMotorLocNear(x) x=x&(~Motor_State_LocNear)//清除电机位置接近
#define GetMotorLocNear(x) x&Motor_State_LocNear//获取电机位置接近状态

typedef struct 
{
	short No;//电机序号 
	short State;//电机状态
	float Spd;//运行速度 （mm/s）
	float Loc;//当前位置 （mm）
	char temprature;//温度
	float Current;//电流
	short Pre_MLoc;//上次转子位置

}MotorX_StateDef;

typedef struct 
{
	short En;//电机使能标志  
	float Spd;//运行速度 （mm/s）
	float Acc;//加速度 (mm/s/s)
	float Loc;//运行距离 （mm）
}MotorX_ComDef;

typedef struct 
{
	short type;//PID形式
	float K_P;//运行速度 （mm/s）
	float K_I;//加速度 (mm/s/s)
	float K_D;//运行距离 （mm）
	float pre_Output;//上次输出
	float pre_erro;//上次误差
	float ppre_erro;//上上次蟛钗
	float memo_I;//需要记录的累加量
}MotorX_PIDDef;

typedef struct
{
	short Lead;//导程
	short Step;//一圈的转数
}MotorX_StructDef;

typedef struct
{
	MotorX_StateDef State;
	MotorX_ComDef Com;
	MotorX_PIDDef SPD_PID;
	MotorX_PIDDef LOC_PID;
	MotorX_StructDef Struct;
	short output;
}MotorXDef;

extern MotorXDef Motor1,Motor2,Motor3,Motor4,MotorA,MotorB,MotorC,MotorD,MotorE,MotorF;



void Motor_Inite(void);//电机初始化
void GetMotorState(unsigned char* data,MotorXDef* Mot);//Can信息提取电机信息
void Motor_Zero(MotorXDef* Mot);//电机设零
#endif
