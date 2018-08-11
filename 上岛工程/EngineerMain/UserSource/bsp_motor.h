#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

//user include
#include "bsp_pid.h"

#define Lift_Limite 305 //抬升极限


#define Motor_En_Disable 0 //电机失能
#define Motor_En_Enable 1 //电机使能
#define Motor_En_Calibration  2 //电机已校准
#define Motor_En_Suspend 3 //电机挂起


#define Motor_State_SpdNear 1 //电机速度达到设定附近
#define Motor_State_LocNear 2 //电机位置达到设定附近
#define Motor_State_Stall   4 //电机堵转
#define Motor_State_OutLine 8 //电机断线

#define PID_Type_NULL -1 //空PID
#define PID_Type_Positional 0 //位置式PID
#define PID_Type_Incremental 1 //增量式PID

//======================================================================
#define SetMotorSpdNear(x) x=x|Motor_State_SpdNear//设定电机速度接近
#define ClearMotorSpdNear(x) x=x&(~Motor_State_SpdNear)//清除电机速度接近
#define GetMotorSpdNear(x) (x&Motor_State_SpdNear)//获取电机速度接近状态

#define SetMotorLocNear(x) x=x|Motor_State_LocNear//设定电机位置接近
#define ClearMotorLocNear(x) x=x&(~Motor_State_LocNear)//清除电机位置接近
#define GetMotorLocNear(x) (x&Motor_State_LocNear)//获取电机位置接近状态

#define SetMotorStall(x) x=x|Motor_State_Stall//设定电机堵转
#define ClearMotorStall(x) x=x&(~Motor_State_Stall)//清除电机堵转
#define GetMotorStall(x) (x&Motor_State_Stall)//获取电机堵转状态

#define SetMotorOutLine(x) x=x|Motor_State_OutLine//设定电机断线
#define ClearMotorOutLine(x) x=x&(~Motor_State_OutLine)//清除电机断线状态
#define GetMotorOutLine(x) (x&Motor_State_OutLine)//获取电机断线状态
//======================================================================

/* DEFINE ------------------------------------------------------------------*/
#define PID_kp_ki_kd -255

//up_down 升降平台

#define KP_Speed_UD 20//双环中的速度环，增量式pid参数
#define KI_Speed_UD 0.3
#define KD_Speed_UD 4

#define KP_Loction_UD 10//双环的位置环，位置式pid参数
#define KI_Loction_UD 0.01
#define KD_Loction_UD 0

#define Speed_limit_UD 400  // mm/s，用于升降机构，位置环输出处限幅 
#define Motor_output_limit_UD 15000

#define Loction_pid_Ilimit 100

#define MOTOR_STRUCT_Lead 5.622f//导程，就是无刷转一圈，执行机构走了多少 /3。14
#define MOTOR_STRUCT_Step 8191 //无刷屁股转一圈，返回的最大机械角度

//YunTai 云台电机

#define KP_Speed_YT 0
#define KI_Speed_YT 0
#define KD_Speed_YT 0

#define KP_Loction_YT 0
#define KI_Loction_YT 0
#define KD_Loction_YT 0

#define Speed_limit_YT 360  // deg/s，用于云台 速度限幅
#define Motor_output_limit_YT 4000

//Chassis 车底盘
#define KP_Speed_CH 20
#define KI_Speed_CH 0.8
#define KD_Speed_CH 4

#define KP_Loction_CH 1
#define KI_Loction_CH 1
#define KD_Loction_CH 1

#define Motor_output_limit_CH 8000


typedef enum
{
	MotorSPD_Ctr=0,
	MotorLOC_Ctr
}MotorPID_TypeDef;

typedef struct 
{
	int No;//电机序号 
	int State;//电机状态
	float Spd;//运行速度 （mm/s）
	float Loc;//当前位置 （mm）
	int Pre_MLoc;//上次转子位置
	
	char temprature;//温度
	float Current;// 电流
}MotorX_StateDef;

typedef struct 
{
	int En;//电机使能标志  
	float Spd;//运行速度 （mm/s）
	float Acc;//加速度 (mm/s/s)
	float Loc;//运行距离 （mm）
}MotorX_ComDef;

typedef struct 
{
	PID moter_pid;
	PID_TYPE motor_pid_type;
	float memo_I;//需要记录的累加量
}MotorX_PIDDef;

typedef struct
{
	float Lead;//导程
	short Step;//一圈的最大机械角度
}MotorX_StructDef;

typedef struct
{
	short OutPut_Stall;//堵转
	short OutPut_Limite;//输出限幅
}MotorX_ProtectDef;

typedef struct
{
	MotorX_StateDef State;
	MotorX_ComDef Com;
	MotorX_PIDDef SPD_PID;
	MotorX_PIDDef LOC_PID;
	MotorX_StructDef Struct;
	MotorX_ProtectDef Protect;
	
	//为了使用时候方便，一些变量放在这里
	MotorPID_TypeDef pid_control_type;
	short Motor_output;//can send date

}MotorXDef;

typedef enum
{
	MotorPur_Cheassis,//地盘电机
	MotorPur_Lift,//提升电机
	MotorPur_Yuntai,//云台电机
	MotorPur_Rotate,//翻转电机
}MotorPurpose_TypeDef;//电机用途

extern MotorXDef Motor1,Motor2,Motor3,Motor4,MotorA,MotorB,MotorC,MotorD,Motor5,Motor6;


void Motor_Inite_All_up_dowm(MotorXDef *motorxdef);
void Motor_Inite_All_Chassis(MotorXDef *motorxdef);
void Motor_Inite_All_YunTai(MotorXDef *motorxdef);
void Motor_Inite_All_Chassis_Little(MotorXDef *motorxdef);
void Motor_PID_change(MotorXDef *motorxdef, float kp_speed, float ki_speed, float kd_speed, float kp_loction, float ki_loction, float kd_loction);
void Motor_control_UD(MotorXDef *motorxdef, MotorPID_TypeDef pid_control_type_t);
void Motor_Zero(MotorXDef* Mot);
void MotorLocBalence_SpdEqualize(MotorXDef* motorxA,MotorXDef* motorxB);
void SpeedDistribute(void);//速度分解函数
void MoveMotor_EN_ALLSET(char en);
void LiftMotor_EN_ALLSET(char en);
void YuntaiMotor_EN_ALLSET(char en);
char MotorProtect(MotorXDef *motorxdef);//电机保护函数

void GetMotorState(unsigned char* data, MotorXDef* Mot);//获取电机运行状态
int Motor_Inite_ALL(void);//初始化电机并启动PID任务

#endif
