/**
******************************************************************************
* @file     bsp_motor.c
* @author   zyc hhy
* @version  1.0
* @date     2018.03.23
* @brief
******************************************************************************
* @attention

******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/

//math
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
//user
#include "bsp_pid.h"
#include "bsp_motor.h"

//========== other Lib=======
#include "Remote.h"
#include "SystemState.h"
#include "bsp_can.h"


MotorXDef Motor1,Motor2,Motor3,Motor4,MotorA,MotorB,MotorC,MotorD,MotorE,MotorF;

//=====================================================
//						 
// 												内部函数
//
//
//=====================================================



/**
* @brief  电机结构体内变量初始化
* @param  MotorXDef *motorxdef :输入为电机结构体，MortorA ~ MortorZ
* @retval None
* @note   需要注意的是：
*			1、	 作为平台升降机构，可能需要在位置环的输出处限幅，Speed_limit_UD，防止位置环计算的期望速度太大
*			2、  类似于积分限幅，输出限幅，需要在调参数之后，在宏定义中更改，而且增量式pid中，没有写积分限幅
*
*/

void Motor_Inite_All_up_dowm(MotorXDef *motorxdef)
{
	//state MotorPur_Lift
	motorxdef->State.No=MotorPur_Lift;//点击用途 提升机构
	motorxdef->State.State = 0;
	motorxdef->State.Spd = 0;
	motorxdef->State.Loc = 0;
	motorxdef->State.Pre_MLoc = 0;

	//com
	motorxdef->Com.Acc = 0;
	motorxdef->Com.En = Motor_En_Disable;
	motorxdef->Com.Loc = 0;
	motorxdef->Com.Spd = 0;	
	
	//struct
	motorxdef->Struct.Lead = MOTOR_STRUCT_Lead;
	motorxdef->Struct.Step = MOTOR_STRUCT_Step;

	motorxdef->Motor_output=0;
	//pid_control_type
	motorxdef->pid_control_type = 0;
	
	//pid
	motorxdef->LOC_PID.motor_pid_type = Positional_PID;
	motorxdef->LOC_PID.memo_I = 0;

	motorxdef->SPD_PID.motor_pid_type = Incremental_PID;
	motorxdef->SPD_PID.memo_I = 0;

	motorxdef->LOC_PID.moter_pid.P = KP_Loction_UD;
	motorxdef->LOC_PID.moter_pid.I = KI_Loction_UD;
	motorxdef->LOC_PID.moter_pid.D = KD_Loction_UD;
	motorxdef->SPD_PID.moter_pid.P = KP_Speed_UD;
	motorxdef->SPD_PID.moter_pid.I = KI_Speed_UD;
	motorxdef->SPD_PID.moter_pid.D = KD_Speed_UD;

	motorxdef->LOC_PID.moter_pid.U = 0;
	motorxdef->SPD_PID.moter_pid.U = 0;
	motorxdef->LOC_PID.moter_pid.E = 0;
	motorxdef->SPD_PID.moter_pid.E = 0;
	motorxdef->LOC_PID.moter_pid.PreE = 0;
	motorxdef->SPD_PID.moter_pid.PreE = 0;
	motorxdef->LOC_PID.moter_pid.PrePreE = 0;
	motorxdef->SPD_PID.moter_pid.PrePreE = 0;

	motorxdef->LOC_PID.moter_pid.Ulimit = Speed_limit_UD;
	motorxdef->SPD_PID.moter_pid.Ulimit = Motor_output_limit_UD;
	motorxdef->LOC_PID.moter_pid.Ilimit = Loction_pid_Ilimit;
	//motorxdef->SPD_PID.moter_pid.Ilimit = Motor_output_limit_UD;

}

void Motor_Inite_All_Chassis(MotorXDef *motorxdef)
{
	//state MotorPur_Cheassis
	motorxdef->State.No=MotorPur_Cheassis;//点击用途 地盘驱动
	motorxdef->State.State = 0;
	motorxdef->State.Spd = 0;
	motorxdef->State.Loc = 0;
	motorxdef->State.Pre_MLoc = 0;

	//com
	motorxdef->Com.Acc = 0;
	motorxdef->Com.En = Motor_En_Disable;
	motorxdef->Com.Loc = 0;
	motorxdef->Com.Spd = 0;	
	
	//struct
	motorxdef->Struct.Lead = 24.8;
	motorxdef->Struct.Step = MOTOR_STRUCT_Step;

	motorxdef->Motor_output=0;
	//pid_control_type
	motorxdef->pid_control_type = 0;
	
	//pid
	motorxdef->LOC_PID.motor_pid_type = Positional_PID;
	motorxdef->LOC_PID.memo_I = 0;

	motorxdef->SPD_PID.motor_pid_type = Incremental_PID;
	motorxdef->SPD_PID.memo_I = 0;

	motorxdef->LOC_PID.moter_pid.P = KP_Loction_UD;
	motorxdef->LOC_PID.moter_pid.I = KI_Loction_UD;
	motorxdef->LOC_PID.moter_pid.D = KD_Loction_UD;
	motorxdef->SPD_PID.moter_pid.P = KP_Speed_CH;
	motorxdef->SPD_PID.moter_pid.I = KI_Speed_CH;
	motorxdef->SPD_PID.moter_pid.D = KD_Speed_CH;

	motorxdef->LOC_PID.moter_pid.U = 0;
	motorxdef->SPD_PID.moter_pid.U = 0;
	motorxdef->LOC_PID.moter_pid.E = 0;
	motorxdef->SPD_PID.moter_pid.E = 0;
	motorxdef->LOC_PID.moter_pid.PreE = 0;
	motorxdef->SPD_PID.moter_pid.PreE = 0;
	motorxdef->LOC_PID.moter_pid.PrePreE = 0;
	motorxdef->SPD_PID.moter_pid.PrePreE = 0;

	motorxdef->LOC_PID.moter_pid.Ulimit = Speed_limit_UD;
	motorxdef->SPD_PID.moter_pid.Ulimit = Motor_output_limit_CH;
	motorxdef->LOC_PID.moter_pid.Ilimit = Loction_pid_Ilimit;
	//motorxdef->SPD_PID.moter_pid.Ilimit = Motor_output_limit_UD;

}


/**
* @brief  设置所有底盘电机使能位
* @param  
* @retval
* @note  
*/
void MoveMotor_EN_ALLSET(char en)
{
	Motor1.Com.En=en;
	Motor2.Com.En=en;
	Motor3.Com.En=en;
	Motor4.Com.En=en;

}

/**
* @brief  设置所有提升电机使能位
* @param  
* @retval
* @note  
*/
void LiftMotor_EN_ALLSET(char en)
{
	MotorA.Com.En=en;
	MotorB.Com.En=en;
	MotorC.Com.En=en;
	MotorD.Com.En=en;
	MotorE.Com.En=en;
	MotorF.Com.En=en;
}
/**
* @brief  PID系数初始化
* @param  MotorXDef *motorxdef :输入为电机结构体，MortorA ~ MortorZ
* @retval None
* @note   需要注意的是：
*			1、  kp，ki，kd这几个变量如果为 -255，则使用默认参数，如果为正常参数，则正常。
*/

void Motor_PID_change(MotorXDef *motorxdef, float kp_speed, float ki_speed, float kd_speed, float kp_loction, float ki_loction, float kd_loction)
{

	if (kp_speed < 0)
	{
		motorxdef->SPD_PID.moter_pid.P = KP_Speed_UD;
	}
	else
	{
		motorxdef->SPD_PID.moter_pid.P = kp_speed;
	}

	if (ki_speed < 0)
	{
		motorxdef->SPD_PID.moter_pid.I = KI_Speed_UD;
	}
	else
	{
		motorxdef->SPD_PID.moter_pid.I = ki_speed;
	}

	if (kd_speed < 0)
	{
		motorxdef->SPD_PID.moter_pid.D = KD_Speed_UD;
	}
	else
	{
		motorxdef->SPD_PID.moter_pid.D = kd_speed;
	}

	if (kp_loction < 0)
	{
		motorxdef->LOC_PID.moter_pid.P = KP_Loction_UD;
	}
	else
	{
		motorxdef->LOC_PID.moter_pid.P = kp_loction;
	}

	if (ki_loction < 0)
	{
		motorxdef->LOC_PID.moter_pid.I = KI_Loction_UD;
	}
	else
	{
		motorxdef->LOC_PID.moter_pid.P = ki_loction;
	}

	if (kd_loction < 0)
	{
		motorxdef->LOC_PID.moter_pid.D = KD_Loction_UD;
	}
	else
	{
		motorxdef->LOC_PID.moter_pid.P = kd_loction;
	}

}

/**
* @brief  电机设零
* @param  
* @retval None
* @note   
*/
void Motor_Zero(MotorXDef* motorxdef)
{
	motorxdef->State.Loc = 0;
}

/**
* @brief  电机路程平衡速度补偿
* @param  motorxA 同步目标电机     motorxB 被同步电机
* @retval None
* @note   
*/
void MotorLocBalence_SpdEqualize(MotorXDef* motorxA,MotorXDef* motorxB)
{
	float loc_e=motorxA->State.Loc-motorxB->State.Loc;//获取位置误差
	float equalize_k=loc_e*1;//速度补偿
	
	if(equalize_k<-20)equalize_k=-20;//速度补偿限幅  20
	if(equalize_k>20)equalize_k=20;
	motorxB->Com.Spd+=equalize_k;
	//motorxA->State.Loc = 0;

}





/**
* @brief  升降平台电机控制函数
* @param  MotorXDef *motorxdef :输入为电机结构体，MortorA ~ MortorZ 
* @param   unsigned int pid_control_type_t: use to judge "only speed_pid" or " loction_pid and speed_pid" 
*          0:only speed_pid
*		   1:loction_pid and speed_pid
* @retval None
* @note   需要注意的是：
*         调用此函数前，需要其他函数获取 
*			motorxdef->Com.Spd、  速度期望							 
*			motorxdef->State.Spd、速度反馈								 
*			motorxdef->Com.Loc、  位置期望									 
*			motorxdef->State.loc、位置反馈										 
*			motorxdef->pid_control_type、手控or电脑控，手控只有速度环，电脑控是位置速度环结合
*		 
*/
void  Motor_control_UD(MotorXDef *motorxdef, MotorPID_TypeDef pid_control_type_t)
{
	//Local variable initialization
	float expected_value_t = 0;
	float actual_value_t = 0;
	//uint8_t onece_flag = 0;
	//read date from motor

	// pid_control
	if(MotorProtect(motorxdef))return; //电机保护函数
	
	
	if (pid_control_type_t == MotorSPD_Ctr)//only speed_pid
	{

		//Motor_PID_change(motorxdef, KP_single_Speed_UD, KI_single_Speed_UD, KD_single_Speed_UD, -255, -255, -255);
		expected_value_t = motorxdef->Com.Spd;
		actual_value_t = motorxdef->State.Spd;
		motorxdef->Motor_output = PID_Control_normal(&motorxdef->SPD_PID.moter_pid, expected_value_t, actual_value_t, Incremental_PID);

	}
	if (pid_control_type_t == MotorLOC_Ctr)//loction_pid and speed_pid
	{
		//Motor_PID_change(motorxdef, KP_Speed_UD, KI_Speed_UD, KD_Speed_UD, -255, -255, -255);

		expected_value_t = motorxdef->Com.Loc;
		actual_value_t = motorxdef->State.Loc;
		float loction_pid_output = PID_Control_normal(&motorxdef->LOC_PID.moter_pid, expected_value_t, actual_value_t, Positional_PID);//升降平台 位置环用位置式pid

		actual_value_t = motorxdef->State.Spd;
		motorxdef->Motor_output = PID_Control_normal(&motorxdef->SPD_PID.moter_pid, loction_pid_output, actual_value_t, Incremental_PID);
	}
}

//电机保护函数
char MotorProtect(MotorXDef *motorxdef)
{
	//电机失能
	if(motorxdef->Com.En==Motor_En_Disable)
	{
		motorxdef->Motor_output=0;
		motorxdef->SPD_PID.moter_pid.U=0;
		motorxdef->SPD_PID.moter_pid.E=0;
		motorxdef->SPD_PID.moter_pid.PreE=0;
		motorxdef->SPD_PID.moter_pid.PrePreE=0;
		motorxdef->LOC_PID.moter_pid.Intergral=0;
		motorxdef->LOC_PID.moter_pid.U=0;
		return 1;
	}
	
	//堵转保护 或者 自动清零
	if(GetMotorStall(motorxdef->State.State)||SystemState.Mode==SystemMode_Set_Zero)
	{
		motorxdef->SPD_PID.moter_pid.Ulimit=Motor_Stall_Output+100; //设定堵转限幅
	}
	else
	{
		switch(motorxdef->State.No)//依据电机类型恢复电流限幅
		{
			case MotorPur_Cheassis:
				motorxdef->SPD_PID.moter_pid.Ulimit=Motor_output_limit_CH;//恢复地盘电机限幅
				break;
			case MotorPur_Lift:
				motorxdef->SPD_PID.moter_pid.Ulimit=Motor_output_limit_UD;//恢复提升电机限幅
				break;				
		}
	}
	return 0;
}

//给电机输出加入死区   data 带放入数组的首地址  output 电机输出  deadzone 死区
void AddDeadZone(unsigned char*data,short output,short deadzone)
{
	if(output>0)output+=deadzone;
	if(output<0)output-=deadzone;
	//*(short*)data = ((short)output>>8) | ((short)output&0x00ff)<<8;
	*data=(char)(output>>8);
	*(data+1)=(char)(output&0x00ff);
}

//发送控制指令 PIDTask 中调用
void SentControlData()
{
	//  遥控器 RC_Data.switch_right==2 就是失能
		unsigned char data[8]={0};
		//==============================地盘
		AddDeadZone(&data[0],(short)Motor1.Motor_output,200);
		AddDeadZone(&data[2],(short)Motor2.Motor_output,200);
		AddDeadZone(&data[4],(short)Motor3.Motor_output,200);
		AddDeadZone(&data[6],(short)Motor4.Motor_output,200);

		if(!(SystemState.Enable&SystemState_Enable_Move)||RC_Data.switch_right==2)
			 memset(data,0,8);
		Can_Sent_msgToQue(1,0x1FF,data);//地盘电机编号应该是5~8  ID：0x1ff 修改Can接受那边参数
		
		//==============================升降平台
		memset(data,0,8);
		AddDeadZone(&data[0],(short)MotorA.Motor_output,200);
		AddDeadZone(&data[2],(short)MotorB.Motor_output,200);

		if(!(SystemState.Enable&SystemState_Enable_Lift)||RC_Data.switch_right==2)
			 memset(data,0,8);
		Can_Sent_msgToQue(1,0x200,data);//一级抬升电机
		
		memset(data,0,8);
		AddDeadZone(&data[0],(short)MotorE.Motor_output,200);
		AddDeadZone(&data[2],(short)MotorF.Motor_output,200);
		AddDeadZone(&data[4],(short)MotorC.Motor_output,200);
		AddDeadZone(&data[6],(short)MotorD.Motor_output,200);
		if(!(SystemState.Enable&SystemState_Enable_Lift)||RC_Data.switch_right==2)
			 memset(data,0,8);
		Can_Sent_msgToQue(2,0x200,data);//二级抬升以及伸缩旋转
		
		//扩展版1 夹子与灯光
		memset(data,0,8);
		AddDeadZone(&data[0],SystemState.Robot.Catch,0);
		AddDeadZone(&data[2],SystemState.Robot.Pump,0);
		AddDeadZone(&data[4],SystemState.Robot.Magazine,0);
		AddDeadZone(&data[6],SystemState.Robot.Magazine_Shell,0);
		Can_Sent_msgToQue(2,CAN_EXTEND_RECEIVE_ID,data);//扩展版 夹持与真空泵
		
		//扩展版2 救援架 加血卡
		memset(data,0,8);
		AddDeadZone(&data[0],SystemState.Robot.ReliefFrame,0);
		AddDeadZone(&data[6],SystemState.Robot.AddHPCard,0);
		
		Can_Sent_msgToQue(1,CAN_EXTEND2_RECEIVE_ID,data);//扩展版 夹持与真空泵

}
//=================================================================
//
//								    任务函数
//
//
//=================================================================

//电机PID控制任务
void vMotorPIDCtr_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	vTaskDelay(1000 / portTICK_RATE_MS);//延迟启动1s
	MotorPID_TypeDef Lift_PIDCtr_Type=MotorSPD_Ctr;

	while(1)//电机控制
	{
		GetTaskPeriod(PID_Task_No);//获取运行间隔
		
		//=======================计算控制量===============
		//地盘电机PID控制(纯速度环)
		Motor_control_UD(&Motor1,MotorSPD_Ctr);
		Motor_control_UD(&Motor2,MotorSPD_Ctr);
		Motor_control_UD(&Motor3,MotorSPD_Ctr);
		Motor_control_UD(&Motor4,MotorSPD_Ctr);
		
		switch(SystemState.Mode)
		{
			case SystemMode_Set_Zero://电机复位时进行速度控制
				Lift_PIDCtr_Type=MotorSPD_Ctr;
			break;
			default:
				Lift_PIDCtr_Type=MotorLOC_Ctr;//位置控制
			break;
		}
		Motor_control_UD(&MotorA,Lift_PIDCtr_Type);//一级抬升
		Motor_control_UD(&MotorB,Lift_PIDCtr_Type);//一级抬升
		Motor_control_UD(&MotorC,Lift_PIDCtr_Type);//二级抬升
		Motor_control_UD(&MotorD,Lift_PIDCtr_Type);//二级抬升
		Motor_control_UD(&MotorE,Lift_PIDCtr_Type);//抓取机构前伸
		Motor_control_UD(&MotorF,Lift_PIDCtr_Type);//弹箱旋转
		
		//=======================发送控制指令=============
		SentControlData();//通过can发送控制指令 
		vTaskDelayUntil(&xLastWakeTime,5/ portTICK_RATE_MS);
	}
}


//=================================================================
//
//								外部调用函数
//
//
//=================================================================

//获取电机状态  在BSP_CAN.c调用
void GetMotorState(unsigned char* data, MotorXDef* motorxdef)
{
	short now_MLoc = ((short)data[0] << 8) | data[1];
	short pre_MLoc = motorxdef->State.Pre_MLoc;
	float tem_e;
	float Loc_e = now_MLoc - pre_MLoc;//路程差

	if (Loc_e < (-0.66*motorxdef->Struct.Step))Loc_e += motorxdef->Struct.Step;
	if (Loc_e > (0.66*motorxdef->Struct.Step))Loc_e -= motorxdef->Struct.Step;

	motorxdef->State.Loc += Loc_e / motorxdef->Struct.Step*motorxdef->Struct.Lead;//获取位置(mm)
	motorxdef->State.Spd = (short)((data[2] << 8) | data[3])*1.0 * motorxdef->Struct.Lead / 60;//获取速度(mm/s)
	motorxdef->State.Current = (short)((data[4] << 8) | data[5]) / 819.2;//获取电流(mm/s)
	motorxdef->State.temprature = data[6];//获取温度
	motorxdef->State.Pre_MLoc = now_MLoc;//将本次转子位置记录

	tem_e = motorxdef->State.Loc - motorxdef->Com.Loc;//位置差
	if (tem_e*tem_e < 25)
		SetMotorLocNear(motorxdef->State.State);
	else
		ClearMotorLocNear(motorxdef->State.State);

	tem_e = motorxdef->State.Spd - motorxdef->Com.Spd;//速度差
	if (tem_e*tem_e < 100)
		SetMotorSpdNear(motorxdef->State.State);
	else
		ClearMotorSpdNear(motorxdef->State.State);
}


/**
* @brief  初始化所有电机
* @param  MotorXDef *motorxdef :输入为电机结构体，MortorA ~ MortorZ
* @retval None
* @note  
*/

int Motor_Inite_ALL()
{
	Motor_Inite_All_up_dowm(&MotorA);
	Motor_Inite_All_up_dowm(&MotorB);
	Motor_Inite_All_up_dowm(&MotorC);
	Motor_Inite_All_up_dowm(&MotorD);
	Motor_Inite_All_up_dowm(&MotorE);
	Motor_Inite_All_up_dowm(&MotorF);
	MotorF.Struct.Lead=13.333;//电机E的导程为  13.333度/r
	
	Motor_Inite_All_Chassis(&Motor1);
	Motor_Inite_All_Chassis(&Motor2);
	Motor_Inite_All_Chassis(&Motor3);
	Motor_Inite_All_Chassis(&Motor4);
	
	if(!xTaskCreate( vMotorPIDCtr_Task, "MotorPIDCtr_Task", 200, NULL, 5, NULL ))return 0;
	return 1;

}


/******************************************************************************
* @attention
*			速度分解：
*					  1、轮子布局    1    2
*						             	 3    4
*					  2、车体坐标系 笛卡尔坐标系，横坐标x朝右，纵坐标y朝上
*					  3、速度分解公式
									w1			+1	+1	+(L1+L2)
									w2			-1	+1	-(L1+L2)  VX
									w3  = 1/R *	-1	+1	+(L1+L2)  VY
									w4			+1	+1	-(L1+L2)  W
******************************************************************************/
/**
* @brief 全方位轮速度分解
* @param
* @retval None
* @note  根据摇杆的xyw三个方向的给定量，分解到四个电机上
*        需要注意的是，遥控器给定的是实际量，实际的车身x,y,w,单位是mm/s，mm/s,rad/s
*/

//3、速度分解系数
#define Y_MecWheel 1.0f
#define X_MecWheel 1.0f
#define RobW_MecW  0.4875f // RobW_MecW =（车体长度+车体宽度）/ 2 /1000

void SpeedDistribute()
{
    Motor1.Com.Spd = +SystemState.Robot.Move_X_SPD * X_MecWheel + SystemState.Robot.Move_Y_SPD *Y_MecWheel + SystemState.Robot.Move_Z_SPD * RobW_MecW;
    Motor2.Com.Spd = -SystemState.Robot.Move_X_SPD * X_MecWheel + SystemState.Robot.Move_Y_SPD *Y_MecWheel - SystemState.Robot.Move_Z_SPD * RobW_MecW;
		Motor3.Com.Spd = -SystemState.Robot.Move_X_SPD * X_MecWheel + SystemState.Robot.Move_Y_SPD *Y_MecWheel + SystemState.Robot.Move_Z_SPD * RobW_MecW;
    Motor4.Com.Spd = +SystemState.Robot.Move_X_SPD * X_MecWheel + SystemState.Robot.Move_Y_SPD *Y_MecWheel - SystemState.Robot.Move_Z_SPD * RobW_MecW;

    Motor2.Com.Spd = 0 - Motor2.Com.Spd;
    Motor4.Com.Spd = 0 - Motor4.Com.Spd;
}

