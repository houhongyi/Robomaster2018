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
// 												�ڲ�����
//
//
//=====================================================



/**
* @brief  ����ṹ���ڱ�����ʼ��
* @param  MotorXDef *motorxdef :����Ϊ����ṹ�壬MortorA ~ MortorZ
* @retval None
* @note   ��Ҫע����ǣ�
*			1��	 ��Ϊƽ̨����������������Ҫ��λ�û���������޷���Speed_limit_UD����ֹλ�û�����������ٶ�̫��
*			2��  �����ڻ����޷�������޷�����Ҫ�ڵ�����֮���ں궨���и��ģ���������ʽpid�У�û��д�����޷�
*
*/

void Motor_Inite_All_up_dowm(MotorXDef *motorxdef)
{
	//state MotorPur_Lift
	motorxdef->State.No=MotorPur_Lift;//�����; ��������
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
	motorxdef->State.No=MotorPur_Cheassis;//�����; ��������
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
* @brief  �������е��̵��ʹ��λ
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
* @brief  ���������������ʹ��λ
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
* @brief  PIDϵ����ʼ��
* @param  MotorXDef *motorxdef :����Ϊ����ṹ�壬MortorA ~ MortorZ
* @retval None
* @note   ��Ҫע����ǣ�
*			1��  kp��ki��kd�⼸���������Ϊ -255����ʹ��Ĭ�ϲ��������Ϊ������������������
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
* @brief  �������
* @param  
* @retval None
* @note   
*/
void Motor_Zero(MotorXDef* motorxdef)
{
	motorxdef->State.Loc = 0;
}

/**
* @brief  ���·��ƽ���ٶȲ���
* @param  motorxA ͬ��Ŀ����     motorxB ��ͬ�����
* @retval None
* @note   
*/
void MotorLocBalence_SpdEqualize(MotorXDef* motorxA,MotorXDef* motorxB)
{
	float loc_e=motorxA->State.Loc-motorxB->State.Loc;//��ȡλ�����
	float equalize_k=loc_e*1;//�ٶȲ���
	
	if(equalize_k<-20)equalize_k=-20;//�ٶȲ����޷�  20
	if(equalize_k>20)equalize_k=20;
	motorxB->Com.Spd+=equalize_k;
	//motorxA->State.Loc = 0;

}





/**
* @brief  ����ƽ̨������ƺ���
* @param  MotorXDef *motorxdef :����Ϊ����ṹ�壬MortorA ~ MortorZ 
* @param   unsigned int pid_control_type_t: use to judge "only speed_pid" or " loction_pid and speed_pid" 
*          0:only speed_pid
*		   1:loction_pid and speed_pid
* @retval None
* @note   ��Ҫע����ǣ�
*         ���ô˺���ǰ����Ҫ����������ȡ 
*			motorxdef->Com.Spd��  �ٶ�����							 
*			motorxdef->State.Spd���ٶȷ���								 
*			motorxdef->Com.Loc��  λ������									 
*			motorxdef->State.loc��λ�÷���										 
*			motorxdef->pid_control_type���ֿ�or���Կأ��ֿ�ֻ���ٶȻ������Կ���λ���ٶȻ����
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
	if(MotorProtect(motorxdef))return; //�����������
	
	
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
		float loction_pid_output = PID_Control_normal(&motorxdef->LOC_PID.moter_pid, expected_value_t, actual_value_t, Positional_PID);//����ƽ̨ λ�û���λ��ʽpid

		actual_value_t = motorxdef->State.Spd;
		motorxdef->Motor_output = PID_Control_normal(&motorxdef->SPD_PID.moter_pid, loction_pid_output, actual_value_t, Incremental_PID);
	}
}

//�����������
char MotorProtect(MotorXDef *motorxdef)
{
	//���ʧ��
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
	
	//��ת���� ���� �Զ�����
	if(GetMotorStall(motorxdef->State.State)||SystemState.Mode==SystemMode_Set_Zero)
	{
		motorxdef->SPD_PID.moter_pid.Ulimit=Motor_Stall_Output+100; //�趨��ת�޷�
	}
	else
	{
		switch(motorxdef->State.No)//���ݵ�����ͻָ������޷�
		{
			case MotorPur_Cheassis:
				motorxdef->SPD_PID.moter_pid.Ulimit=Motor_output_limit_CH;//�ָ����̵���޷�
				break;
			case MotorPur_Lift:
				motorxdef->SPD_PID.moter_pid.Ulimit=Motor_output_limit_UD;//�ָ���������޷�
				break;				
		}
	}
	return 0;
}

//����������������   data ������������׵�ַ  output ������  deadzone ����
void AddDeadZone(unsigned char*data,short output,short deadzone)
{
	if(output>0)output+=deadzone;
	if(output<0)output-=deadzone;
	//*(short*)data = ((short)output>>8) | ((short)output&0x00ff)<<8;
	*data=(char)(output>>8);
	*(data+1)=(char)(output&0x00ff);
}

//���Ϳ���ָ�� PIDTask �е���
void SentControlData()
{
	//  ң���� RC_Data.switch_right==2 ����ʧ��
		unsigned char data[8]={0};
		//==============================����
		AddDeadZone(&data[0],(short)Motor1.Motor_output,200);
		AddDeadZone(&data[2],(short)Motor2.Motor_output,200);
		AddDeadZone(&data[4],(short)Motor3.Motor_output,200);
		AddDeadZone(&data[6],(short)Motor4.Motor_output,200);

		if(!(SystemState.Enable&SystemState_Enable_Move)||RC_Data.switch_right==2)
			 memset(data,0,8);
		Can_Sent_msgToQue(1,0x1FF,data);//���̵�����Ӧ����5~8  ID��0x1ff �޸�Can�����Ǳ߲���
		
		//==============================����ƽ̨
		memset(data,0,8);
		AddDeadZone(&data[0],(short)MotorA.Motor_output,200);
		AddDeadZone(&data[2],(short)MotorB.Motor_output,200);

		if(!(SystemState.Enable&SystemState_Enable_Lift)||RC_Data.switch_right==2)
			 memset(data,0,8);
		Can_Sent_msgToQue(1,0x200,data);//һ��̧�����
		
		memset(data,0,8);
		AddDeadZone(&data[0],(short)MotorE.Motor_output,200);
		AddDeadZone(&data[2],(short)MotorF.Motor_output,200);
		AddDeadZone(&data[4],(short)MotorC.Motor_output,200);
		AddDeadZone(&data[6],(short)MotorD.Motor_output,200);
		if(!(SystemState.Enable&SystemState_Enable_Lift)||RC_Data.switch_right==2)
			 memset(data,0,8);
		Can_Sent_msgToQue(2,0x200,data);//����̧���Լ�������ת
		
		//��չ��1 ������ƹ�
		memset(data,0,8);
		AddDeadZone(&data[0],SystemState.Robot.Catch,0);
		AddDeadZone(&data[2],SystemState.Robot.Pump,0);
		AddDeadZone(&data[4],SystemState.Robot.Magazine,0);
		AddDeadZone(&data[6],SystemState.Robot.Magazine_Shell,0);
		Can_Sent_msgToQue(2,CAN_EXTEND_RECEIVE_ID,data);//��չ�� �г�����ձ�
		
		//��չ��2 ��Ԯ�� ��Ѫ��
		memset(data,0,8);
		AddDeadZone(&data[0],SystemState.Robot.ReliefFrame,0);
		AddDeadZone(&data[6],SystemState.Robot.AddHPCard,0);
		
		Can_Sent_msgToQue(1,CAN_EXTEND2_RECEIVE_ID,data);//��չ�� �г�����ձ�

}
//=================================================================
//
//								    ������
//
//
//=================================================================

//���PID��������
void vMotorPIDCtr_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	vTaskDelay(1000 / portTICK_RATE_MS);//�ӳ�����1s
	MotorPID_TypeDef Lift_PIDCtr_Type=MotorSPD_Ctr;

	while(1)//�������
	{
		GetTaskPeriod(PID_Task_No);//��ȡ���м��
		
		//=======================���������===============
		//���̵��PID����(���ٶȻ�)
		Motor_control_UD(&Motor1,MotorSPD_Ctr);
		Motor_control_UD(&Motor2,MotorSPD_Ctr);
		Motor_control_UD(&Motor3,MotorSPD_Ctr);
		Motor_control_UD(&Motor4,MotorSPD_Ctr);
		
		switch(SystemState.Mode)
		{
			case SystemMode_Set_Zero://�����λʱ�����ٶȿ���
				Lift_PIDCtr_Type=MotorSPD_Ctr;
			break;
			default:
				Lift_PIDCtr_Type=MotorLOC_Ctr;//λ�ÿ���
			break;
		}
		Motor_control_UD(&MotorA,Lift_PIDCtr_Type);//һ��̧��
		Motor_control_UD(&MotorB,Lift_PIDCtr_Type);//һ��̧��
		Motor_control_UD(&MotorC,Lift_PIDCtr_Type);//����̧��
		Motor_control_UD(&MotorD,Lift_PIDCtr_Type);//����̧��
		Motor_control_UD(&MotorE,Lift_PIDCtr_Type);//ץȡ����ǰ��
		Motor_control_UD(&MotorF,Lift_PIDCtr_Type);//������ת
		
		//=======================���Ϳ���ָ��=============
		SentControlData();//ͨ��can���Ϳ���ָ�� 
		vTaskDelayUntil(&xLastWakeTime,5/ portTICK_RATE_MS);
	}
}


//=================================================================
//
//								�ⲿ���ú���
//
//
//=================================================================

//��ȡ���״̬  ��BSP_CAN.c����
void GetMotorState(unsigned char* data, MotorXDef* motorxdef)
{
	short now_MLoc = ((short)data[0] << 8) | data[1];
	short pre_MLoc = motorxdef->State.Pre_MLoc;
	float tem_e;
	float Loc_e = now_MLoc - pre_MLoc;//·�̲�

	if (Loc_e < (-0.66*motorxdef->Struct.Step))Loc_e += motorxdef->Struct.Step;
	if (Loc_e > (0.66*motorxdef->Struct.Step))Loc_e -= motorxdef->Struct.Step;

	motorxdef->State.Loc += Loc_e / motorxdef->Struct.Step*motorxdef->Struct.Lead;//��ȡλ��(mm)
	motorxdef->State.Spd = (short)((data[2] << 8) | data[3])*1.0 * motorxdef->Struct.Lead / 60;//��ȡ�ٶ�(mm/s)
	motorxdef->State.Current = (short)((data[4] << 8) | data[5]) / 819.2;//��ȡ����(mm/s)
	motorxdef->State.temprature = data[6];//��ȡ�¶�
	motorxdef->State.Pre_MLoc = now_MLoc;//������ת��λ�ü�¼

	tem_e = motorxdef->State.Loc - motorxdef->Com.Loc;//λ�ò�
	if (tem_e*tem_e < 25)
		SetMotorLocNear(motorxdef->State.State);
	else
		ClearMotorLocNear(motorxdef->State.State);

	tem_e = motorxdef->State.Spd - motorxdef->Com.Spd;//�ٶȲ�
	if (tem_e*tem_e < 100)
		SetMotorSpdNear(motorxdef->State.State);
	else
		ClearMotorSpdNear(motorxdef->State.State);
}


/**
* @brief  ��ʼ�����е��
* @param  MotorXDef *motorxdef :����Ϊ����ṹ�壬MortorA ~ MortorZ
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
	MotorF.Struct.Lead=13.333;//���E�ĵ���Ϊ  13.333��/r
	
	Motor_Inite_All_Chassis(&Motor1);
	Motor_Inite_All_Chassis(&Motor2);
	Motor_Inite_All_Chassis(&Motor3);
	Motor_Inite_All_Chassis(&Motor4);
	
	if(!xTaskCreate( vMotorPIDCtr_Task, "MotorPIDCtr_Task", 200, NULL, 5, NULL ))return 0;
	return 1;

}


/******************************************************************************
* @attention
*			�ٶȷֽ⣺
*					  1�����Ӳ���    1    2
*						             	 3    4
*					  2����������ϵ �ѿ�������ϵ��������x���ң�������y����
*					  3���ٶȷֽ⹫ʽ
									w1			+1	+1	+(L1+L2)
									w2			-1	+1	-(L1+L2)  VX
									w3  = 1/R *	-1	+1	+(L1+L2)  VY
									w4			+1	+1	-(L1+L2)  W
******************************************************************************/
/**
* @brief ȫ��λ���ٶȷֽ�
* @param
* @retval None
* @note  ����ҡ�˵�xyw��������ĸ��������ֽ⵽�ĸ������
*        ��Ҫע����ǣ�ң������������ʵ������ʵ�ʵĳ���x,y,w,��λ��mm/s��mm/s,rad/s
*/

//3���ٶȷֽ�ϵ��
#define Y_MecWheel 1.0f
#define X_MecWheel 1.0f
#define RobW_MecW  0.4875f // RobW_MecW =�����峤��+�����ȣ�/ 2 /1000

void SpeedDistribute()
{
    Motor1.Com.Spd = +SystemState.Robot.Move_X_SPD * X_MecWheel + SystemState.Robot.Move_Y_SPD *Y_MecWheel + SystemState.Robot.Move_Z_SPD * RobW_MecW;
    Motor2.Com.Spd = -SystemState.Robot.Move_X_SPD * X_MecWheel + SystemState.Robot.Move_Y_SPD *Y_MecWheel - SystemState.Robot.Move_Z_SPD * RobW_MecW;
		Motor3.Com.Spd = -SystemState.Robot.Move_X_SPD * X_MecWheel + SystemState.Robot.Move_Y_SPD *Y_MecWheel + SystemState.Robot.Move_Z_SPD * RobW_MecW;
    Motor4.Com.Spd = +SystemState.Robot.Move_X_SPD * X_MecWheel + SystemState.Robot.Move_Y_SPD *Y_MecWheel - SystemState.Robot.Move_Z_SPD * RobW_MecW;

    Motor2.Com.Spd = 0 - Motor2.Com.Spd;
    Motor4.Com.Spd = 0 - Motor4.Com.Spd;
}

