#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

//user include
#include "bsp_pid.h"


#define Motor_En_Disable 0 //���ʧ��
#define Motor_En_Enable 1 //���ʹ��
#define Motor_En_Suspend 2 //�������
#define Motor_En_Calibration  3 //�������

#define Motor_State_SpdNear 1 //����ٶȴﵽ�趨����
#define Motor_State_LocNear 2 //���λ�ôﵽ�趨����
#define Motor_State_Stall   4 //�����ת
#define Motor_State_OutLine 8 //�������

#define PID_Type_NULL -1 //��PID
#define PID_Type_Positional 0 //λ��ʽPID
#define PID_Type_Incremental 1 //����ʽPID

//======================================================================
#define SetMotorSpdNear(x) x=x|Motor_State_SpdNear//�趨����ٶȽӽ�
#define ClearMotorSpdNear(x) x=x&(~Motor_State_SpdNear)//�������ٶȽӽ�
#define GetMotorSpdNear(x) (x&Motor_State_SpdNear)//��ȡ����ٶȽӽ�״̬

#define SetMotorLocNear(x) x=x|Motor_State_LocNear//�趨���λ�ýӽ�
#define ClearMotorLocNear(x) x=x&(~Motor_State_LocNear)//������λ�ýӽ�
#define GetMotorLocNear(x) (x&Motor_State_LocNear)//��ȡ���λ�ýӽ�״̬

#define SetMotorStall(x) x=x|Motor_State_Stall//�趨�����ת
#define ClearMotorStall(x) x=x&(~Motor_State_Stall)//��������ת
#define GetMotorStall(x) (x&Motor_State_Stall)//��ȡ�����ת״̬

#define SetMotorOutLine(x) x=x|Motor_State_OutLine//�趨�������
#define ClearMotorOutLine(x) x=x&(~Motor_State_OutLine)//����������״̬
#define GetMotorOutLine(x) (x&Motor_State_OutLine)//��ȡ�������״̬
//======================================================================

/* DEFINE ------------------------------------------------------------------*/
#define PID_kp_ki_kd -255

//up_down ����ƽ̨

#define KP_Speed_UD 20//˫���е��ٶȻ�������ʽpid����
#define KI_Speed_UD 0.3
#define KD_Speed_UD 4

#define KP_Loction_UD 10//˫����λ�û���λ��ʽpid����
#define KI_Loction_UD 0.01
#define KD_Loction_UD 0

#define KP_single_Speed_UD 20 //���ٶȻ�������ʽpid����
#define KI_single_Speed_UD 0.3
#define KD_single_Speed_UD 4

#define Speed_limit_UD 400  // mm/s����������������λ�û�������޷� 
#define Motor_output_limit_UD 5000

#define Loction_pid_Ilimit 100

#define MOTOR_STRUCT_Lead 10.912//���̣�������ˢתһȦ��ִ�л������˶���
#define MOTOR_STRUCT_Step 8191 //��ˢƨ��תһȦ�����ص�����е�Ƕ�

//Chassis ������
#define KP_Speed_CH 30
#define KI_Speed_CH 1.2
#define KD_Speed_CH 15

#define KP_Loction_CH 1
#define KI_Loction_CH 1
#define KD_Loction_CH 1

#define Motor_output_limit_CH 10000

typedef enum
{
	MotorSPD_Ctr=0,
	MotorLOC_Ctr
}MotorPID_TypeDef;

typedef struct 
{
	int No;//������ 
	int State;//���״̬
	float Spd;//�����ٶ� ��mm/s��
	float Loc;//��ǰλ�� ��mm��
	int Pre_MLoc;//�ϴ�ת��λ��
	
	char temprature;//�¶�
	float Current;// ����
}MotorX_StateDef;

typedef struct 
{
	int En;//���ʹ�ܱ�־  
	float Spd;//�����ٶ� ��mm/s��
	float Acc;//���ٶ� (mm/s/s)
	float Loc;//���о��� ��mm��
}MotorX_ComDef;

typedef struct 
{
	PID moter_pid;
	PID_TYPE motor_pid_type;
	float memo_I;//��Ҫ��¼���ۼ���
}MotorX_PIDDef;

typedef struct
{
	float Lead;//����
	short Step;//һȦ������е�Ƕ�
}MotorX_StructDef;

typedef struct
{
	MotorX_StateDef State;
	MotorX_ComDef Com;
	MotorX_PIDDef SPD_PID;
	MotorX_PIDDef LOC_PID;
	MotorX_StructDef Struct;

	//Ϊ��ʹ��ʱ�򷽱㣬һЩ������������
	short Motor_output;//can send date
	MotorPID_TypeDef pid_control_type;
}MotorXDef;




typedef enum
{
	MotorPur_Cheassis,//���̵��
	MotorPur_Lift,//�������
}MotorPurpose_TypeDef;//�����;

extern MotorXDef Motor1,Motor2;


void Motor_Inite_All_up_dowm(MotorXDef *motorxdef);
void Motor_Inite_All_Chassis(MotorXDef *motorxdef);
void Motor_PID_change(MotorXDef *motorxdef, float kp_speed, float ki_speed, float kd_speed, float kp_loction, float ki_loction, float kd_loction);
void Motor_control_UD(MotorXDef *motorxdef, MotorPID_TypeDef pid_control_type_t);
void Motor_Zero(MotorXDef* Mot);
void MotorLocBalence_SpdEqualize(MotorXDef* motorxA,MotorXDef* motorxB);
void GetMotorState(unsigned char* data, MotorXDef* Mot);
int Motor_Inite_ALL(void);
void MoveMotor_EN_ALLSET(char en);
void LiftMotor_EN_ALLSET(char en);
char MotorProtect(MotorXDef *motorxdef);//�����������
#endif
