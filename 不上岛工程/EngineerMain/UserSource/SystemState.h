#ifndef __SysState_H__
#define __SysState_H__

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "bsp_motor.h"
#include "MyCom.h"
//===============================================================ϵͳģʽ����
#define SystemMode_Remote_MoveControl  0 //ң�ز�������ģʽ
#define SystemMode_Remote_LiftControl  1 //ң�ز�������ģʽ
#define SystemMode_KeyBoard_Control  2 //���̲���ģʽ
#define SystemMode_Auto_Control  3 //�Զ�����ģʽ
#define SystemMode_Set_Zero      4 //�Զ�����
#define SystemMode_Disable       5 //ʧ��ģʽ

#define SystemState_Enable  1 //ʹ��λ 
#define SystemState_Enable_Move  2 //����ʹ��λ
#define SystemState_Enable_Lift  4 //̧��ʹ��λ

#define SystemState_Normal  0 //״̬����
#define SystemState_MotorStall  1 //�����ת
#define SystemState_MotorOutline  2 //�������

#define SystemTask_GetBullet  0 //��ȡ�ӵ����� ��0λ

#define SetSystemState(x) SystemState.State=SystemState.State|x //����ϵͳ״̬
#define ClearSystemState(x) SystemState.State=SystemState.State&(~x) //���ϵͳ״̬
#define GetSystemState(x) (SystemState.State&x)//��ȡϵͳ״̬



//==============================================================�����ת����
#define Motor_Stall_Time 200
#define Motor_Stall_Spd 5
#define Motor_Stall_Output 4000  //�����ת�����ֵ
#define OutLine_Time 50 //���߼��ʱ��

typedef enum
{
	Beep_Silence, //��Ĭ
	Beep_Beeping, //����
}BEEPMode;

typedef struct{
	float Camera_X;
	float Camera_Y;
	float Camera_Z;
}SensorDex;

typedef struct{
	short Catch;//����
	short Pump;//��ձ�
	short Magazine;//����
	short Magazine_Shell;//���ָ�
	short ReliefFrame;//��Ԯ��
	short Electromagnet;//�����
	short Camara_LED;//�ƹ�
	short AddHPCard;//��Ѫ��
	float Move_X_SPD;//�����ٶ� ��Ϊ��
	float Move_Y_SPD;//ǰ���ٶ� ǰΪ��
	float Move_Z_SPD;//��ת�ٶ� ˳ʱ��Ϊ��
	
	short Lift_Catch;//����Ϊ0 ��ȡΪ1
	float Lift_Z_Loc;//���������߶�
	float Lift_X_Loc;//��������ǰ��λ��
	float Lift_A_Loc;//����Ƕ�
	
}RobotDef;

typedef struct{
	short Mode;//����ģʽ
	short Enable;//״̬
	short State;//״̬
	short Task;//����
	BEEPMode Beep;//������
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//ʱ����������
	long OutLine_Flag;//���߱�־
	
	SensorDex Sensor;//����������
	RobotDef Robot;//�����˿���
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
	RemoteCtr_No,//ң��
	ExtendCtr_No,//��չ��
	ExtendCtr2_No,//��չ��2
	
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
extern float g_Time_TaskPeriod[TaskTotal_No];//���������м��
extern float g_TimePre_Taskstart[TaskTotal_No];//��������������ʱ��

extern SystemStateDef SystemState;
extern MotorXDef* Motors[MotorTotal_No];

int SystemState_Inite(void);//SystemState��ʼ��
inline void RefreshSysTime(void);//ˢ��ϵͳʱ�䣨mm��
float GetSystemTimer(void);//��ȡϵͳ��ǰ׼ȷʱ��
void MesureTimePeriod(void);//�������м��
void GetTaskPeriod(TaskX_NoDEF No);//�����������м��




void RefreshMotorOutLineTime(MotorX_NoDEF Mot_No);//ˢ�µ��ͨ��ʱ������
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//ˢ������ͨ��ʱ��ʱ������
void MotorStall_Check(void);//�����ת�����
void OutLine_Check(void);//���߼����
#endif
