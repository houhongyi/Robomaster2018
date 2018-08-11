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
#define SystemTask_UpIsland  1  //�ϵ����� ��1λ
#define SystemTask_DownIsland  2 //�µ����� ��2λ
#define SystemTask_AutoMove  3 //�Զ��ƶ� ��3λ

#define SetSystemState(x) SystemState.State=SystemState.State|x //����ϵͳ״̬
#define ClearSystemState(x) SystemState.State=SystemState.State&(~x) //���ϵͳ״̬
#define GetSystemState(x) (SystemState.State&x)//��ȡϵͳ״̬



//==============================================================�����ת����
#define Motor_Stall_Time 200
#define Motor_Stall_Spd 5
#define OutLine_Time 50 //���߼��ʱ��

typedef enum
{
	None=0,
	Forward=1,//ǰ��
	Backward=-1,//����
}Direct_enum;//���ö���

typedef enum
{
	Beep_Silence, //��Ĭ
	Beep_Beeping, //����
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
	short Catch;//����
	short Pump;//��ձ�
	short Magazine;//����
	short ReliefFrame;//��Ԯ��
	short ReliefFrame2;//��Ԯ�� ����
	short ReliefFinger;//��Ԯ��ָ
	short Camara_LED;//�ƹ�
	short AddHPCard;//��Ѫ��
	short EleBreak;//����ƶ���
	short Supplyer;//
	short GetBulletFrame;//��ȡ�ӵ�������
	float Move_X_SPD;//�����ٶ� ��Ϊ��
	float Move_Y_SPD;//ǰ���ٶ� ǰΪ��
	float Move_Z_SPD;//��ת�ٶ� ˳ʱ��Ϊ��
	
	//short Lift_Catch;//����Ϊ0 ��ȡΪ1
	float Lift_Z_Loc;//�����ܸ߶�
	float Lift_A_Loc;//������ת�Ƕ�
	float Lift_Y_Loc;//����ǰ������
	float Lift_Y_SPD;//����ǰ���ٶ�
	float Guide_Frame;//�����ܽǶ�
	YunTaiDex YunTai;
}RobotDef;

typedef struct{
	short Mode;//����ģʽ
	short Enable;//״̬
	short State;//״̬
	short Task;//����
	short GetBulletFinish;//�Ƿ����ȡ�� ���ڿ��Ƴ����Զ��ƶ�
	BEEPMode Beep;//������
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//ʱ����������
	long OutLine_Flag;//���߱�־
	
	UDIslanMode UDMode;//���µ�ģʽ
	DistSensorDex DistSensor;//���봫��������
	SensorDex Sensor;//����������
	RobotDef Robot;//�����˿���
	TaskHandleDex TaskHandle;//������
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
	RemoteCtr_No,//ң��
	ExtendCtr_No,//��չ��
	ExtendCtr2_No,//��չ��2
	DisMesur_No,//����
	YuntaiCtr_No,//��̨���ư�
	
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

void YuntaiMsg_GetState(unsigned char* s);//��ȡ��̨��ǰ�Ƕ�

#endif
