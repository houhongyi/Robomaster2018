#ifndef __SysState_H
#define __SysState_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "MyCom.h"

//==============================================================�����ת����
#define Motor_Stall_Time 200
#define Motor_Stall_Spd 5
#define Motor_Stall_Output 2500  //�����ת�����ֵ
#define OutLine_Time 50 //���߼��ʱ��

//===============================================================ϵͳģʽ����
#define System_Disable        0 //ʧ��ģʽ
//#define SystemMode_Set_Zero       1 //�Զ�����
//#define SystemMode_RemoteControl  2 //ң�ز���
//#define SystemMode_WIFI_Control   3 //wifi����ģʽ
//#define SystemMode_Auto_Control   4 //�Զ�����ģʽ*/

#define SystemState_Enable  1 //ʹ��λ 


#define SystemState_Normal  0 //״̬����
#define SystemState_MotorStall  1 //�����ת
#define SystemState_MotorOutline  2 //�������

#define SystemTask_GetBullet  0 //��ȡ�ӵ����� ��0λ

#define SetSystemState(x) SystemState.State=SystemState.State|x //����ϵͳ״̬
#define ClearSystemState(x) SystemState.State=SystemState.State&(~x) //���ϵͳ״̬
#define GetSystemState(x) (SystemState.State&x)//��ȡϵͳ״̬


	#ifndef __MyCom_H
		#define my_abs(x) ((x)>0?(x):-(x)) //ABS�궨��
		#define MyFlagSet(x,y) x=x|(0x00000001<<y) //���ñ�־λ  y�ڼ�λ
		#define MyFlagClear(x,y) x=x&~(0x00000001<<y)
		#define MyFlagGet(x,y) (x&(0x00000001<<y))
	#endif

typedef enum
{
	Beep_Silence, //��Ĭ
	Beep_Beeping, //����
}BEEPMode;

typedef enum
{
	Mode_Cali,//У׼ģʽ
	Mode_Cruise,//Ѳ��
	Mode_Attack,//����ģʽ
	Mode_Foolish,//���㹥��ģʽ
	Mode_Defense,//����ģʽ
	Mode_Crazy,//����ģʽ
	Mode_YuntaiCTR,//��̨����ģʽģʽ
}GarderMode;//�ڱ�����ģʽ

typedef enum
{
	State_AutoCTR,//�Զ�����
	State_RemoteCTR,//ң��������
	State_WifiCTR,//WIFI����ģʽ
}GarderState;//�ڱ�����״̬

typedef enum
{
	Enemy_Finding=0,//�����ҵо�
	Enemy_Finded,//�ҵ��о�
}GarderFindEnemyDef;//�ڱ��о�ɨ��״̬

typedef enum
{
	Fire_Deny=0,//��ֹ����
	Fire_Permite=1,//������
}GarderFireDef;//�ڱ�������Ȩ

typedef enum
{
	YuntaiCTR_Deny=0,//��̨�ܾ�����
	YuntaiCTR_Applay=1,//��̨�������
}YuntaiApplayDef;//��̨����


typedef struct{
	GarderMode Mode;//����ģʽ
	GarderState RunState;//����״̬
	GarderFindEnemyDef FindEnemy;//�о�ɨ��״̬
	GarderFireDef GarderFire;//�о�������Ȩ
	YuntaiApplayDef YuntaiApplay;//��̨�������
	short Limite_Right;//����λ
	short Limite_Left;//����λ
	short Yuntai_CTR_APP_SPD;//��̨���������ٶ�
	short M2006_A_Current;
	short M2006_B_Current;
}RoboteDef;


typedef struct{
	
	char State;//״̬
	short Enable;
	short Task;//����
	BEEPMode Beep;//������
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//ʱ����������
	long OutLine_Flag;//���߱�־
	float Wifi_ctrTime;//WIFI�ϴη�������ʱ��
	
	RoboteDef Robote;//������
}SystemStateDef;

typedef enum
{
	Motor1_No=0,
	Motor2_No,

	MotorTotal_No
}MotorX_NoDEF;

typedef enum
{
	RemoteCtr_No=0,//ң��
	YUNTAICtrBoard_No,//��̨��
	WIFIBoard_No,//Wifi��
	Judgment_No,//����ϵͳ
	Judgment_hurt_No,//����ϵͳ
	Motor2006CTR_No,//2006�������͸��
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

float GetSystemTime(void);//��ȡ��ǰϵͳʱ��
void RefreshSysTime(void);//����ʱ�� �ж�ˢ���е���
int SystemState_Inite(void);//ϵͳ��ʼ��
void RefreshMotorOutLineTime(MotorX_NoDEF Mot_No);//ˢ�µ��ͨ������
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//ˢ������ͨ������
void MesureTimePeriod(void);
#endif

