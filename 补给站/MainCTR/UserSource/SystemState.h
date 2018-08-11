#ifndef __SysState_H__
#define __SysState_H__

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "MyCom.h"
//===============================================================ϵͳģʽ����

#define SetSystemState(x) SystemState.State=SystemState.State|x //����ϵͳ״̬
#define ClearSystemState(x) SystemState.State=SystemState.State&(~x) //���ϵͳ״̬
#define GetSystemState(x) (SystemState.State&x)//��ȡϵͳ״̬

//==============================================================�����ת����

#define OutLine_Time 100 //���߼��ʱ��

typedef enum
{
	Beep_Silence, //��Ĭ
	Beep_Beeping, //����
}BEEPMode;

typedef struct
{
	float Engneer_communi_t;
	float Hero_communi_t;
}NIR_Commui_tDef;

typedef struct
{
	char Cylinder_UL;
	char Cylinder_UR;
	char Cylinder_L;
	char Cylinder_R;
	char Cylinder_M;
}CylindeDef;


typedef struct{
	short Mode;//����ģʽ
	short Enable;//״̬
	short State;//״̬
	short Task;//����
	BEEPMode Beep;//������
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//ʱ����������
	long OutLine_Flag;//���߱�־
	CylindeDef Cylinde;//����״̬
	NIR_Commui_tDef NIR_Commui_t;//���һ�κ���ͨ��ʱ��
}SystemStateDef;


typedef enum
{
	MosCtr_Heart,
	Motor_Lift,
	NFC_Reader_L,
	NFC_Reader_R,
	NFC_Reader_LCard,
	NFC_Reader_RCard,
	NIR_Reader,
	
	DeviceTotal_No	
}DeviceX_NoDEF;


extern SystemStateDef SystemState;


int SystemState_Inite(void);//SystemState��ʼ��
inline void RefreshSysTime(void);//ˢ��ϵͳʱ�䣨mm��
float GetSystemTimer(void);//��ȡϵͳ��ǰ׼ȷʱ��
void MesureTimePeriod(void);//�������м��

void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//ˢ������ͨ��ʱ��ʱ������
void OutLine_Check(void);//���߼����
#endif
