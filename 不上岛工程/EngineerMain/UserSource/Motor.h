#ifndef __Motor_H
#define __Motor_H

#define Motor_En_Disable 0 //���ʧ��
#define Motor_En_Enable 1 //���ʹ��

#define Motor_State_SpdNear 1 //����ٶȴﵽ�趨����
#define Motor_State_LocNear 2 //���λ�ôﵽ�趨����

#define PID_Type_NULL -1 //��PID
#define PID_Type_Positional 0 //λ��ʽPID
#define PID_Type_Incremental 1 //����ʽPID

#define SetMotorSpdNear(x) x=x|Motor_State_SpdNear//�趨����ٶȽӽ�
#define ClearMotorSpdNear(x) x=x&(~Motor_State_SpdNear)//�������ٶȽӽ�
#define GetMotorSpdNear(x) x&Motor_State_SpdNear//��ȡ����ٶȽӽ�״̬
#define SetMotorLocNear(x) x=x|Motor_State_LocNear//�趨���λ�ýӽ�
#define ClearMotorLocNear(x) x=x&(~Motor_State_LocNear)//������λ�ýӽ�
#define GetMotorLocNear(x) x&Motor_State_LocNear//��ȡ���λ�ýӽ�״̬

typedef struct 
{
	short No;//������ 
	short State;//���״̬
	float Spd;//�����ٶ� ��mm/s��
	float Loc;//��ǰλ�� ��mm��
	char temprature;//�¶�
	float Current;//����
	short Pre_MLoc;//�ϴ�ת��λ��

}MotorX_StateDef;

typedef struct 
{
	short En;//���ʹ�ܱ�־  
	float Spd;//�����ٶ� ��mm/s��
	float Acc;//���ٶ� (mm/s/s)
	float Loc;//���о��� ��mm��
}MotorX_ComDef;

typedef struct 
{
	short type;//PID��ʽ
	float K_P;//�����ٶ� ��mm/s��
	float K_I;//���ٶ� (mm/s/s)
	float K_D;//���о��� ��mm��
	float pre_Output;//�ϴ����
	float pre_erro;//�ϴ����
	float ppre_erro;//���ϴ����
	float memo_I;//��Ҫ��¼���ۼ���
}MotorX_PIDDef;

typedef struct
{
	short Lead;//����
	short Step;//һȦ��ת��
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



void Motor_Inite(void);//�����ʼ��
void GetMotorState(unsigned char* data,MotorXDef* Mot);//Can��Ϣ��ȡ�����Ϣ
void Motor_Zero(MotorXDef* Mot);//�������
#endif
