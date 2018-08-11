#include "include_bsp.h"

MotorXDef Motor1,Motor2,Motor3,Motor4,MotorA,MotorB,MotorC,MotorD,MotorE,MotorF;;

void MotorInite_default(MotorXDef* Mot);

void Motor_Inite()
{
  MotorInite_default(&Motor1);
	MotorInite_default(&Motor2);
	MotorInite_default(&Motor3);
	MotorInite_default(&Motor4);
	MotorInite_default(&MotorA);
	MotorInite_default(&MotorB);
	MotorInite_default(&MotorC);
	MotorInite_default(&MotorD);
	MotorInite_default(&MotorE);
	MotorInite_default(&MotorF);
}

//Ĭ�ϵ�����ò���
void MotorInite_default(MotorXDef* Mot)
{
	Mot->State.No=-1;//������
	Mot->State.State=0;//���״̬��־
	Mot->State.Spd=0;//����ٶ� mm/s
	Mot->State.Loc=0;//���λ�� mm
	Mot->State.Pre_MLoc=0;//�ϴε��ת��λ�� num
	
	Mot->output=0;//���
	
	Mot->Com.En=Motor_En_Enable;//���ʹ��
	Mot->Com.Spd=0;//�趨����ٶ� mm/s
	Mot->Com.Acc=-1;//�趨������ٶ�  mm/s^2
	Mot->Com.Loc=0;//�趨���λ��
	
	Mot->SPD_PID.type=PID_Type_Incremental;//�趨����ٶ�PID����
	Mot->SPD_PID.K_P=0;
	Mot->SPD_PID.K_D=0;
	Mot->SPD_PID.K_I=0;
	Mot->SPD_PID.pre_Output=0;//�ϴ�PID���
	Mot->SPD_PID.pre_erro=0;//�ϴ����
	Mot->SPD_PID.ppre_erro=0;//���ϴ����
	Mot->SPD_PID.memo_I=0;//��Ҫ��¼�Ļ�������
	
	Mot->LOC_PID.type=PID_Type_Incremental;//�趨���λ��PID����
	Mot->LOC_PID.K_P=0;
	Mot->LOC_PID.K_D=0;
	Mot->LOC_PID.K_I=0;
	Mot->LOC_PID.pre_Output=0;//�ϴ�PID���
	Mot->LOC_PID.pre_erro=0;//�ϴ����
	Mot->LOC_PID.ppre_erro=0;//���ϴ����
	Mot->LOC_PID.memo_I=0;//��Ҫ��¼�Ļ�������
	
	Mot->Struct.Lead=10;//������� �����ת������ֱ��*3.14 / ���ٱ�..
	Mot->Struct.Step=8191;//���תһȦ��õ�����
}

void Motor_Zero(MotorXDef* Mot)//�������
{
		Mot->State.Loc=0;
}

//ͨ��Can���ݻ�ȡ�����ǰ״̬
/*
  Mot  ----  ���ָ��
	data ----- Can���ݵ�ַ
*/

void GetMotorState(unsigned char* data,MotorXDef* Mot)
{
	short now_MLoc=((short)data[0]<<8)|data[1];
	short pre_MLoc=Mot->State.Pre_MLoc;
	float tem_e;
	float Loc_e=now_MLoc-pre_MLoc;//·�̲�
	
	if(Loc_e < (-0.66*Mot->Struct.Step) )Loc_e+=Mot->Struct.Step;
	if(Loc_e > (0.66*Mot->Struct.Step) )Loc_e-=Mot->Struct.Step;
	
	Mot->State.Loc+=Loc_e/Mot->Struct.Step*Mot->Struct.Lead;//��ȡλ��(mm)
	Mot->State.Spd=(short)((data[2]<<8)|data[3])*1.0 * Mot->Struct.Lead / 60;//��ȡ�ٶ�(mm/s)
	Mot->State.Current=(short)((data[4]<<8)|data[5]) /819.2 ;//��ȡ����(mm/s)
	Mot->State.temprature=data[6];//��ȡ�¶�
	Mot->State.Pre_MLoc=now_MLoc;//������ת��λ�ü�¼
	
	tem_e=Mot->State.Loc - Mot->Com.Loc;//λ�ò�
	if (tem_e*tem_e<25)
		SetMotorLocNear(Mot->State.State);
	else
		ClearMotorLocNear(Mot->State.State);
	
	tem_e=Mot->State.Spd - Mot->Com.Spd;//�ٶȲ�
	if (tem_e*tem_e < 100)
		SetMotorSpdNear(Mot->State.State);
	else
		ClearMotorSpdNear(Mot->State.State);
}

