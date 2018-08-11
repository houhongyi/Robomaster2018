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

//默认电机配置参数
void MotorInite_default(MotorXDef* Mot)
{
	Mot->State.No=-1;//电机编号
	Mot->State.State=0;//电机状态标志
	Mot->State.Spd=0;//电机速度 mm/s
	Mot->State.Loc=0;//电机位置 mm
	Mot->State.Pre_MLoc=0;//上次电机转子位置 num
	
	Mot->output=0;//输出
	
	Mot->Com.En=Motor_En_Enable;//电机使能
	Mot->Com.Spd=0;//设定电机速度 mm/s
	Mot->Com.Acc=-1;//设定电机加速度  mm/s^2
	Mot->Com.Loc=0;//设定电机位置
	
	Mot->SPD_PID.type=PID_Type_Incremental;//设定电机速度PID类型
	Mot->SPD_PID.K_P=0;
	Mot->SPD_PID.K_D=0;
	Mot->SPD_PID.K_I=0;
	Mot->SPD_PID.pre_Output=0;//上次PID输出
	Mot->SPD_PID.pre_erro=0;//上次误差
	Mot->SPD_PID.ppre_erro=0;//上上次误差
	Mot->SPD_PID.memo_I=0;//需要记录的积分数据
	
	Mot->LOC_PID.type=PID_Type_Incremental;//设定电机位置PID类型
	Mot->LOC_PID.K_P=0;
	Mot->LOC_PID.K_D=0;
	Mot->LOC_PID.K_I=0;
	Mot->LOC_PID.pre_Output=0;//上次PID输出
	Mot->LOC_PID.pre_erro=0;//上次误差
	Mot->LOC_PID.ppre_erro=0;//上上次误差
	Mot->LOC_PID.memo_I=0;//需要记录的积分数据
	
	Mot->Struct.Lead=10;//电机导程 外接旋转机构的直径*3.14 / 减速比..
	Mot->Struct.Step=8191;//电机转一圈获得的脉冲
}

void Motor_Zero(MotorXDef* Mot)//电机设零
{
		Mot->State.Loc=0;
}

//通过Can数据获取电机当前状态
/*
  Mot  ----  电机指针
	data ----- Can数据地址
*/

void GetMotorState(unsigned char* data,MotorXDef* Mot)
{
	short now_MLoc=((short)data[0]<<8)|data[1];
	short pre_MLoc=Mot->State.Pre_MLoc;
	float tem_e;
	float Loc_e=now_MLoc-pre_MLoc;//路程差
	
	if(Loc_e < (-0.66*Mot->Struct.Step) )Loc_e+=Mot->Struct.Step;
	if(Loc_e > (0.66*Mot->Struct.Step) )Loc_e-=Mot->Struct.Step;
	
	Mot->State.Loc+=Loc_e/Mot->Struct.Step*Mot->Struct.Lead;//获取位置(mm)
	Mot->State.Spd=(short)((data[2]<<8)|data[3])*1.0 * Mot->Struct.Lead / 60;//获取速度(mm/s)
	Mot->State.Current=(short)((data[4]<<8)|data[5]) /819.2 ;//获取电流(mm/s)
	Mot->State.temprature=data[6];//获取温度
	Mot->State.Pre_MLoc=now_MLoc;//将本次转子位置记录
	
	tem_e=Mot->State.Loc - Mot->Com.Loc;//位置差
	if (tem_e*tem_e<25)
		SetMotorLocNear(Mot->State.State);
	else
		ClearMotorLocNear(Mot->State.State);
	
	tem_e=Mot->State.Spd - Mot->Com.Spd;//速度差
	if (tem_e*tem_e < 100)
		SetMotorSpdNear(Mot->State.State);
	else
		ClearMotorSpdNear(Mot->State.State);
}

