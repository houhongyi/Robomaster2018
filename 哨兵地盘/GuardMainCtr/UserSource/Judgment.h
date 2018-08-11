#ifndef __JUDGMENT_H__
#define __JUDGMENT_H__

#include <stm32f4xx.h>

/** 
  * @brief  judgement data command id
  */
typedef enum
{
  GAME_INFO_ID       = 0x0001,  //10Hz
  REAL_BLOOD_DATA_ID = 0x0002,
  REAL_SHOOT_DATA_ID = 0x0003,
	REAL_CHESS_POWER_ID= 0x0004,
  REAL_FIELD_DATA_ID = 0x0005,  //10hZ
  GAME_RESULT_ID     = 0x0006,
  GAIN_BUFF_ID       = 0x0007,
  Robo_Postion_ID			=0X0008,
	
  STU_CUSTOM_DATA_ID = 0x0100,//上传ID

} judge_data_id_e;

typedef struct
{
	unsigned short stateRemianTime;
	unsigned char gameProgress;
	unsigned char robotLevel;
	unsigned short remainHP;
}extGameRobotState_t;//比赛状态

typedef struct
{
	unsigned char armorType;
	unsigned char hurType ;
}extRobotHurt_t;//装甲状态

typedef struct
{
	unsigned char bulletType;//子弹类型
	unsigned char bulletFreq;//子弹频率
	float         bulletSpeed;//子弹速度
}extShootData_t;

typedef struct
{
	float chassisVolt;//地盘电压
	float chassisCurrent;//地盘电流
	float chassisPower;//地盘功率
	float chassisPowerBuffer;//地盘功率缓冲
	unsigned short shooterHeat0;//17mm枪口热量
}extPoerHeatData_t;

typedef struct
{
	unsigned char cardType;//卡类型
	unsigned char cardIdx;
}extRfidDetect_t;

typedef struct
{
	unsigned char buffType;
	unsigned char buffAddition;
}extGetBuff_t;

typedef struct
{
	float data1;
	float data2;
	float data3;
	unsigned char mask;
}extShowData_t;


typedef struct
{
	extGameRobotState_t 	GameRobotState;
	extRobotHurt_t 				RobotHurt;
	extShootData_t 				ShootDat;
	extPoerHeatData_t 		PoerHeatData;
	extRfidDetect_t 			RfidDetect;
	extGetBuff_t 					GetBuff;
	extShowData_t 				ShowData;

}JUDGEMENT;
extern JUDGEMENT JudgeMent;

void judgement_data_handler(uint8_t *p_frame);

void judgement_data_Division(uint8_t *p_frame);//裁判系统数据分割
#endif

