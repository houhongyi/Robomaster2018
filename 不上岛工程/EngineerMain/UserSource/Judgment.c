#include "judgment.h"
#include "protocol.h"
#include "string.h"


//======= other Lib
#include "SystemState.h"

JUDGEMENT JudgeMent={0};

uint16_t data_length=0,cmd_id=0;


//裁判系统数据分割
void judgement_data_Division(uint8_t *p_frame)
{
	char i=0;
	uint8_t *s_frame=p_frame;
	uint16_t tem_data_length=0;
	for(i=0;i<10;i++)
	{
		if(*s_frame==0xA5)//帧头
		{
			if(verify_crc8_check_sum(s_frame, 5))
			{	
				memcpy(&tem_data_length,(s_frame + 1),sizeof(uint16_t));//数据长度
				if(tem_data_length==0)break;
				judgement_data_handler(s_frame);
				s_frame+=5+2+tem_data_length+2;
			}
		}
		else
		{
			s_frame++;
		}
		if(s_frame-p_frame>=170)break;
	}	
}

void judgement_data_handler(uint8_t *p_frame)
{

  memcpy(&data_length,(p_frame + 1),sizeof(uint16_t));//数据长度
	memcpy(&cmd_id,(p_frame + 5),sizeof(uint16_t));//信息ID
	
	if(!verify_crc16_check_sum(p_frame,5+2+data_length+2))//进行校验
		return;
#ifdef __SysState_H
	RefreshDeviceOutLineTime(Judgment_No);//刷新裁判系统通讯时间   Judgment_hurt_No
#endif	
  switch (cmd_id)
  {
    case GAME_INFO_ID:
			memcpy(&JudgeMent.GameRobotState.stateRemianTime,(p_frame + 7 + 0),sizeof(uint16_t));//剩余时间
			memcpy(&JudgeMent.GameRobotState.gameProgress,(p_frame + 7 + 2 ),sizeof(uint8_t));//比赛阶段
			memcpy(&JudgeMent.GameRobotState.robotLevel,(p_frame + 7 + 3 ),sizeof(uint8_t));//机器人等级
			memcpy(&JudgeMent.GameRobotState.remainHP,(p_frame + 7 + 4 ),sizeof(uint16_t));//机器人当前血量
	
    break;

    case REAL_BLOOD_DATA_ID://伤害数据

			JudgeMent.RobotHurt.armorType=*(p_frame + 7 + 0 ) & 0x07;//伤害方向
			JudgeMent.RobotHurt.hurType=*(p_frame + 7 + 0 ) >> 4;//伤害原因
#ifdef __SysState_H
			RefreshDeviceOutLineTime(Judgment_hurt_No);//刷新裁判系统 伤害帧头
			MyFlagClear(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_hurt_No));//清除裁判系统 伤害检测断线
#endif
    break;

    case REAL_SHOOT_DATA_ID:
			memcpy(&JudgeMent.ShootDat.bulletFreq,(p_frame + 7 + 1),sizeof(uint8_t));//弹频
			memcpy(&JudgeMent.ShootDat.bulletSpeed,(p_frame + 7 + 2),sizeof(float));//弹速
    break;
		
		case REAL_CHESS_POWER_ID:
			memcpy(&JudgeMent.PoerHeatData.chassisVolt,(p_frame + 7 + 0),sizeof(float));//地盘电压 
			memcpy(&JudgeMent.PoerHeatData.chassisCurrent,(p_frame + 7 + 4),sizeof(float));//地盘电流
			memcpy(&JudgeMent.PoerHeatData.chassisPower,(p_frame + 7 + 8),sizeof(float));//地盘功率
			memcpy(&JudgeMent.PoerHeatData.chassisPowerBuffer,(p_frame + 7 + 12),sizeof(float));//地盘功率缓冲
			memcpy(&JudgeMent.PoerHeatData.shooterHeat0,(p_frame + 7 + 16),sizeof(uint16_t));//地盘功率缓冲

    break;
		case REAL_FIELD_DATA_ID:
			memcpy(&JudgeMent.RfidDetect.cardType,(p_frame + 7 + 0 ),sizeof(uint8_t));//加成卡
			memcpy(&JudgeMent.RfidDetect.cardIdx,(p_frame + 7 + 1 ),sizeof(uint8_t));//卡索引号

			break;
		default:
			
			break;
  }  
}


