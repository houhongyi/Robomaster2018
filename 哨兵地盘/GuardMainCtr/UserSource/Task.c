#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "MyCom.h"
#include "SystemState.h"
#include "bsp_Uart.h"
#include "bsp_can.h"
#include "bsp_motor.h"
#include "Remote.h"
#include "Wifi.h"
#include "Judgment.h"

#include <string.h>

void CTR_CruiseMode(void);//巡航模式控制
void CTR_AttackMode(float attack_Loc);//攻击模式控制
void CTR_FoolishMode(float attack_Loc);//定点攻击模式控制
void CTR_DefenseMode(void);// 防御模式
void CTR_YuntaiCTRMode(void);//云台控制模式
void CTR_CrazyMode(void);//暴走模式


void CTR_WIFIMode(void);//WIFI速度控制模式

void GarderModeChange(void);//模式切换

//机器人控制函数
void vRoboteCtr_Task(void *pvParameters)//控制程序
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	GarderMode Mode_pre=Mode_Cruise;
	float Attack_Loc=1000;
	while(1)
	{
		
		if(!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+RemoteCtr_No))))//如果遥控上线
		{
			switch(RC_Data.switch_right)
			{
				case 2:
					SystemState.Enable=System_Disable;
				break;
				default :
					SystemState.Enable=SystemState_Enable;
					break;
				
			}
			Motor1.pid_control_type=MotorSPD_Ctr;//遥控器上线 地盘为速度控制
			Motor2.pid_control_type=MotorSPD_Ctr;
			Motor1.Com.Spd=RC_Data.ch4*8;
			Motor2.Com.Spd=RC_Data.ch4*8;
		}
		else//遥控器关闭
		{
			if( SystemState.Time<60000 && MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+YUNTAICtrBoard_No)))//等待云台上线
			{
					vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
					continue;
			}
			SystemState.Enable=SystemState_Enable;
			GarderModeChange();//模式切换
			//============================运行状态 更改电机控制方式
			switch(SystemState.Robote.RunState)
			{
				case State_AutoCTR:
					switch(SystemState.Robote.Mode)
					{
						case Mode_YuntaiCTR:
							Motor1.pid_control_type=MotorSPD_Ctr;// 云台控制方式
							Motor2.pid_control_type=MotorSPD_Ctr;
							break;
						default :
							Motor1.pid_control_type=MotorLOC_Ctr;
							Motor2.pid_control_type=MotorLOC_Ctr;
							break;
					}
					
					break;
				case State_WifiCTR:
					
					Motor1.pid_control_type=MotorSPD_Ctr;
					Motor2.pid_control_type=MotorSPD_Ctr;
				
					if(GetSystemTime()-SystemState.Wifi_ctrTime>1000)//没有WIFI控制 1s 后切换为自动运行模式
						SystemState.Robote.RunState=State_AutoCTR;
					
					break;
				default:
					
					break;					
			}
			//============================运行模式
			if(SystemState.Robote.RunState==State_WifiCTR)//如果是WIFI控制模式
			{
				CTR_WIFIMode();
			}
			else
			{
				switch(SystemState.Robote.Mode)// 模式切换在 WIFI控制中 或者 在与云台通信过程中更改
				{
					case Mode_Cali:
						break;
					case Mode_Cruise:
							CTR_CruiseMode();//巡航模式
						break;
					case Mode_Attack:
							if(Mode_pre!=Mode_Attack)
								Attack_Loc=Motor1.State.Loc;
								CTR_AttackMode(Attack_Loc);//攻击模式
						break;
					case Mode_Foolish:
						if(Mode_pre!=Mode_Attack)
								Attack_Loc=Motor1.State.Loc;
								CTR_FoolishMode(Attack_Loc);//定点攻击模式
						break;
					case Mode_Defense:
							CTR_DefenseMode();//  防御模式
						break;
					case Mode_Crazy:
						CTR_CrazyMode();//暴走模式
						break;
					case  Mode_YuntaiCTR://云台控制模式
							CTR_YuntaiCTRMode();
						break;
					default:
						break;
					
				}
				Mode_pre=SystemState.Robote.Mode;
			}
		}
		vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	}
}


//距离测量
void vDisGet_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	char LA=0;
	char LB=0;
	char LA_Pre=0;
	char LB_Pre=0;
	while(1)
	{
		LA=HAL_GPIO_ReadPin(LimitA_GPIO_Port,LimitA_Pin);
		LB=HAL_GPIO_ReadPin(LimitB_GPIO_Port,LimitB_Pin);
		
		if(LA & LA_Pre)
		{
			SystemState.Robote.Limite_Left=400;
			Motor1.State.Loc=4500; //轨道全长 5486
			Motor2.State.Loc=4500;
		}
			
		else
			SystemState.Robote.Limite_Left=1000;
		
		if(LB & LB_Pre)
		{
			SystemState.Robote.Limite_Right=400;
				Motor1.State.Loc=400; //轨道全长 5486
				Motor2.State.Loc=400;
		}
		else
			SystemState.Robote.Limite_Right=1000;
		
		LA_Pre=LA;
		LB_Pre=LB;
		vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	}
}

//LED
void vLED_Task(void *pvParameters)
{
	char i=0;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		if(SystemState.Robote.RunState==State_AutoCTR)
		{
			for(i=0;i<SystemState.Robote.Mode;i++)
			{
				
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
				vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
				vTaskDelayUntil(&xLastWakeTime,290/ portTICK_RATE_MS);
			}
		}
		else if(SystemState.Robote.RunState==State_WifiCTR)
		{
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
				vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
				vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
		}
		vTaskDelayUntil(&xLastWakeTime,1000/ portTICK_RATE_MS);
	}
	
	
}

//LED2
void vLED2_Task(void *pvParameters)
{
	char i=0;
	char deviceNo=0;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		for(deviceNo=Motor1_No;deviceNo<DeviceTotal_No+MotorTotal_No;deviceNo++)
		{
			if(deviceNo==Judgment_hurt_No+MotorTotal_No)//跳过检测裁判系统伤害
				continue;
			if(deviceNo==RemoteCtr_No+MotorTotal_No)//跳过检测遥控
				continue;
			if(deviceNo==WIFIBoard_No+MotorTotal_No)//跳过检测WIFI
				continue;

			if(MyFlagGet(SystemState.OutLine_Flag,deviceNo))
			{
				for(i=0;i<=deviceNo;i++)
				{
					vTaskDelayUntil(&xLastWakeTime,260/ portTICK_RATE_MS);
					HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
					vTaskDelayUntil(&xLastWakeTime,40/ portTICK_RATE_MS);
					HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
				}
			}
			vTaskDelayUntil(&xLastWakeTime,1000/ portTICK_RATE_MS);
		}
		
		vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
	}
}

//云台遥控信息发送
void vRemoteSenTOYuntai_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	unsigned char data[8]={0};
	RC_Data_Short tem_Data_Short={0};
	while(1)
	{
		if(!MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+RemoteCtr_No)))//获取遥控器状态
		{
			tem_Data_Short.ch1=RC_Data.ch1;
			tem_Data_Short.ch2=RC_Data.ch2;
			tem_Data_Short.switch_left=RC_Data.switch_left;
			tem_Data_Short.switch_right=RC_Data.switch_right;
			tem_Data_Short.State=0;
			tem_Data_Short.Bullet_Remain=1;//热量计算得出
			RemoteMsg_CanSent(data,&tem_Data_Short);
			Can_Sent_msgToQue(2,CAN_YUNTAI_Ctr_RXID,data);
		}
		
		//============================== 向云台发送 热量与位置
		
		if(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_hurt_No)))//获取裁判系统伤害检测数据是否更新
			JudgeMent.RobotHurt.armorType=8;//如果100ms内装甲没有受到攻击，装甲清零

		//  [0:1]热量值 高位在前   [2:5]位置信息 [高8位 次高] [7]开火授权
		memset(data,0,8);
		data[0]=(unsigned char)(JudgeMent.PoerHeatData.shooterHeat0>>8);//热量值
		data[1]=(unsigned char)(JudgeMent.PoerHeatData.shooterHeat0&0xff);
		
		data[2]=(unsigned char)(((int)Motor1.State.Loc>>24)&0xff);//M1位置
		data[3]=(unsigned char)(((int)Motor1.State.Loc>>16)&0xff);
		data[4]=(unsigned char)(((int)Motor1.State.Loc>>8)&0xff);
		data[5]=(unsigned char)( (int)Motor1.State.Loc&0xff);
		
		data[6]=SystemState.Robote.GarderFire<<4;//开火授权
		if(JudgeMent.RobotHurt.hurType==0)//只有受到攻击的装甲类型才会发送
			data[6]=data[6] | (JudgeMent.RobotHurt.armorType);
		else
			data[6]=data[6] | 0x08;//装甲没有受到攻击 发送装甲号8
		
		if(MyFlagGet(SystemState.OutLine_Flag,Motor1_No) || MyFlagGet(SystemState.OutLine_Flag,Motor2_No))
			MyFlagSet(data[7],1);//代表底盘失联
		
		Can_Sent_msgToQue(2,CAN_YUNTAI_Ctr2_RXID,data);
		
		if(!MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))//裁判系统是否上线
		{
			memset(data,0,8);
			data[0]=(unsigned char)(JudgeMent.GameRobotState.remainHP>>8);//发送剩余血量
			data[1]=(unsigned char)(JudgeMent.GameRobotState.remainHP&0xff);
			Can_Sent_msgToQue(2,CAN_YUNTAI_Ctr3_RXID,data);
		}
		
		vTaskDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);
	}
}



//串口处理
void UARTAnalys(UART_HandleTypeDef *huart,char* s,int len)
{
	if(huart==&huart1)
	{
		judgement_data_Division((uint8_t *)s);//裁判系统解算
	}
	
	if(huart==&huart4)//WIFI串口
	{
		switch(WifiDataRec((unsigned char*)s,&Wifi))
		{
			case Wifi_PingCom://Ping命令
					 if(Wifi.RxAdr==WifiState_Gaurder)//判断接收方
						Wifi_PingASK(Wifi.RxAdr,Wifi.TxAdr,Wifi.Data.ID);//回复PingASK
				break;
			 
			case Wifi_Success://正常模式
				
					if(Wifi.RxAdr!=WifiState_Gaurder)//判断接收方
						break;
					if(Wifi.Data.Com!=WifiCom_GarderRemoteCTR)//如果不是哨兵模拟控制
						break;
					SystemState.Robote.RunState=State_WifiCTR;//运行模式切换为WIFI控制模式
					//SystemState.Robote.Mode=(GarderMode)Wifi.Data.Mode;//运行模式为本次运行模式
					SystemState.Wifi_ctrTime=GetSystemTime();//获取本次WIFI控制时间
				break;
			default:
				break;
		}
	}
}

//========================================================
//
//									测试任务
//
//
//==========================================================
int n=0;
int g_mem_min=0;//最小剩余内存
int g_mem_n=0;//当前剩余内存
void vTest_Task(void *pvParameters)//测试程序
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	char printdata[30]={0};
	while(1)
	{
		g_mem_min=xPortGetMinimumEverFreeHeapSize();//获取最小内存剩余
		g_mem_n=xPortGetFreeHeapSize();//获取当前内存剩余
		
		if(SystemState.Robote.RunState==State_WifiCTR)//如果是WIFI控制模式
			{
				My_print("State_WifiCTR\r\n");
			}
			else
			{
				switch(SystemState.Robote.Mode)// 模式切换在 WIFI控制中 或者 在与云台通信过程中更改
				{
					case Mode_Cali:
						break;
					case Mode_Cruise:
							My_print("Cruise\r\n");
						break;
					case Mode_Attack:
							My_print("Mode_Attack\r\n");
						break;
					case Mode_Foolish:
							My_print("Mode_Foolish\r\n");
						break;
					case Mode_Defense:
							My_print("Mode_Defense\r\n");
						break;
					case Mode_Crazy:
							My_print("Mode_Crazy\r\n");
						break;
					case  Mode_YuntaiCTR://云台控制模式
							My_print("Mode_YuntaiCTR\r\n");
						break;
					default:
						break;
				}
			}
		vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
		memset(printdata,0,30);
		sprintf(printdata," Dist:%.3f <%d><%d>\r\n",Motor1.State.Loc,SystemState.Robote.FindEnemy,SystemState.Robote.GarderFire);
		UART_SendToQueue(&huart3,printdata);
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
}

//========================================================
//
//												调试任务			
//			
//
//==========================================================
char gs[300]={0};
void TaskSta_task(void *pvParameters)
{
    UBaseType_t   ArraySize;
    TaskStatus_t  *StatusArray;
    uint8_t       x;

    ArraySize = uxTaskGetNumberOfTasks();
    StatusArray = pvPortMalloc(ArraySize*sizeof(TaskStatus_t));
    while(1)
    {
        if(StatusArray != NULL){ 

            ArraySize = uxTaskGetSystemState( (TaskStatus_t *)  StatusArray,
                                              (UBaseType_t   ) ArraySize,
                                              NULL );

            //sprintf(gs,"TaskName\t\t\tPriority\t\tTaskNumber\t\tMinStk\t\t\n");
            for(x = 0;x<ArraySize;x++){

                sprintf(gs,"%s\t\t\t%d\t\t\t%d\t\t\t%d\t\t%d\r\n",
                        StatusArray[x].pcTaskName,
                        (int)StatusArray[x].uxCurrentPriority,
                        (int)StatusArray[x].xTaskNumber,
                        (int)StatusArray[x].usStackHighWaterMark,
                        (int)0);
            }
            //sprintf(gs,"\n\n");
        }
        vTaskDelay(2000);
    }
}


//========================================================
//
//									内部函数
//
//
//==========================================================


//=====================================
//从云台获取是否发现敌军
//第一位 是否发现敌军 (0:未发现 1：发现)
//
void GetMsgFromYunTai(char*s)
{
	SystemState.Robote.FindEnemy=(GarderFindEnemyDef)(s[0]& 0x01);//是否发现敌军
	SystemState.Robote.YuntaiApplay=(YuntaiApplayDef)((s[0]>>1) & 0x01);//云台申请控制底盘
	SystemState.Robote.Yuntai_CTR_APP_SPD=(short)s[1]<<8|s[2];//云台控制申请速度解析
}

//从云台获取控制2006电机转速电流
void Get2006MotorMoveFromYunTai(char*s)
{
	SystemState.Robote.M2006_A_Current=(short)s[4]<<8|s[5];
	SystemState.Robote.M2006_B_Current=(short)s[6]<<8|s[7];
}

float g_BitByHeroTime=0; //被英雄攻击时间
unsigned short g_HP_Pre=0;//上次血量
//哨兵模式切换
void GarderModeChange()
{
	if(g_HP_Pre-JudgeMent.GameRobotState.remainHP>490)
		g_BitByHeroTime=GetSystemTime();//记录当前被英雄击打时间
	g_HP_Pre=JudgeMent.GameRobotState.remainHP;//更新时间
	switch(SystemState.Robote.RunState)
	{
		case State_AutoCTR:
				
				if(JudgeMent.GameRobotState.remainHP<=1500 ||  (GetSystemTime()-g_BitByHeroTime<5000))//如果血量少于1000 或者被英雄攻击 切换为暴走模式   最高优先级
				{
						SystemState.Robote.Mode=Mode_Crazy;//切换为暴走模式 
						return;
				}
				else
				{	
					if(SystemState.Robote.Mode==Mode_Crazy)//如果血量高于1000 将暴走模式释放到巡航模式
						SystemState.Robote.Mode=Mode_Cruise;
				}
			
				if(Enemy_Finded==SystemState.Robote.FindEnemy)//找到敌军
				{
					if(SystemState.Robote.Mode==Mode_Cruise)
					{
						SystemState.Robote.Mode=Mode_Attack;//转换为攻击模式
					}
				}
				else
				{
					if(SystemState.Robote.Mode==Mode_Attack || SystemState.Robote.Mode==Mode_Foolish)
						SystemState.Robote.Mode=Mode_Cruise;//没有发现敌军 释放回巡航模式
				}
						
				//处理云台申请
				if(SystemState.Robote.YuntaiApplay==YuntaiCTR_Applay  //云台控制申请 且不是暴走模式不是防御模式
						&& SystemState.Robote.Mode!=Mode_Defense 
						&& SystemState.Robote.Mode!=Mode_Crazy)
					SystemState.Robote.Mode=Mode_YuntaiCTR;//更改为云台控制
				else
				{
						if(SystemState.Robote.Mode==Mode_YuntaiCTR)//如果是云台不再控制将运行模式释放到巡航模式
							SystemState.Robote.Mode=Mode_Cruise;
				}
			break;
		
		case State_WifiCTR://wifi模式切换在WIFI接收处理函数那里赋值
			break;
		default:
			break;
	}
}


// 背朝基地 右侧为近端 0  左侧为远端 4500+  向左运行为正
#define FarDist 4400  //远端运行位置
#define NearMidDist 2200  //近端中部运行位置
#define NearDist 500  //近端运行位置
#define Cruise_SPD 500 // 巡航速度

#define Attack_SPD 1 //攻击模式下的速度
#define Attack_Dist 2 //攻击模式下的左右移动的距离

void CTR_CruiseMode()//巡航模式控制
{
	static char Run_Dir=0;// 0:向远端运行 1:向近端运行
	
	if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//如果裁判系统上线以及（开机超过超过90s或者进入对战中）
		SystemState.Robote.GarderFire=Fire_Permite;//允许开火
	else
		SystemState.Robote.GarderFire=Fire_Deny;//禁止开火
	
	if(Run_Dir==0)
	{
		Motor1.Com.Loc=FarDist;
		Motor2.Com.Loc=FarDist;
	}
	else
	{
		Motor1.Com.Loc=NearDist;
		Motor2.Com.Loc=NearDist;
	}
	
	Motor1.LOC_PID.moter_pid.Ulimit=Cruise_SPD;
	Motor2.LOC_PID.moter_pid.Ulimit=Cruise_SPD;
	
	//通过传感器控制反向  SystemState.Robote.Limite_Left
	if(Motor1.State.Loc>FarDist-100)Run_Dir=1;
	//if(SystemState.Robote.Limite_Left < 500 || Motor1.State.Loc>8000)Run_Dir=1;
	if(Motor1.State.Loc<NearDist+100)Run_Dir=0;
	
}

void CTR_AttackMode(float attack_Loc)//攻击模式控制
{
	static char Run_Dir=0;// 0:向远端运行 1:向近端运行
	
	if(attack_Loc+Attack_Dist>FarDist)attack_Loc=FarDist-Attack_Dist;
	if(attack_Loc-Attack_Dist<NearDist)attack_Loc=NearDist+Attack_Dist;
	
	if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//如果裁判系统上线以及（开机超过超过90s或者进入对战中）
		SystemState.Robote.GarderFire=Fire_Permite;//允许开火
	else
		SystemState.Robote.GarderFire=Fire_Deny;//禁止开火
	
	if(Run_Dir==0)
	{
		Motor1.Com.Loc=attack_Loc+Attack_Dist;
		Motor2.Com.Loc=attack_Loc+Attack_Dist;
	}
	else
	{
		Motor1.Com.Loc=attack_Loc-Attack_Dist;
		Motor2.Com.Loc=attack_Loc-Attack_Dist;
	}
	
	Motor1.LOC_PID.moter_pid.Ulimit=Attack_SPD;
	Motor2.LOC_PID.moter_pid.Ulimit=Attack_SPD;
	
	if(Motor1.State.Loc>(attack_Loc+Attack_Dist-20))
		Run_Dir=1;
	if(Motor1.State.Loc<(attack_Loc-Attack_Dist+20))
		Run_Dir=0;

}

void CTR_FoolishMode(float attack_Loc)//定点攻击模式控制
{
	if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//如果裁判系统上线以及（开机超过超过90s或者进入对战中）
		SystemState.Robote.GarderFire=Fire_Permite;//允许开火
	else
		SystemState.Robote.GarderFire=Fire_Deny;//禁止开火
	
	Motor1.Com.Loc=attack_Loc;
	Motor2.Com.Loc=attack_Loc;
}

void CTR_DefenseMode()
{
	static char Run_Dir=0;// 0:向远端运行 1:向近端运行
	
	SystemState.Robote.GarderFire=Fire_Deny;//禁止开火
	
	if(Run_Dir==0)
	{
		Motor1.Com.Loc=FarDist;
		Motor2.Com.Loc=FarDist;
	}
	else
	{
		Motor1.Com.Loc=NearDist;
		Motor2.Com.Loc=NearDist;
	}
	
	Motor1.LOC_PID.moter_pid.Ulimit=Cruise_SPD*1.5f;
	Motor2.LOC_PID.moter_pid.Ulimit=Cruise_SPD*1.5f;
	
	if(Motor1.State.Loc>FarDist-100)Run_Dir=1;
	if(Motor1.State.Loc<NearDist+100)Run_Dir=0;

}
int Crazy_n=0;
//暴走模式控制
void CTR_CrazyMode()
{
	static char Run_Dir=0;// 0:向远端运行 1:向近端运行
	static char Run_Loc=0;// 0为右侧到终点  1为左侧到终点
	static char Run_Dir_pre=1;

	
	if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//如果裁判系统上线以及（开机超过超过90s或者进入对战中）
		SystemState.Robote.GarderFire=Fire_Permite;//允许开火
	else
		SystemState.Robote.GarderFire=Fire_Deny;//禁止开火
	
	if(Run_Loc==0)//从右侧到中点
	{
		if(Run_Dir==0)
		{
			Motor1.Com.Loc=NearMidDist-400;
			Motor2.Com.Loc=NearMidDist-400;
		}
		else
		{
			Motor1.Com.Loc=NearDist+100;
			Motor2.Com.Loc=NearDist+100;
		}
	}
	else //从左侧到中点
	{
		if(Run_Dir==0)
		{
			Motor1.Com.Loc=FarDist-100;
			Motor2.Com.Loc=FarDist-100;
		}
		else
		{
			Motor1.Com.Loc=NearMidDist+400;
			Motor2.Com.Loc=NearMidDist+400;
		}
		
	}

	
	if(JudgeMent.GameRobotState.remainHP>1000)
	{
		Motor1.LOC_PID.moter_pid.Ulimit=Cruise_SPD*1.9f;
		Motor2.LOC_PID.moter_pid.Ulimit=Cruise_SPD*1.9f;
	}
	else
	{
		Motor1.LOC_PID.moter_pid.Ulimit=Cruise_SPD*2.3f;
		Motor2.LOC_PID.moter_pid.Ulimit=Cruise_SPD*2.3f;
	}
		
	
	if(Run_Loc==0)//从右侧到中点
	{
		if(Motor1.State.Loc>NearMidDist-410)
			Run_Dir=1;
		if(Motor1.State.Loc<NearDist+110)
		  Run_Dir=0;
	}
	else
	{
		if(Motor1.State.Loc>FarDist-110)
			Run_Dir=1;

		if(Motor1.State.Loc<NearMidDist+410)
			Run_Dir=0;
	}
	
	if(Run_Dir_pre!=Run_Dir)Crazy_n--;
	Run_Dir_pre=Run_Dir;

	if(Crazy_n<=0)//通过循环次数等于0时 换位置跑
	{
		Crazy_n=SystemState.Time%8+1;//获取当前时间对5的余数进行循环次数的设定
		if(Run_Loc==0)
			Run_Loc=1;
		else
			Run_Loc=0;
	}
}

//WIFI控制模式  最右侧是0
void CTR_WIFIMode()
{
	short KeyCode=Wifi.Data.ch1;
	
	if(DBUS_CheckButtonState(KeyCode,KeyCode,KEY_W)==Key_Down)//按下前进
	{
		if(Motor1.State.Loc>NearDist)
		{
			Motor1.Com.Spd=-800;
			Motor2.Com.Spd=-800;
		}
		else
		{
			Motor1.Com.Spd=0;
			Motor2.Com.Spd=0;
		}
	}
	else if(DBUS_CheckButtonState(KeyCode,KeyCode,KEY_S)==Key_Down)
	{
		if(Motor1.State.Loc<FarDist)
		{
			Motor1.Com.Spd=800;
			Motor2.Com.Spd=800;
		}
		else
		{
			Motor1.Com.Spd=0;
			Motor2.Com.Spd=0;
		}
	}
	else
	{
		Motor1.Com.Spd=0;
		Motor2.Com.Spd=0;
	}
	
	if(DBUS_CheckButtonState(KeyCode,KeyCode,KEY_Z)==Key_Down)//模式切换为正常
	{
			SystemState.Robote.Mode=Mode_Cruise;//巡航模式
	}
	
	if(DBUS_CheckButtonState(KeyCode,KeyCode,KEY_Q)==Key_Down)//模式切换为防御模式
	{
			SystemState.Robote.Mode=Mode_Defense;
	}
	
	if(SystemState.Robote.Mode==Mode_Defense)//如果是防御模式
	{
		SystemState.Robote.GarderFire=Fire_Deny;//禁止开火
	}
	else
	{
		if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//如果裁判系统上线以及（开机超过超过90s或者进入对战中）
		SystemState.Robote.GarderFire=Fire_Permite;//允许开火
	else
		SystemState.Robote.GarderFire=Fire_Deny;//禁止开火
	}

}

//云台控制模式
void CTR_YuntaiCTRMode()
{
	if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//如果裁判系统上线以及（开机超过超过90s或者进入对战中）
		SystemState.Robote.GarderFire=Fire_Permite;//允许开火
	else
		SystemState.Robote.GarderFire=Fire_Deny;//禁止开火
	if(Motor1.State.Loc>FarDist & SystemState.Robote.Yuntai_CTR_APP_SPD>0)//限制远端速度
	{
		Motor1.Com.Spd=0;
		Motor2.Com.Spd=0;
	}
	else if(Motor1.State.Loc<NearDist & SystemState.Robote.Yuntai_CTR_APP_SPD<0)//限制近端速度
	{
		Motor1.Com.Spd=0;
		Motor2.Com.Spd=0;
	}
	else
	{
		if(my_abs(SystemState.Robote.Yuntai_CTR_APP_SPD)>3000)//速度限幅
		{
				if(SystemState.Robote.Yuntai_CTR_APP_SPD>0)
				{
					Motor1.Com.Spd=3000;
					Motor2.Com.Spd=3000;
				}else
				{
					Motor1.Com.Spd=-3000;
					Motor2.Com.Spd=-3000;
				}
		}
		else
		{
			Motor1.Com.Spd=SystemState.Robote.Yuntai_CTR_APP_SPD;
			Motor2.Com.Spd=SystemState.Robote.Yuntai_CTR_APP_SPD;
		}
	}
}
