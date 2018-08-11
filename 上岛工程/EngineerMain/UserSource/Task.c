#include "Task.h"
#include <string.h>

#include "iwdg.h"

#include "MyCom.h"
#include "SystemState.h"
#include "bsp_can.h"
#include "bsp_Uart.h"
#include "bsp_motor.h"
#include "Remote.h"
#include "Camera.h"
#include "LaserRangeFinder.h"


void vMotorGetZero_Task(void *pvParameters);//归零任务
void vTask_GetBullet(void *pvParameters);//获取子弹任务

void KeyBoardCtr(void);//键盘控制   再机器人控制中调用
void SetLimitAndTransToMotor(void);//限幅与电机命令传递

void vTask_DownIsland(void *pvParameters);//上岛任务
void vTask_UpIsland(void *pvParameters);//下岛任务
void vAutoMove_Task(void *pvParameters);//自动移动任务

//机器人控制任务
void vRobotCtrTask(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		GetTaskPeriod(RobotCtr_Task_No);
	
		//======================================================设定运行模式
		if(SystemState.Mode==SystemMode_Set_Zero)//判断系统是否在归零状态
		{
			switch(RC_Data.switch_right)
			{
				case 2:
					SystemState.Enable=0;//失能所有
				  break;
				default:
					SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
					break;
			}
		}
		else
		{
			switch(RC_Data.switch_right)//操作控制
			{
				case 1://电脑控制 
					SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
					SystemState.Mode=SystemMode_KeyBoard_Control;//键盘控制
					break;
				case 3://遥控器控制
					switch(RC_Data.switch_left)
					{
							case 1://升降机构控制 松开
								SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
								SystemState.Mode=SystemMode_Remote_LiftControl;
								
								SystemState.Robot.Catch=0;

							break;
							case 3:  //地盘控制
								SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
								SystemState.Mode=SystemMode_Remote_MoveControl;
							break;
							case 2:  //升降机构控制 夹合
								SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
								SystemState.Mode=SystemMode_Remote_LiftControl;
							
								SystemState.Robot.Catch=1000;//夹合

							break;
					}
					break;
					
				case 2:
					SystemState.Enable=0;//失能所有
				  SystemState.Mode=SystemMode_Disable;//失能模式
					break;
			}
		}

		//=======================================================根据模式控制
		switch(SystemState.Mode)//运行模式
		{
			case SystemMode_Remote_MoveControl:
				
				SystemState.Robot.Move_Y_SPD=4*RC_Data.ch4;//地盘前进
			  SystemState.Robot.Move_X_SPD=4*RC_Data.ch3;//地盘横移
			  SystemState.Robot.Move_Z_SPD=9.5*RC_Data.ch1;//地盘旋转
				SystemState.Robot.Lift_Z_Loc+= 0.01*RC_Data.ch2;//悬架提升

			break;
			
			case SystemMode_Remote_LiftControl:
				
				SystemState.Robot.Move_Y_SPD=0;
			  SystemState.Robot.Move_X_SPD=0;
			  SystemState.Robot.Move_Z_SPD=0;
				
				SystemState.Robot.Lift_Z_Loc+= 0.01*RC_Data.ch2;//悬架提升
			
				SystemState.Robot.Lift_Y_SPD = 2*RC_Data.ch4;//悬架前进
				SystemState.Robot.Lift_Y_Loc += 0.01*RC_Data.ch4;//悬架前进
				SystemState.Robot.Move_Y_SPD=2*RC_Data.ch4;//地盘前进
			
				SystemState.Robot.Move_X_SPD = 4*RC_Data.ch3;//地盘横移
				SystemState.Robot.Lift_A_Loc+= 0.01*RC_Data.ch1;//弹箱旋转获取
			
			break;
			
			case SystemMode_KeyBoard_Control:
				switch(RC_Data.switch_left)//键盘模式下的摇杆控制
				{
					case 3: //左边拨中间
							if(RC_Data.ch4>600)//上岛任务
							{
								if(!MyFlagGet(SystemState.Task,SystemTask_UpIsland))
								{
									xTaskCreate( vTask_UpIsland, "UpIsland_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_UpIsland);//启动上岛任务
								}
							}
							if(RC_Data.ch4<-600)//下岛任务
							{
								if(!MyFlagGet(SystemState.Task,SystemTask_DownIsland))
								{
									xTaskCreate( vTask_DownIsland, "DownIsland_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_DownIsland);//启动下岛任务
								}
							}
							
							if(RC_Data.ch3<-600)//获取子弹任务
							{
								if(!MyFlagGet(SystemState.Task,SystemTask_GetBullet))
								{
									xTaskCreate( vTask_GetBullet, "GetBullet_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_getbullet );//启动获取任务
								}
							}
							
							if(RC_Data.ch3>600)//取消任务
							{
								if(SystemState.TaskHandle.hTask_DownIsland!=NULL)
								{
									vTaskDelete(SystemState.TaskHandle.hTask_DownIsland);//退出任务
									SystemState.TaskHandle.hTask_DownIsland=NULL;
									SystemState.UDMode=UDMode_None;
									MyFlagClear(SystemState.Task,SystemTask_DownIsland);//清除上岛任务标志位
								}
								if(SystemState.TaskHandle.hTask_UpIsland!=NULL)
								{
									vTaskDelete(SystemState.TaskHandle.hTask_UpIsland);//退出任务
									SystemState.TaskHandle.hTask_UpIsland=NULL;
									SystemState.UDMode=UDMode_None;
									MyFlagClear(SystemState.Task,SystemTask_UpIsland);//清除上岛任务标志位
								}	
							}

						break;
				}
			
				KeyBoardCtr();//键盘控制
				break;
			case SystemMode_Disable://所有失能
				if(RC_Data.ch1<-600 &&RC_Data.ch2<-600 &&RC_Data.ch3>600 &&RC_Data.ch4<-600)
				{
					SystemState.Mode=SystemMode_Set_Zero;
					while(RC_Data.switch_right==2)
					{
						vTaskDelay(100/ portTICK_RATE_MS);
					}
					xTaskCreate( vMotorGetZero_Task, "MotorZero_Task", 200, NULL, 3, NULL );//电机归零
				}
				break;
		}

		
		
		SetLimitAndTransToMotor();//将命令传递给电机
		vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	}
}

//==============================================================
//
//		电机归零任务(记得更改遥控器状态 要不然遥控器会将速度值覆盖)
//
//
//===============================================================
void vMotorGetZero_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int num;
	char tem_flag;
		
	vTaskDelayUntil(&xLastWakeTime,200/ portTICK_RATE_MS);
	My_print(">>MotorGetZero...Start\r\n");
	while(1)
	{
		if((SystemState.OutLine_Flag&0x7ff)==0)//等待遥控器与提升部分上线
				break;
		if(RC_Data.switch_right==2)break;//遥控器取消了等待
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	while(RC_Data.switch_right==2)//等待遥控器抬起右侧按键
	{
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	if(SystemState.Time<10000)
		vTaskDelayUntil(&xLastWakeTime,2000/ portTICK_RATE_MS);
	
	SystemState.Mode=SystemMode_Set_Zero;//设定设备处于校准状态
	SystemState.Enable|=SystemState_Enable_Lift;//使能提升机构

	MoveMotor_EN_ALLSET(Motor_En_Calibration);//设置所有地盘电机已校准
	YuntaiMotor_EN_ALLSET(Motor_En_Calibration);//设置所有云台电机已校准
	LiftMotor_EN_ALLSET(Motor_En_Enable);//设置所有提升电机使能
	
	SystemState.Robot.EleBreak=1000;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);

	//设置需要校准的电机为速度控制
	MotorA.pid_control_type=MotorSPD_Ctr;
	MotorB.pid_control_type=MotorSPD_Ctr;
	MotorC.pid_control_type=MotorSPD_Ctr;
	MotorD.pid_control_type=MotorSPD_Ctr;
	
	//设定归零速度
	MotorA.Com.Spd=-100;
	MotorB.Com.Spd=100;
	MotorC.Com.Spd=100;
	MotorD.Com.Spd=-100;

	for(num=MotorA_No;num<MotorTotal_No;num++)//减小堵转阈值
	{
		Motors[num]->Protect.OutPut_Stall=Motors[num]->Protect.OutPut_Stall/2;
	}
	
	while(1)
	{
		for(num=MotorA_No;num<=MotorD_No;num++)//判断堵转 失能电机并清零位置
		{
			if(GetMotorStall(Motors[num]->State.State))//判断堵转
			{
				Motor_Zero(Motors[num]);//电机归零
				Motors[num]->Com.En=Motor_En_Calibration;//电机已校准
				Motors[num]->pid_control_type=MotorLOC_Ctr;//恢复电机为位置控制
			}
		}
		tem_flag=0;
		for(num=MotorA_No;num<=MotorD_No;num++)//检测电机是否都校准完毕
		{
			if(Motors[num]->Com.En!=Motor_En_Calibration)tem_flag++;
		}
		if(!tem_flag)break;//如果都已校准 跳出归零运行阶段
		
		if(RC_Data.switch_right==2)break;//遥控器取消了归零
		
		vTaskDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);	
	}

	for(num=MotorA_No;num<MotorTotal_No;num++)//增大堵转输出阈值
	{
		Motors[num]->Protect.OutPut_Stall=Motors[num]->Protect.OutPut_Stall*2;
	}
	
	MotorA.pid_control_type=MotorLOC_Ctr;
	MotorB.pid_control_type=MotorLOC_Ctr;
	MotorC.pid_control_type=MotorLOC_Ctr;
	MotorC.State.Loc=5;
	MotorD.pid_control_type=MotorLOC_Ctr;
	
	SystemState.Mode=SystemMode_Remote_LiftControl;//更改系统状态为遥控器控制模式

	My_print(">>MotorGetZero....OK\r\n");
	
	vTaskDelete(NULL);//退出任务
}

//LED闪烁
void vLED_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int Delay_t=500;
	while(1)
	{
		GetTaskPeriod(LED_Task_No);//获取运行间隔
		
		if(SystemState.OutLine_Flag&0x7ff)
			Delay_t=283;
		else
			Delay_t=950;
		vTaskDelayUntil(&xLastWakeTime,Delay_t/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
	}
}

//LED2闪烁
void vLED2_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	short Delay=450;
	int canSentn=0;
	while(1)
	{
		if(canSentn==g_CanSent_n)
			Delay=200;
		else
			Delay=950;
		canSentn=g_CanSent_n;
		
		vTaskDelayUntil(&xLastWakeTime,Delay/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
	}
}
void LED_OutLineBreath(char*s);
//外部LED
void vSW2812_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	char s[100]={0};
	while(1)
	{
		memset(s,0,100);
		if(MyFlagGet(SystemState.Task,SystemTask_UpIsland))//上岛任务
		{
			s[0]='<';s[1]=7;s[2]='>';
			memcpy(s+3,"\xff\xff\xff\x01\x11\x0a\x0d",7);
			UART_SendToQueue(&huart4,s);
		}
		else if(MyFlagGet(SystemState.Task,SystemTask_DownIsland))//下岛任务
		{
			s[0]='<';s[1]=7;s[2]='>';
			memcpy(s+3,"\xff\xff\xff\x01\x21\x0a\x0d",7);
			UART_SendToQueue(&huart4,s);
		}
		else if(MyFlagGet(SystemState.Task,SystemTask_GetBullet))//取蛋任务
		{
			s[0]='<';s[1]=7;s[2]='>';
			memcpy(s+3,"\xff\xff\xff\x01\x30\x0a\x0d",7);
			UART_SendToQueue(&huart4,s);
		}
		else	
		{
			s[0]='<';s[1]=31;s[2]='>';
			memcpy(s+3,"\xff\xff\xff\x19\x50\x16\x00\x00\x16\x00\x00\x16\x00\x00\x16\x00\x00\x16\x00\x00\x16\x00\x00\x16\x00\x00\x16\x00\x00\x0a\x0d",31);
			LED_OutLineBreath(&s[8]);
			UART_SendToQueue(&huart4,s);
		}
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
}

//蜂鸣器任务
void vBEEP_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int Delay_Being=0;
	int Delay_Silence=200;
	
	HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
	vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
	HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
		
	while(1)
	{
		GetTaskPeriod(Beep_Task_No);//获取运行间隔
		
		switch(SystemState.Beep)
		{
			case Beep_Beeping:
				Delay_Being=200;
				Delay_Silence=0;
				break;
			case Beep_Silence:
				Delay_Being=0;
				Delay_Silence=200;
				break;
		}
		
		if(Delay_Being)
		{
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
			vTaskDelayUntil(&xLastWakeTime,Delay_Being/ portTICK_RATE_MS);
		}
		if(Delay_Silence)
		{
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
			vTaskDelayUntil(&xLastWakeTime,Delay_Silence/ portTICK_RATE_MS);
		}
	}
}

//==============================================================
//
//		上下岛屿获取子弹任务    上下岛过程速度控制需要有独占性
//
//
//===============================================================
//获取子弹
#define GetBullet_degree 200
void vTask_GetBullet(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	MyFlagSet(SystemState.Task,SystemTask_GetBullet);//标志获取弹药箱标志位
	
	MotorC.Com.En=Motor_En_Calibration;//使能电机
	//=======================================================
	SystemState.Robot.GetBulletFrame=1000;//提升架
	SystemState.Robot.Catch=0;//夹子打开
	SystemState.Robot.Lift_A_Loc=GetBullet_degree;//旋转到GetBullet_degree
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待旋转电机完成
	{
		if(GetMotorLocNear(MotorC.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	SystemState.Robot.Catch=1000;//夹子闭合
	vTaskDelayUntil(&xLastWakeTime,300/ portTICK_RATE_MS);
	
	//=======================================================
	SystemState.Robot.Lift_A_Loc=10;//旋转到30
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待旋转电机完成
	{
		if(GetMotorLocNear(MotorC.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
	
	//=======================================================
	SystemState.Robot.Lift_A_Loc=185;//旋转到GetBullet_degree
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	//MotorC.LOC_PID.moter_pid.Ulimit+=200;
	
	while(1)//等待旋转电机完成
	{
		if(GetMotorLocNear(MotorC.State.State))
			break;
		
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	//MotorC.LOC_PID.moter_pid.Ulimit-=200;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	//=======================================================
	SystemState.Robot.Catch=0;//夹子打开
	SystemState.Robot.Lift_A_Loc=5;//旋转到5(回位)
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待旋转电机完成
	{
		if(GetMotorLocNear(MotorC.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	MotorC.Com.En=Motor_En_Disable;//失能电机
	vTaskDelayUntil(&xLastWakeTime,200/ portTICK_RATE_MS);
	MotorC.Com.En=Motor_En_Calibration;//使能电机
	
	SystemState.Robot.GetBulletFrame=0;//落回提升架
	SystemState.GetBulletFinish=1;
	MyFlagClear(SystemState.Task,SystemTask_GetBullet);//清除获取弹箱标志位
	SystemState.TaskHandle.hTask_getbullet=NULL;//清除获取子弹任务句柄
	vTaskDelete(NULL);//退出任务
}

//上岛任务(需要独占车辆控制权)
void vTask_UpIsland(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	MyFlagSet(SystemState.Task,SystemTask_UpIsland);
	SystemState.UDMode=UDMode_UpIsland;
	My_print("UpIsland Task Start....\r\n");
	
	//提升至所需高度
	SystemState.Robot.Lift_Z_Loc=305;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待提升电机完成提升
	{
		if(GetMotorLocNear(MotorA.State.State)
			&&GetMotorLocNear(MotorB.State.State))
			break;
		if(MotorA.State.Loc>285 || MotorB.State.Loc>285)break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	//前进到后端传感器检测到位置
	Motor5.pid_control_type=MotorSPD_Ctr;//小轮电机更改为速度控制
	Motor6.pid_control_type=MotorSPD_Ctr;
	
	SystemState.Robot.Move_Y_SPD=300;
	SystemState.Robot.Lift_Y_SPD=300;
	while(1)
	{
		if(SystemState.DistSensor.Dis_RB<20 && SystemState.DistSensor.Dis_LB<20)
		{
			SystemState.Robot.Move_Y_SPD=0;
			SystemState.Robot.Lift_Y_SPD=0;
			SystemState.Robot.Lift_Y_Loc=0;
			Motor_Zero(&Motor5);//电机归零
			Motor_Zero(&Motor6);//电机归零
			break;
		}
		vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	}	
	
	//落地	
	SystemState.Robot.Lift_Z_Loc=40;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	
	while(1)//等待提升电机完成提升
	{
		if(GetMotorLocNear(MotorA.State.State)
			&&GetMotorLocNear(MotorB.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	MyFlagClear(SystemState.Task,SystemTask_UpIsland);//清除上岛任务标志位
	SystemState.UDMode=UDMode_None;//清除上下岛模式
	SystemState.TaskHandle.hTask_UpIsland=NULL;//清除上岛任务句柄
	My_print("---UpIsland Task Finish.\r\n");
	SystemState.GetBulletFinish=0;//取弹完成标志清零
	vTaskDelete(NULL);//退出任务
}

//下岛
void vTask_DownIsland(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	SystemState.UDMode=UDMode_DownIsland;
	MyFlagSet(SystemState.Task,SystemTask_DownIsland);
	My_print("DownIsland Task Start....\r\n");
	//对正位置
	while(1)
	{
		if(SystemState.DistSensor.Dis_RB<20 && SystemState.DistSensor.Dis_LB>20)//左后出去了
		{
			SystemState.Robot.Move_Y_SPD=0;
			SystemState.Robot.Move_Z_SPD=+300;
		}
		if(SystemState.DistSensor.Dis_RB>20 && SystemState.DistSensor.Dis_LB<20)//右后出去了
		{
			SystemState.Robot.Move_Y_SPD=0;
			SystemState.Robot.Move_Z_SPD=-300;
		}
		if(SystemState.DistSensor.Dis_RB<20 && SystemState.DistSensor.Dis_LB<20)//两端传感器均在里面
		{
			SystemState.Robot.Move_Y_SPD=-100;
			SystemState.Robot.Move_Z_SPD=0;
		}
		if(SystemState.DistSensor.Dis_RB>20 && SystemState.DistSensor.Dis_LB>20)//两端传感器均在外边
		{
			SystemState.Robot.Move_Y_SPD=0;
			SystemState.Robot.Move_Z_SPD=0;
			break;
		}
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	//提升至所需高度
	SystemState.Robot.Lift_Z_Loc=300;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待提升电机完成提升
	{
		if(GetMotorLocNear(MotorA.State.State)
			&&GetMotorLocNear(MotorB.State.State))
			break;
		if(MotorA.State.Loc>285 || MotorB.State.Loc>285)
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	//后退到前端传感器检测到位置
	SystemState.Robot.Move_Y_SPD=-250;
	SystemState.Robot.Lift_Y_SPD=-150;
	while(1)
	{
		if(SystemState.DistSensor.Dis_RF>20 && SystemState.DistSensor.Dis_LF>20)
		{
			SystemState.Robot.Move_Y_SPD=0;
			SystemState.Robot.Lift_Y_Loc=0;
			SystemState.Robot.Lift_Y_SPD=0;
			Motor_Zero(&Motor5);//电机归零
			Motor_Zero(&Motor6);//电机归零
			break;
		}
		vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	}
	Motor5.pid_control_type=MotorLOC_Ctr;//小轮电机更改为位置控制
	Motor6.pid_control_type=MotorLOC_Ctr;
	
	SystemState.Robot.Lift_Y_Loc=-20;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待小轮转到位置
	{
		if(GetMotorLocNear(Motor5.State.State)
			&&GetMotorLocNear(Motor6.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	SystemState.Robot.Lift_Y_Loc=0;
	Motor_Zero(&Motor5);//电机归零
	Motor_Zero(&Motor6);//电机归零
	
	Motor5.pid_control_type=MotorSPD_Ctr;//小轮电机更改为速度控制
	Motor6.pid_control_type=MotorSPD_Ctr;
	//落地	
	SystemState.Robot.Lift_Z_Loc=10;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待提升电机完成提升
	{
		if(GetMotorLocNear(MotorA.State.State)
			&&GetMotorLocNear(MotorB.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	MyFlagClear(SystemState.Task,SystemTask_DownIsland);//清除下岛任务标志位
	SystemState.UDMode=UDMode_None;//清除上下岛模式
	SystemState.TaskHandle.hTask_DownIsland=NULL;//清除获取子弹任务句柄
	My_print("---DownIsland Task Finish.\r\n");
	SystemState.GetBulletFinish=0;//取弹完成标志清零
	vTaskDelete(NULL);//退出任务
}

#define AutoMove_Dis 325
//自动移动任务
void vAutoMove_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	float M1_Loc=Motor1.State.Loc;
	
	int* dir=(int*)pvParameters;
	MyFlagSet(SystemState.Task,SystemTask_AutoMove);//设置自动移动任务标志
	
	if(*dir==1)
		SystemState.Robot.Move_X_SPD=200;
	else
		SystemState.Robot.Move_X_SPD=-200;
	while(1)
	{
		if(My_abs(Motor1.State.Loc-M1_Loc)>=AutoMove_Dis)
			break;
		vTaskDelayUntil(&xLastWakeTime,30/ portTICK_RATE_MS);
	}
	
	MyFlagClear(SystemState.Task,SystemTask_AutoMove);//清除自动移动任务标志
	SystemState.TaskHandle.hTask_automove=NULL;
	
	SystemState.GetBulletFinish=0;//取弹完成标志清零
	vTaskDelete(NULL);//退出任务
}

//自动对准任务  (没有启动) 需要修改串口
void vAutoAim_Task(void *pvParameters)
{
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	char s[20]={0};
	while(1)
	{
		GetTaskPeriod(Test_Task_No);//获取运行间隔
		
		memset(s,0,20);
		Camera_UART_Send_Buff((unsigned char*)s,CALL_Command,0xffffffff);
		if(RC_Data.mouse.press_right||(RC_Data.switch_right==1 &&RC_Data.switch_left==2))
			My_print(s);
		vTaskDelayUntil(&xLastWakeTime,30/ portTICK_RATE_MS);

	}
}

//================================================================
//****************************************************************
//****************************看门狗****************************
//****************************************************************
//=================================================================

void vFEEDDOG_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	vTaskDelayUntil(&xLastWakeTime,2000/ portTICK_RATE_MS);
	__HAL_IWDG_START(&hiwdg);//开启看门狗
	
	while(1)
	{
		//__HAL_IWDG_RELOAD_COUNTER(&hiwdg);//更新看门狗
		
		if(MyFlagGet(SystemState.OutLine_Flag,RemoteCtr_No+MotorTotal_No))//如果遥控器断线
		{
			RC_Data.ch1=0;
			RC_Data.ch2=0;
			RC_Data.ch3=0;
			RC_Data.ch4=0;
			RC_Data.keyBoard.key_code=0;
			RC_Data.keyBoard.jumpkey_code=0;
			RC_Data.mouse.x=0;
			RC_Data.mouse.y=0;
			RC_Data.mouse.press_left=0;
			RC_Data.mouse.press_right=0;
		}
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
}


//================================================================
//****************************************************************
//****************************测试任务****************************
//****************************************************************
//=================================================================
int g_mem_min=0;//最小内存剩余
int g_mem_n=0;//当前内存剩余
void vTest_Task(void *pvParameters)
{
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	//char data[9+3]={0};
	while(1)
	{
		/*data[0]='<';
		data[1]='9';
		data[2]='>';
		VisualScope_DataFill(&data[3],Motor5.State.Spd,Motor5.Com.Spd,Motor5.State.Loc,Motor5.Com.Loc);
		My_print(data);*/
		
		//g_mem_min=xPortGetMinimumEverFreeHeapSize();
		//g_mem_n=xPortGetFreeHeapSize();
		
		vTaskDelayUntil(&xLastWakeTime,1000/ portTICK_RATE_MS);
	}
  //vTaskDelete(NULL);//退出任务
}



//=====================================================================================
//                                 其他函数
//
//
//=====================================================================================
void LED_OutLineBreath(char*s)
{
	int i=0;
	for(i=Motor1_No;i<Motor5_No;i++)
	{
		if(MyFlagGet(SystemState.OutLine_Flag,i))
		{
			s[3*i+0]=0;//将绿色清零
			s[3*i+1]=100;//将红色亮起
		}
	}
	
	for(i=Motor5_No;i<MotorTotal_No;i++)
	{
		if(MyFlagGet(SystemState.OutLine_Flag,i))
		{
			s[3*(i-Motor5_No)+0]=0;//将绿色清零
			s[3*(i-Motor5_No)+2]=100;//将蓝色亮起
		}
	}
	
	if(MyFlagGet(SystemState.OutLine_Flag,RemoteCtr_No+MotorTotal_No))//遥控器未连接
	{
		s[21]=0;s[23]=100;
	}
	
	if(MyFlagGet(SystemState.OutLine_Flag,DisMesur_No+MotorTotal_No))//测距板
	{
		s[18]=0;s[20]=100;
	}
		
	if(MyFlagGet(SystemState.OutLine_Flag,YuntaiCtr_No+MotorTotal_No))//云台控制板
	{
		s[15]=0;s[17]=100;
	}
}


//限幅与命令传递 vRobotCtr中调用
void SetLimitAndTransToMotor()
{
	//提升机构限幅
		if(SystemState.Robot.Lift_Z_Loc<10)SystemState.Robot.Lift_Z_Loc=10;if(SystemState.Robot.Lift_Z_Loc>Lift_Limite)SystemState.Robot.Lift_Z_Loc=Lift_Limite;
		if(SystemState.Robot.Lift_A_Loc<0)SystemState.Robot.Lift_A_Loc=0;if(SystemState.Robot.Lift_A_Loc>200)SystemState.Robot.Lift_A_Loc=200;
		
		SpeedDistribute();//速度分解并赋给电机轮
	
				
		MotorA.Com.Loc=SystemState.Robot.Lift_Z_Loc;//抬升
		MotorB.Com.Loc=-SystemState.Robot.Lift_Z_Loc;//抬升

		MotorC.Com.Loc=-SystemState.Robot.Lift_A_Loc;//旋转
		MotorD.Com.Loc=	SystemState.Robot.Guide_Frame;//导流架角度
		
		Motor5.Com.Spd=-SystemState.Robot.Lift_Y_SPD;
		Motor6.Com.Spd= SystemState.Robot.Lift_Y_SPD;
		Motor5.Com.Loc=-SystemState.Robot.Lift_Y_Loc;//闭环前进距离
		Motor6.Com.Loc= SystemState.Robot.Lift_Y_Loc;//闭环前进距离
	
		if(MotorA.Com.Loc<0)MotorA.Com.Loc=0;if(MotorA.Com.Loc> Lift_Limite)MotorA.Com.Loc= Lift_Limite;
		if(MotorB.Com.Loc>0)MotorB.Com.Loc=0;if(MotorB.Com.Loc<-Lift_Limite)MotorB.Com.Loc=-Lift_Limite;
		if(MotorC.Com.Loc>0)MotorC.Com.Loc=0;if(MotorC.Com.Loc<-200)MotorC.Com.Loc=-200;
		if(MotorD.Com.Loc<0)MotorD.Com.Loc=0;if(MotorD.Com.Loc>135)MotorD.Com.Loc=135;
		
		if(SystemState.Robot.YunTai.Com_Pich>60)SystemState.Robot.YunTai.Com_Pich=60;
		if(SystemState.Robot.YunTai.Com_Pich<-70)SystemState.Robot.YunTai.Com_Pich=-70;
		
		if(SystemState.Robot.YunTai.Com_Pich>240)SystemState.Robot.YunTai.Com_Pich=240;
		if(SystemState.Robot.YunTai.Com_Pich<-180)SystemState.Robot.YunTai.Com_Pich=-180;
}


short BoardKeyCode=0;//本次键盘码
short BoardKeyCode_Pre=0;//上次键盘码
char Mouse_L=0;//鼠标左键状态
char Mouse_L_Pre=0;//上次鼠标左键状态
char Mouse_R=0;//鼠标左键状态
char Mouse_R_Pre=0;//上次鼠标左键状态

#define MOVE_MaxSPD 2500 //速度最大值
#define Move_ACC 40 //最大加减速


//键盘控制   再机器人控制中调用
void KeyBoardCtr()
{
	float SPD_k=1;//速度控制系数
	static int Times_n=0;
	float Shift_Key=0;//Shift按下
	float CTR_Key=0;//CTR按下
	short V_Key=0;//V按下
	short G_Key=0;//G按下
	static float SPD_Max=MOVE_MaxSPD;//速度最大值
	static float SPD_W_Max=MOVE_MaxSPD;//速度最大值
	float SPD_ACC=Move_ACC;//加减速
	
	static float Key_Q_t=0;//上次按下Q的时间
	static float Key_E_t=0;//上次按下E的时间
	static float Key_C_t=0;//上次按下C的时间
	static float Key_V_t=0;//上次按下V的时间
	static float Key_Z_t=0;//上次按下Z的时间
	static float Key_R_t=0;//上次按下R的时间
	static float Key_F_t=0;//上次按下F的时间
	static float Mouse_L_t=0;//上次按下鼠标左键的时间
	static float Mouse_R_t=0;//上次按下鼠标右键的时间
	
	//int AutoMoveDir=0;
	
	BoardKeyCode_Pre=BoardKeyCode;//存入上次按键状态量
	BoardKeyCode=RC_Data.keyBoard.key_code;//获取当前遥控变量
	Mouse_L_Pre=Mouse_L;//存入上次鼠标左键值
	Mouse_L=RC_Data.mouse.press_left;//获取本次鼠标左键值
	
	Mouse_R_Pre=Mouse_R;//存入上次鼠标左键值
	Mouse_R=RC_Data.mouse.press_right;//获取本次鼠标右键

	//=============================================================================== CTR 按键======慢速运动
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_CTRL)==Key_Down)
			CTR_Key=1;

	//=============================================================================== SHIFT 按键======升降
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_SHIFT)==Key_Down)
	{
		Shift_Key=1;//Shift按下
	}	
	//=============================================================================== V 按键======= 换向
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_V)==Key_Down)
	{
		V_Key=1;//V键按下
	}	
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_V)==Key_Fall)
	{
		if(GetSystemTimer()-Key_V_t<300)//双击V
		{
			if(SystemState.Robot.YunTai.Direct==Forward)
				SystemState.Robot.YunTai.Direct=Backward;
			else
				SystemState.Robot.YunTai.Direct=Forward;
		}

		
		Key_V_t=GetSystemTimer();
	}
	
	//=============================================================================== G 按键======= 激光测距
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_G)==Key_Down)
	{
		G_Key=1;
	}
	
	//=============================================================================== 鼠标
	
	if(SystemState.UDMode==UDMode_None &&	(!V_Key) )//在上下岛或者按下V的时候屏蔽鼠标控制车体转向
	{
		SystemState.Robot.Move_Z_SPD=RC_Data.mouse.x*30;//鼠标横向
	}
	else
	{
		if(SystemState.UDMode==UDMode_UpIsland || V_Key)
			SystemState.Robot.Move_Z_SPD=0;
	}
	
	//==================== 云台角度 控制
	if(V_Key)
	{
		SystemState.Robot.YunTai.Com_Yaw+=RC_Data.mouse.x*0.03;//云台Yaw角度
	}
	else
	{
		if(SystemState.Robot.YunTai.Direct==Forward)
			SystemState.Robot.YunTai.Com_Yaw=0;
		else
			SystemState.Robot.YunTai.Com_Yaw=-180;
	}
	SystemState.Robot.YunTai.Com_Pich-=RC_Data.mouse.y*0.02;//云台Pich角度
	
	
	if(DBUS_CheckButtonState(Mouse_L,Mouse_L_Pre,0x01)==Key_Fall){ //鼠标左键
		if(GetSystemTimer()-Mouse_L_t<300)//双击鼠标左键
		{
				SystemState.Robot.ReliefFinger=1000;//救援架释放
		}
		else
		{
				SystemState.Robot.ReliefFinger=0;//救援架
		}
		Mouse_L_t=GetSystemTimer();//单击鼠标L键
	}
	
	
	if(DBUS_CheckButtonState(Mouse_R,Mouse_R_Pre,0x01)==Key_Fall){ //鼠标右键
		if(GetSystemTimer()-Mouse_R_t<300)//双击鼠标右键
		{
			if(CTR_Key)
				SystemState.Robot.ReliefFrame=1000;//救援手指释放
			else
			{
				SystemState.Robot.ReliefFrame=1000;//救援手指释放
				SystemState.Robot.ReliefFrame2=1000;//救援手指释放(备用)
			}
				
		}
		else
		{
				SystemState.Robot.ReliefFrame=0;//救援手指收回
				SystemState.Robot.ReliefFrame2=0;//救援手指收回
		}
		Mouse_R_t=GetSystemTimer();//单击鼠标R键
	}
	


	if(SystemState.UDMode!=UDMode_None || MyFlagGet(SystemState.Task,SystemTask_AutoMove))//如果是上下岛模式 或者是自动运行模式 需要屏蔽按键
		goto CTR_Other;
	//=============================================================================== W 按键======
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_W)==Key_Down){//如果W按下
		if(Shift_Key)//如果Shift按下
		{
			if(SystemState.Robot.Lift_Z_Loc<100)//Shift按键按下
			{
				SPD_W_Max=MOVE_MaxSPD*1.5;//增加最高速度
				SPD_ACC=Move_ACC*2;
			}
			SystemState.Robot.Move_Y_SPD+=SPD_ACC * SystemState.Robot.YunTai.Direct;
		}
		else if(G_Key)//G按下   补弹防止跌落
		{
			if(SystemState.Robot.YunTai.Direct==Forward)//如果当前方向向前
			{
				if((SystemState.DistSensor.Dis_LF<20||SystemState.DistSensor.Dis_LF==255) //前方两个传感器防止跌落
					&& (SystemState.DistSensor.Dis_RF<20 || SystemState.DistSensor.Dis_RF==255))
				{
					SystemState.Robot.Move_Y_SPD=200 * SystemState.Robot.YunTai.Direct;
				}
				else
					SystemState.Robot.Move_Y_SPD=0;
			}else
			{
				if((SystemState.DistSensor.Dis_LB<20 || SystemState.DistSensor.Dis_LB==255)  //后方两个传感器防止跌落
					&& (SystemState.DistSensor.Dis_RB<20 || SystemState.DistSensor.Dis_RB==255))
				{
					SystemState.Robot.Move_Y_SPD=200 * SystemState.Robot.YunTai.Direct;
				}
				else
					SystemState.Robot.Move_Y_SPD=0;
			}
				
		}
		else if(CTR_Key)//CTR按下
		{
			if(SystemState.Robot.Lift_Z_Loc>100)
			{
				SystemState.Robot.Move_Y_SPD=300 * SystemState.Robot.YunTai.Direct;
				SystemState.Robot.Lift_Y_SPD=300 * SystemState.Robot.YunTai.Direct;
			}
			else
			{
				SystemState.Robot.Move_Y_SPD=800 * SystemState.Robot.YunTai.Direct;
			}
		}
		else
		{
			SystemState.Robot.Move_Y_SPD+=SPD_ACC * SystemState.Robot.YunTai.Direct;
		}
			
		if(My_abs(SystemState.Robot.Move_Y_SPD)>SPD_W_Max)SystemState.Robot.Move_Y_SPD=SPD_W_Max * SystemState.Robot.YunTai.Direct;
		
		//急刹车
		if(SystemState.Robot.YunTai.Direct==Forward)
		{
			if(SystemState.Robot.Move_Y_SPD<0) SystemState.Robot.Move_Y_SPD=0;
		}
		else
		{
			if(SystemState.Robot.Move_Y_SPD>0) SystemState.Robot.Move_Y_SPD=0;
		}
		
	}
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_W)==Key_Raise){
		SPD_W_Max=MOVE_MaxSPD;
		SPD_ACC=Move_ACC;
		SystemState.Robot.Lift_Y_SPD=0;
	}
	//=============================================================================== S 按键======
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_S)==Key_Down){
		if(CTR_Key)
		{
			if(SystemState.Robot.Lift_Z_Loc>100)
			{
				SystemState.Robot.Lift_Y_SPD=-300 * SystemState.Robot.YunTai.Direct;
			}
			
			SystemState.Robot.Move_Y_SPD=-300 * SystemState.Robot.YunTai.Direct;
		}
		else
		{
			SystemState.Robot.Move_Y_SPD-=SPD_ACC * SystemState.Robot.YunTai.Direct;
		}

		if(My_abs(SystemState.Robot.Move_Y_SPD)>SPD_Max)SystemState.Robot.Move_Y_SPD=-SPD_Max * SystemState.Robot.YunTai.Direct;
		//急刹车
		if(SystemState.Robot.YunTai.Direct==Forward)
		{
			if(SystemState.Robot.Move_Y_SPD>0) SystemState.Robot.Move_Y_SPD=0;
		}
		else
		{
			if(SystemState.Robot.Move_Y_SPD<0) SystemState.Robot.Move_Y_SPD=0;
		}
	}
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_S)==Key_Raise){
		SystemState.Robot.Lift_Y_SPD=0;
	}
	//=============================================================================== WS 都未按下=
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_W)==Key_Up && 
		 DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_S)==Key_Up)
	{
		if(My_abs(SystemState.Robot.Move_Y_SPD)>SPD_ACC)
			SystemState.Robot.Move_Y_SPD=(My_abs(SystemState.Robot.Move_Y_SPD)-SPD_ACC*SPD_k)*IsPositive(SystemState.Robot.Move_Y_SPD);
		else
			SystemState.Robot.Move_Y_SPD=0;
	}
	//=============================================================================== A 按键======
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_A)==Key_Down){
		if(CTR_Key)
		{
			SystemState.Robot.Move_X_SPD=-250 * SystemState.Robot.YunTai.Direct;
		}
		else
		{
			SystemState.Robot.Move_X_SPD-=SPD_ACC * SystemState.Robot.YunTai.Direct;
		}
		if(My_abs(SystemState.Robot.Move_X_SPD)>SPD_Max) SystemState.Robot.Move_X_SPD=-SPD_Max * SystemState.Robot.YunTai.Direct;
		
		if(SystemState.Robot.YunTai.Direct==Forward)
		{
			if(SystemState.Robot.Move_X_SPD>0) SystemState.Robot.Move_X_SPD=0;
		}
		else
		{
			if(SystemState.Robot.Move_X_SPD<0) SystemState.Robot.Move_X_SPD=0;
		}
	}
		
	//=============================================================================== D 按键======
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_D)==Key_Down){
		if(CTR_Key)
		{
			SystemState.Robot.Move_X_SPD=250 * SystemState.Robot.YunTai.Direct;
		}
		else
		{
			SystemState.Robot.Move_X_SPD+=SPD_ACC * SystemState.Robot.YunTai.Direct;
		}
		if(My_abs(SystemState.Robot.Move_X_SPD) > SPD_Max)SystemState.Robot.Move_X_SPD= SPD_Max * SystemState.Robot.YunTai.Direct;
		
		if(SystemState.Robot.YunTai.Direct==Forward)
		{
			if(SystemState.Robot.Move_X_SPD<0) SystemState.Robot.Move_X_SPD=0;
		}
		else
		{
			if(SystemState.Robot.Move_X_SPD>0) SystemState.Robot.Move_X_SPD=0;
		}
		
		

	}

	//=============================================================================== AD 都未按下=
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_A)!=Key_Down && 
		 DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_D)!=Key_Down)
	{
		if(My_abs(SystemState.Robot.Move_X_SPD)>SPD_ACC)
			SystemState.Robot.Move_X_SPD=(My_abs(SystemState.Robot.Move_X_SPD)-SPD_ACC*SPD_k)*IsPositive(SystemState.Robot.Move_X_SPD);
		else
			SystemState.Robot.Move_X_SPD=0;
	}
	
	
	//=============================================================================== R 按键====== 提升
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_R)==Key_Down&&(!CTR_Key)){
			SystemState.Robot.Lift_Z_Loc+=3;
	}
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_R)==Key_Fall && CTR_Key)
	{
		if(GetSystemTimer()-Key_R_t<300)// CTR+双击R
		{
			if(SystemState.Robot.YunTai.Direct==Forward)
			{
				if(!MyFlagGet(SystemState.Task,SystemTask_UpIsland))
				{
					xTaskCreate( vTask_UpIsland, "UpIsland_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_UpIsland);//启动上岛任务
					goto CTR_Other;
				}		
			}
			if(SystemState.Robot.YunTai.Direct==Backward)
			{
				if(!MyFlagGet(SystemState.Task,SystemTask_DownIsland))
				{
					xTaskCreate( vTask_DownIsland, "DownIsland_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_DownIsland);//启动下岛任务
					goto CTR_Other;
				}	
			}	
		}
		Key_R_t=GetSystemTimer();
	}
	//=============================================================================== F 按键====== 下降
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_F)==Key_Down &&(!CTR_Key)){
			SystemState.Robot.Lift_Z_Loc-=3;
	}
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_F)==Key_Fall && CTR_Key)
	{
		if(GetSystemTimer()-Key_F_t<300)// CTR+双击F
		{							
			if(!MyFlagGet(SystemState.Task,SystemTask_DownIsland))
			{
				xTaskCreate( vTask_DownIsland, "DownIsland_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_DownIsland);//启动下岛任务
				goto CTR_Other;
			}
		}
		Key_F_t=GetSystemTimer();
	}
	//=============================================================================== Q 按键====== 旋转弹箱
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_Q)==Key_Fall){
		if(CTR_Key)
		{
			if(GetSystemTimer()-Key_Q_t<300)//双击Q
			{
				SystemState.Robot.Lift_A_Loc=190;//倾倒
			}
			else
			{
				SystemState.Robot.Lift_A_Loc=0;//归位
			}	
		}
		Key_Q_t=GetSystemTimer();
	}
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_Q)==Key_Down)
	{
		if(!CTR_Key)
			SystemState.Robot.Move_Z_SPD=-2500;//逆时针旋转
	}
	//=============================================================================== E 按键=======抓取
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_E)==Key_Fall){
		if(CTR_Key)
		{
			if(GetSystemTimer()-Key_E_t<300)//双击E
			{
				SystemState.Robot.Catch=1000;//抓取
				
			}
			else
			{
				SystemState.Robot.Catch=0;//放松
				
			}
		}
		Key_E_t=GetSystemTimer();
	}
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_E)==Key_Down)
	{
		if(!CTR_Key)
			SystemState.Robot.Move_Z_SPD=2500;//顺时针旋转
	}
	
CTR_Other:
	//=============================================================================== Z 按键=======抓取位置1
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_Z)==Key_Fall){
		if(GetSystemTimer()-Key_Z_t<300 && (!CTR_Key))//双击Z
		{
			SystemState.Robot.GetBulletFrame=1000;
		}
		else
		{
			if(CTR_Key)//CTR+Z 撤销恢复
			{
				//首先取消任务
				if(SystemState.UDMode!=UDMode_None)
				{
					if(MyFlagGet(SystemState.Task,SystemTask_DownIsland))//如果在下岛 取消下岛任务 并将系统上下岛状态清除
					{
						if(SystemState.TaskHandle.hTask_DownIsland)
							vTaskDelete(SystemState.TaskHandle.hTask_DownIsland);//退出任务
						SystemState.TaskHandle.hTask_DownIsland=NULL;
						SystemState.UDMode=UDMode_None;
						MyFlagClear(SystemState.Task,SystemTask_DownIsland);//清除下岛任务标志位
					}
					if(MyFlagGet(SystemState.Task,SystemTask_UpIsland))//如果在上岛 取消上岛任务 并将系统上下岛状态清除
					{
						if(SystemState.TaskHandle.hTask_UpIsland)
							vTaskDelete(SystemState.TaskHandle.hTask_UpIsland);//退出任务
						SystemState.TaskHandle.hTask_UpIsland=NULL;
						SystemState.UDMode=UDMode_None;
						MyFlagClear(SystemState.Task,SystemTask_UpIsland);//清除上岛任务标志位
					}
				}
				else if(MyFlagGet(SystemState.Task,SystemTask_GetBullet))//如果在取蛋
				{
					if(SystemState.TaskHandle.hTask_getbullet)
						vTaskDelete(SystemState.TaskHandle.hTask_getbullet);//退出任务
					SystemState.TaskHandle.hTask_getbullet=NULL;
					MyFlagClear(SystemState.Task,SystemTask_GetBullet);//取蛋标志位
				}
				else if(MyFlagGet(SystemState.Task,SystemTask_AutoMove))
				{
					if(SystemState.TaskHandle.hTask_automove)
						vTaskDelete(SystemState.TaskHandle.hTask_automove);//退出任务
					SystemState.TaskHandle.hTask_automove=NULL;
					MyFlagClear(SystemState.Task,SystemTask_AutoMove);//自动移动标志位
				}
				else
				{
					SystemState.Robot.GetBulletFrame=0;
					SystemState.Robot.Guide_Frame=0;
					SystemState.Robot.Catch=0;
					SystemState.Robot.ReliefFinger=0;
					SystemState.Robot.ReliefFrame=0;
					SystemState.Robot.ReliefFrame2=0;
					SystemState.Robot.Lift_A_Loc=5;
					SystemState.Robot.Lift_Z_Loc=10;
				}
			}
			else//单击Z
			{
				SystemState.Robot.GetBulletFrame=0;
			}
		}
		Key_Z_t=GetSystemTimer();
	}
	
	//=============================================================================== X 按键=======获取子弹
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_X)==Key_Fall)
	{
		if(!CTR_Key)
		{
			if(!MyFlagGet(SystemState.Task,SystemTask_GetBullet))
			{
				xTaskCreate( vTask_GetBullet, "GetBullet_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_getbullet );//启动获取任务
			}
		}
		else
		{
			if(!MyFlagGet(SystemState.Task,SystemTask_AutoMove))
			{
				xTaskCreate( vAutoMove_Task, "vAutoMove_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_automove );
			}
		}
	}
	//=============================================================================== C 按键=======放出子弹
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_C)==Key_Fall)
	{
		if(GetSystemTimer()-Key_C_t<300)//双击C
		{
			SystemState.Robot.Magazine=1000;//放出子弹
			SystemState.Robot.Guide_Frame=110;//子弹导流板旋转
		}
		else//单击C
		{
			SystemState.Robot.Magazine=0;//弹仓关闭
			SystemState.Robot.Guide_Frame=0;//子弹导流板旋转
		}
		
		Key_C_t=GetSystemTimer();
		
	}
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_C)==Key_Down || SystemState.Robot.Guide_Frame>50)
	{
		
		SystemState.Robot.Supplyer=1;//补给站通信机制
		if(Times_n%10==0)
			CAN_Send_Message(&hcan2, 0x300,"GCC\x00\x00\x00\x00\x00");//发送沟通补给站的命令
	}else
		SystemState.Robot.Supplyer=0;
	

	Times_n++;
	if(Times_n==100)
		Times_n=0;
}

//获取距离传感器数值
void GetDistMesure(unsigned char*s)
{
	SystemState.DistSensor.Dis_RF=s[0];
	SystemState.DistSensor.Dis_LF=s[1];
	SystemState.DistSensor.Dis_RB=s[2];
	SystemState.DistSensor.Dis_LB=s[3];
	SystemState.DistSensor.Dis_X1=s[4];
	SystemState.DistSensor.Dis_X2=s[5];
}

//======================================
//									串口处理函数
//======================================
void UARTAnalys(UART_HandleTypeDef *huart,char* s,int len)
{
	if(huart==&huart3)
	{
		int mem_n=0,mem_min=0;
		if(IsStrInc(s,"GetMemoState"))
		{
			mem_min=xPortGetMinimumEverFreeHeapSize();
			mem_n=xPortGetFreeHeapSize();
			sprintf(s,"Memo remain: %d Min %d\r\n",mem_n,mem_min);
			My_print(s);
			return;
		}
		if(Camera_UART_Receive_Buff((unsigned char*)s))
		{
			if(mid_pose.X)
				SystemState.Sensor.Camera_X=mid_pose.X-380;
			else
				SystemState.Sensor.Camera_X=0;
			if(mid_pose.Y)
				SystemState.Sensor.Camera_Y=-(mid_pose.Y-340);
			else
				SystemState.Sensor.Camera_Y=0;
		}
	}
	if(huart==&huart6)
	{
		LRF_Analy((unsigned char*)s);//激光解析
	}
}

