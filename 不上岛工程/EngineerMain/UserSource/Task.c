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
								SystemState.Robot.Pump=0;
							break;
							case 3:  //地盘控制
								SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
								SystemState.Mode=SystemMode_Remote_MoveControl;
							break;
							case 2:  //升降机构控制 夹合
								SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
								SystemState.Mode=SystemMode_Remote_LiftControl;
							
								SystemState.Robot.Catch=1000;//夹合
								SystemState.Robot.Pump=1000;
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
				SystemState.Robot.Lift_Z_Loc+= 0.01*RC_Data.ch2;//爪子提升

			break;
			
			case SystemMode_Remote_LiftControl:
				
				SystemState.Robot.Move_Y_SPD=0;
			  SystemState.Robot.Move_X_SPD=0;
			  SystemState.Robot.Move_Z_SPD=0;
				
				SystemState.Robot.Lift_Z_Loc+= 0.01*RC_Data.ch2;//爪子提升
				SystemState.Robot.Lift_X_Loc+= 0.01*RC_Data.ch4;//爪子前伸
				SystemState.Robot.Move_X_SPD = 4*RC_Data.ch3;//地盘横移
				SystemState.Robot.Lift_A_Loc+= 0.01*RC_Data.ch1;//弹箱旋转获取
			
			
			break;
			
			case SystemMode_KeyBoard_Control:
				switch(RC_Data.switch_left)//键盘模式下的摇杆控制
				{
					case 3: //左边拨中间
							if(RC_Data.ch4<-600)SystemState.Robot.Magazine=0;
							if(RC_Data.ch4>600)SystemState.Robot.Magazine=1000;//弹仓闸门打开
							if(RC_Data.ch3<-600)SystemState.Robot.Magazine_Shell=0;
							if(RC_Data.ch3>600)SystemState.Robot.Magazine_Shell=1000;//弹仓盖打开
							
							if(RC_Data.ch2<-600)SystemState.Robot.Electromagnet=0;
							if(RC_Data.ch2>600)SystemState.Robot.Electromagnet=1000;//电磁铁吸合
							if(RC_Data.ch2<-200)SystemState.Robot.ReliefFrame=0;
							if(RC_Data.ch2>200)SystemState.Robot.ReliefFrame=1000;//救援架伸出
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
		if(SystemState.Robot.Lift_Z_Loc>100)SystemState.Robot.Magazine_Shell=1000;//弹仓盖在升起的时候打开 落下的时候关闭
					else SystemState.Robot.Magazine_Shell=0;
		
		
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
	
	if(SystemState.Time<10000)
		vTaskDelayUntil(&xLastWakeTime,2000/ portTICK_RATE_MS);
	
	SystemState.Mode=SystemMode_Set_Zero;//设定设备处于校准状态
	SystemState.Enable|=SystemState_Enable_Lift;//使能提升机构

	MoveMotor_EN_ALLSET(Motor_En_Enable);//设置所有地盘电机使能
	LiftMotor_EN_ALLSET(Motor_En_Enable);//设置所有提升电机使能
	
	MotorA.Com.Spd=50;
	MotorB.Com.Spd=-50;
	MotorC.Com.Spd=-50;
	MotorD.Com.Spd=50;
	MotorE.Com.Spd=-50;
	MotorF.Com.Spd=-150;
	
	while(1)
	{
		for(num=MotorA_No;num<MotorTotal_No;num++)//判断堵转 失能电机并清零位置
		{
			if(GetMotorStall(Motors[num]->State.State))//判断堵转
			{
				Motors[num]->Com.En=Motor_En_Disable;//电机失能
				Motors[num]->Motor_output=0;//电机输出为0
			}
		}
		tem_flag=0;
		for(num=MotorA_No;num<MotorTotal_No;num++)//检测电机是否都失能
		{
			if(Motors[num]->Com.En)tem_flag++;
		}
		if(!tem_flag)break;//如果都失能 代表所有电机都运行到底端了  跳出归零运行阶段
		
		if(RC_Data.switch_right==2)break;//遥控器取消了归零
		
		vTaskDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);	
	}
	
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);//延时100ms 等待皮带回复自然长度
	
	for(num=MotorA_No;num<MotorTotal_No;num++)//使能所有电机
	{
		Motor_Zero(Motors[num]);//电机归零
		Motors[num]->Com.Spd=0;//电机速度控制为0
		Motors[num]->Com.Loc=0;//电机位置控制为0
		Motors[num]->SPD_PID.moter_pid.Ulimit = Motor_output_limit_UD;//增加输出限幅
		Motors[num]->Com.En=Motor_En_Enable;//电机使能
	}
	SystemState.Robot.Lift_Z_Loc=0;
	SystemState.Robot.Lift_X_Loc=0;
	SystemState.Robot.Lift_A_Loc=180;
	
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

//获取子弹
void vTask_GetBullet(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	SystemState.Robot.Catch=1000;//夹子闭合
	SystemState.Robot.Pump=1000;//打开气泵
	vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);//等待吸合
	
	//===============================首先提升110mm
	SystemState.Robot.Lift_Z_Loc+=130;
	if(SystemState.Robot.Lift_Z_Loc>Lift_First_Limite + Lift_Second_Limite)
		SystemState.Robot.Lift_Z_Loc=Lift_First_Limite + Lift_Second_Limite;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待完成
	{
		if(GetMotorLocNear(MotorA.State.State)&&
			GetMotorLocNear(MotorB.State.State)&&
			GetMotorLocNear(MotorC.State.State)&&
			GetMotorLocNear(MotorD.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	
	//===============================收回
	SystemState.Robot.Lift_X_Loc=0;//
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待完成
	{
		/*if(My_abs(MotorE.State.Loc)<180)
		SystemState.Robot.Lift_A_Loc=My_abs(MotorE.State.Loc)*0.56f+80.0f;*/
		
		if(GetMotorLocNear(MotorE.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	
	//===============================旋转弹箱
	SystemState.Robot.Lift_A_Loc=70;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待完成
	{
		if(GetMotorLocNear(MotorF.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	vTaskDelayUntil(&xLastWakeTime,2000/ portTICK_RATE_MS);//等待子弹落下

	//===============================推出
	SystemState.Robot.Lift_X_Loc=200;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待完成
	{
		if(My_abs(MotorE.State.Loc)<180)
		SystemState.Robot.Lift_A_Loc=My_abs(MotorE.State.Loc)*0.56f+80.0f;
		
		if(GetMotorLocNear(MotorE.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	//================================下降
	SystemState.Robot.Lift_Z_Loc-=70;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//等待完成
	{
		if(GetMotorLocNear(MotorA.State.State)&&
			GetMotorLocNear(MotorB.State.State)&&
			GetMotorLocNear(MotorC.State.State)&&
			GetMotorLocNear(MotorD.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	//===============================丢弃弹箱
	SystemState.Robot.Lift_A_Loc=180;//弹箱归位
	SystemState.Robot.Catch=0;//夹子打开
	SystemState.Robot.Pump=0;//关闭气泵
	vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);//等待弹箱掉下
	
	
	//===============================归位
	SystemState.Robot.Lift_X_Loc=0;//收回
	SystemState.Robot.Lift_Z_Loc+=70;
	SystemState.Robot.Lift_A_Loc=180;//弹箱归位
	vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	while(1)//等待完成
	{
		if(GetMotorLocNear(MotorA.State.State)&&
			GetMotorLocNear(MotorB.State.State)&&
			GetMotorLocNear(MotorC.State.State)&&
			GetMotorLocNear(MotorD.State.State)&&
			GetMotorLocNear(MotorE.State.State)&&
			GetMotorLocNear(MotorF.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	MyFlagClear(SystemState.Task,SystemTask_GetBullet);//清除获取弹箱标志位
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
	
	while(1)
	{

		//g_mem_min=xPortGetMinimumEverFreeHeapSize();
		//g_mem_n=xPortGetFreeHeapSize();
		
		vTaskDelayUntil(&xLastWakeTime,200/ portTICK_RATE_MS);
	}
  //vTaskDelete(NULL);//退出任务
}



//=====================================================================================
//                                 其他函数
//
//
//=====================================================================================
//限幅与命令传递 vRobotCtr中调用
void SetLimitAndTransToMotor()
{
		float fist_lift=0;
		float second_lift=0;
	//提升机构限幅
		if(SystemState.Robot.Lift_Z_Loc<0)SystemState.Robot.Lift_Z_Loc=0;if(SystemState.Robot.Lift_Z_Loc>Lift_First_Limite+Lift_Second_Limite)SystemState.Robot.Lift_Z_Loc=Lift_First_Limite+Lift_Second_Limite;
		if(SystemState.Robot.Lift_X_Loc<0)SystemState.Robot.Lift_X_Loc=0;if(SystemState.Robot.Lift_X_Loc>200)SystemState.Robot.Lift_X_Loc=200;
		if(SystemState.Robot.Lift_A_Loc<0)SystemState.Robot.Lift_A_Loc=0;if(SystemState.Robot.Lift_A_Loc>200)SystemState.Robot.Lift_A_Loc=200;
		
		SpeedDistribute();//速度分解并赋给电机轮
	
		fist_lift=SystemState.Robot.Lift_Z_Loc * Lift_First_Limite / ( Lift_First_Limite + Lift_Second_Limite );
		second_lift=SystemState.Robot.Lift_Z_Loc * Lift_Second_Limite / ( Lift_First_Limite + Lift_Second_Limite );
		
		MotorA.Com.Loc=-fist_lift;//一级抬升
		MotorB.Com.Loc=fist_lift;//一级抬升
		MotorC.Com.Loc=second_lift;//二级抬升
		MotorD.Com.Loc=-second_lift;//二级抬升
		
		MotorE.Com.Loc= SystemState.Robot.Lift_X_Loc;//抓取前伸
		MotorF.Com.Loc= SystemState.Robot.Lift_A_Loc;//弹箱旋转
			
		if(MotorA.Com.Loc>0)MotorA.Com.Loc=0;if(MotorA.Com.Loc<-Lift_First_Limite)MotorA.Com.Loc=-Lift_First_Limite;
		if(MotorB.Com.Loc<0)MotorB.Com.Loc=0;if(MotorB.Com.Loc> Lift_First_Limite)MotorB.Com.Loc= Lift_First_Limite;
		if(MotorC.Com.Loc<0)MotorC.Com.Loc=0;if(MotorC.Com.Loc> Lift_Second_Limite)MotorC.Com.Loc= Lift_Second_Limite;
		if(MotorD.Com.Loc>0)MotorD.Com.Loc=0;if(MotorD.Com.Loc<-Lift_Second_Limite)MotorD.Com.Loc=-Lift_Second_Limite;
		if(MotorE.Com.Loc<0)MotorE.Com.Loc=0;if(MotorE.Com.Loc>190)MotorE.Com.Loc=190;
		if(MotorF.Com.Loc>200)MotorF.Com.Loc=200;if(MotorF.Com.Loc<0)MotorF.Com.Loc=0;
}


short BoardKeyCode=0;//本次键盘码
short BoardKeyCode_Pre=0;//上次键盘码
char Mouse_L=0;//鼠标左键状态
char Mouse_L_Pre=0;//上次鼠标左键状态
char Mouse_R=0;//鼠标左键状态
char Mouse_R_Pre=0;//上次鼠标左键状态

#define MOVE_MaxSPD 2500 //速度最大值
#define Move_ACC 40 //最大加减速

float g_HeighGetBLoc=512;//高位加持位置
float g_LowGetBLoc=205;//地位加持位置
float g_LaserDist=0.44f;//激光对正位置

//键盘控制   再机器人控制中调用
void KeyBoardCtr()
{
	float SPD_k=1;//速度控制系数

	float Shift_Key=0;//Shift按下
	float CTR_Key=0;//CTR按下
	static float SPD_Max=MOVE_MaxSPD;//速度最大值
	static float SPD_W_Max=MOVE_MaxSPD;//速度最大值
	float SPD_ACC=Move_ACC;//加减速
	
	static float Key_Q_t=0;//上次按下Q的时间
	static float Key_E_t=0;//上次按下E的时间
	static float Key_C_t=0;//上次按下C的时间
	static float Key_V_t=0;//上次按下C的时间
	static float Key_Z_t=0;//上次按下C的时间
	static float Mouse_L_t=0;//上次按下鼠标左键的时间
	
	
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
	
	//=============================================================================== 鼠标
	SystemState.Robot.Move_Z_SPD=RC_Data.mouse.x*30;//鼠标横向
	
	if(DBUS_CheckButtonState(Mouse_L,Mouse_L_Pre,0x01)==Key_Fall){ //鼠标左键
		if(GetSystemTimer()-Mouse_L_t<300)//双击鼠标左键
		{
				SystemState.Robot.ReliefFrame=1000;//救援架释放
		}
		else
		{
				SystemState.Robot.ReliefFrame=0;//救援架释放
		}
		Mouse_L_t=GetSystemTimer();//单击鼠标L键
	}
	
	
	if(RC_Data.mouse.press_right||RC_Data.switch_left==2)//按下鼠标右键
	{
		SPD_k=0;//键盘失能 不用还原成1
		SystemState.Robot.Camara_LED=1000;//补光
		
		
		SystemState.Robot.Move_X_SPD=SystemState.Sensor.Camera_X;
		SystemState.Robot.Lift_Z_Loc+=SystemState.Sensor.Camera_Y*0.02f;
		goto CTR_Other;
	}
	else
	{
		SystemState.Robot.Camara_LED=0;//补光
	}

	
	//=============================================================================== W 按键======
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_W)==Key_Down){
		if(Shift_Key)
		{
			if(SystemState.Robot.Lift_Z_Loc<100)
			{
				SPD_W_Max=MOVE_MaxSPD*1.5;
				SPD_ACC=Move_ACC*2;
			}
			SystemState.Robot.Move_Y_SPD+=SPD_ACC;
		}
		else if(CTR_Key)
		{
			if(SystemState.Robot.Lift_Z_Loc>100)
				SystemState.Robot.Move_Y_SPD=300;
			else
				SystemState.Robot.Move_Y_SPD=800;
		}
		else
		{
			SystemState.Robot.Move_Y_SPD+=SPD_ACC;
		}
			
		if(SystemState.Robot.Move_Y_SPD>SPD_W_Max)SystemState.Robot.Move_Y_SPD=SPD_W_Max;
		if(SystemState.Robot.Move_Y_SPD<0) SystemState.Robot.Move_Y_SPD=0;
	}
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_W)==Key_Raise){
		SPD_W_Max=MOVE_MaxSPD;
		SPD_ACC=Move_ACC;
	}
	//=============================================================================== S 按键======
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_S)==Key_Down){
		if(CTR_Key)
		{
			SystemState.Robot.Move_Y_SPD=-500;
		}
		else
		{
			SystemState.Robot.Move_Y_SPD-=SPD_ACC;
		}

		if(SystemState.Robot.Move_Y_SPD<(-SPD_Max))SystemState.Robot.Move_Y_SPD=-SPD_Max;
		if(SystemState.Robot.Move_Y_SPD>0) SystemState.Robot.Move_Y_SPD=0;
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
			SystemState.Robot.Move_X_SPD=-250;
		}
		else
		{
			SystemState.Robot.Move_X_SPD-=SPD_ACC;
		}
		if(SystemState.Robot.Move_X_SPD<(-SPD_Max))SystemState.Robot.Move_X_SPD=-SPD_Max;
		if(SystemState.Robot.Move_X_SPD>0) SystemState.Robot.Move_X_SPD=0;

		//SystemState.Robot.Lift_X_Loc+=2*Loc_k;
	}
		
	//=============================================================================== D 按键======
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_D)==Key_Down){
		if(CTR_Key)
		{
			SystemState.Robot.Move_X_SPD=250;
		}
		else
		{
			SystemState.Robot.Move_X_SPD+=SPD_ACC;
		}
		if(SystemState.Robot.Move_X_SPD > SPD_Max)SystemState.Robot.Move_X_SPD= SPD_Max;
		if(SystemState.Robot.Move_X_SPD<0) SystemState.Robot.Move_X_SPD=0;
		
		//SystemState.Robot.Lift_X_Loc-=2*Loc_k;
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
	
CTR_Other:	
	//=============================================================================== R 按键====== 提升
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_R)==Key_Down){
		if(!CTR_Key)
			SystemState.Robot.Lift_Z_Loc+=4;
		else
			SystemState.Robot.Lift_X_Loc=200;
	}
	//=============================================================================== F 按键====== 下降
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_F)==Key_Down){
		if(!CTR_Key)
			SystemState.Robot.Lift_Z_Loc-=1.5f;
		else
			SystemState.Robot.Lift_X_Loc=0;
	}
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_F)==Key_Fall){
		if(Shift_Key)
		{
			//当夹子在高位加持附近 或者在低位加持附近时 可以使用快速下降
			if((SystemState.Robot.Lift_Z_Loc > g_HeighGetBLoc-130)||
				((SystemState.Robot.Lift_Z_Loc > g_LowGetBLoc-130)&&(SystemState.Robot.Lift_Z_Loc < g_LowGetBLoc+20)))
				SystemState.Robot.Lift_Z_Loc-=30.0f;
		}
			
	}
	//=============================================================================== Q 按键====== 旋转弹箱
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_Q)==Key_Fall){
		if(CTR_Key)
		{
			if(GetSystemTimer()-Key_Q_t<300)//双击Q
			{
				SystemState.Robot.Lift_A_Loc=80;//倾倒
			}
			else
			{
				SystemState.Robot.Lift_A_Loc=180;//归位
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
				SystemState.Robot.Pump=1000;//抓取
			}
			else
			{
				SystemState.Robot.Catch=0;//放松
				SystemState.Robot.Pump=0;//放松
			}
		}
		Key_E_t=GetSystemTimer();
	}
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_E)==Key_Down)
	{
		if(!CTR_Key)
			SystemState.Robot.Move_Z_SPD=2500;//顺时针旋转
	}
	//=============================================================================== Z 按键=======抓取位置1
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_Z)==Key_Fall){
		if(GetSystemTimer()-Key_Z_t<300 && (!CTR_Key))//双击Z
		{
			SystemState.Robot.Lift_Z_Loc=g_HeighGetBLoc;
			SystemState.Robot.Lift_X_Loc=180;
			SystemState.Robot.Lift_A_Loc=180;
			SystemState.Robot.Catch=0;
			SystemState.Robot.Pump=0;
		}
		else
		{
			if(!CTR_Key)
			{
				if(SystemState.Robot.Lift_X_Loc<20)
				{
					SystemState.Robot.Lift_Z_Loc=g_LowGetBLoc;
					SystemState.Robot.Lift_X_Loc=190;
					SystemState.Robot.Lift_A_Loc=180;
					SystemState.Robot.Catch=0;
					SystemState.Robot.Pump=0;
				}
			}
			else
			{
				if(!SystemState.Robot.Magazine)//如果弹仓打开
				{
					if(SystemState.Robot.Lift_X_Loc!=0)
					{
						SystemState.Robot.Lift_X_Loc=0;
					}
					else
					{
						SystemState.Robot.Lift_Z_Loc=10;
						SystemState.Robot.Lift_A_Loc=180;
						SystemState.Robot.Catch=0;//夹子释放
						SystemState.Robot.Pump=0;//关闭泵
						SystemState.Robot.ReliefFrame=0;//收回救援架
						SystemState.Robot.Magazine=0;//弹仓关闭
					}
				}
				else
					SystemState.Robot.Magazine=0;//关闭弹仓
			}
		}
		Key_Z_t=GetSystemTimer();//单击Q键
	}
	
	//=============================================================================== X 按键=======获取子弹
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_X)==Key_Fall)
	{
		if(!MyFlagGet(SystemState.Task,SystemTask_GetBullet))
		{
			MyFlagSet(SystemState.Task,SystemTask_GetBullet);
			xTaskCreate( vTask_GetBullet, "GetBullet_Task", 200, NULL, 3, NULL );//启动获取任务
		}
	}
		
	//=============================================================================== C 按键=======放出子弹
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_C)==Key_Fall)
	{
		if(GetSystemTimer()-Key_C_t<300)//双击C
		{
			SystemState.Robot.Magazine=1000;//放出子弹
		}
		else//单击C
		{
			SystemState.Robot.Magazine=0;//弹仓关闭
		}
		
		Key_C_t=GetSystemTimer();
	}
	
	//=============================================================================== V 按键======= 加血卡
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_V)==Key_Fall)
	{
		if(GetSystemTimer()-Key_V_t<300)//双击V
		{
			SystemState.Robot.AddHPCard=1000;//加血卡释放
		}
		Key_V_t=GetSystemTimer();
	}
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_V)==Key_Up)
	{
		SystemState.Robot.AddHPCard=0;//加血卡复位
	}
	//=============================================================================== G 按键======= 激光测距
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_G)==Key_Down)
	{
		StartTOMeasure();//开始测量
		if(LaserRF.State==LRF_Mesure_OK)
		{
			if(SystemState.Robot.Move_Y_SPD<1000)//高速情况下不启动自动对正
			{
				SystemState.Robot.Move_X_SPD=(LaserRF.Dist-g_LaserDist)*(-2500);
				if(SystemState.Robot.Move_X_SPD> 500) SystemState.Robot.Move_X_SPD= 500;
				if(SystemState.Robot.Move_X_SPD<-500) SystemState.Robot.Move_X_SPD=-500;
			}
		}
	}
	else
	{
		StopMeasure();//停止测量
	}

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

