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


void vMotorGetZero_Task(void *pvParameters);//��������
void vTask_GetBullet(void *pvParameters);//��ȡ�ӵ�����

void KeyBoardCtr(void);//���̿���   �ٻ����˿����е���
void SetLimitAndTransToMotor(void);//�޷����������

void vTask_DownIsland(void *pvParameters);//�ϵ�����
void vTask_UpIsland(void *pvParameters);//�µ�����
void vAutoMove_Task(void *pvParameters);//�Զ��ƶ�����

//�����˿�������
void vRobotCtrTask(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		GetTaskPeriod(RobotCtr_Task_No);
	
		//======================================================�趨����ģʽ
		if(SystemState.Mode==SystemMode_Set_Zero)//�ж�ϵͳ�Ƿ��ڹ���״̬
		{
			switch(RC_Data.switch_right)
			{
				case 2:
					SystemState.Enable=0;//ʧ������
				  break;
				default:
					SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
					break;
			}
		}
		else
		{
			switch(RC_Data.switch_right)//��������
			{
				case 1://���Կ��� 
					SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
					SystemState.Mode=SystemMode_KeyBoard_Control;//���̿���
					break;
				case 3://ң��������
					switch(RC_Data.switch_left)
					{
							case 1://������������ �ɿ�
								SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
								SystemState.Mode=SystemMode_Remote_LiftControl;
								
								SystemState.Robot.Catch=0;

							break;
							case 3:  //���̿���
								SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
								SystemState.Mode=SystemMode_Remote_MoveControl;
							break;
							case 2:  //������������ �к�
								SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
								SystemState.Mode=SystemMode_Remote_LiftControl;
							
								SystemState.Robot.Catch=1000;//�к�

							break;
					}
					break;
					
				case 2:
					SystemState.Enable=0;//ʧ������
				  SystemState.Mode=SystemMode_Disable;//ʧ��ģʽ
					break;
			}
		}

		//=======================================================����ģʽ����
		switch(SystemState.Mode)//����ģʽ
		{
			case SystemMode_Remote_MoveControl:
				
				SystemState.Robot.Move_Y_SPD=4*RC_Data.ch4;//����ǰ��
			  SystemState.Robot.Move_X_SPD=4*RC_Data.ch3;//���̺���
			  SystemState.Robot.Move_Z_SPD=9.5*RC_Data.ch1;//������ת
				SystemState.Robot.Lift_Z_Loc+= 0.01*RC_Data.ch2;//��������

			break;
			
			case SystemMode_Remote_LiftControl:
				
				SystemState.Robot.Move_Y_SPD=0;
			  SystemState.Robot.Move_X_SPD=0;
			  SystemState.Robot.Move_Z_SPD=0;
				
				SystemState.Robot.Lift_Z_Loc+= 0.01*RC_Data.ch2;//��������
			
				SystemState.Robot.Lift_Y_SPD = 2*RC_Data.ch4;//����ǰ��
				SystemState.Robot.Lift_Y_Loc += 0.01*RC_Data.ch4;//����ǰ��
				SystemState.Robot.Move_Y_SPD=2*RC_Data.ch4;//����ǰ��
			
				SystemState.Robot.Move_X_SPD = 4*RC_Data.ch3;//���̺���
				SystemState.Robot.Lift_A_Loc+= 0.01*RC_Data.ch1;//������ת��ȡ
			
			break;
			
			case SystemMode_KeyBoard_Control:
				switch(RC_Data.switch_left)//����ģʽ�µ�ҡ�˿���
				{
					case 3: //��߲��м�
							if(RC_Data.ch4>600)//�ϵ�����
							{
								if(!MyFlagGet(SystemState.Task,SystemTask_UpIsland))
								{
									xTaskCreate( vTask_UpIsland, "UpIsland_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_UpIsland);//�����ϵ�����
								}
							}
							if(RC_Data.ch4<-600)//�µ�����
							{
								if(!MyFlagGet(SystemState.Task,SystemTask_DownIsland))
								{
									xTaskCreate( vTask_DownIsland, "DownIsland_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_DownIsland);//�����µ�����
								}
							}
							
							if(RC_Data.ch3<-600)//��ȡ�ӵ�����
							{
								if(!MyFlagGet(SystemState.Task,SystemTask_GetBullet))
								{
									xTaskCreate( vTask_GetBullet, "GetBullet_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_getbullet );//������ȡ����
								}
							}
							
							if(RC_Data.ch3>600)//ȡ������
							{
								if(SystemState.TaskHandle.hTask_DownIsland!=NULL)
								{
									vTaskDelete(SystemState.TaskHandle.hTask_DownIsland);//�˳�����
									SystemState.TaskHandle.hTask_DownIsland=NULL;
									SystemState.UDMode=UDMode_None;
									MyFlagClear(SystemState.Task,SystemTask_DownIsland);//����ϵ������־λ
								}
								if(SystemState.TaskHandle.hTask_UpIsland!=NULL)
								{
									vTaskDelete(SystemState.TaskHandle.hTask_UpIsland);//�˳�����
									SystemState.TaskHandle.hTask_UpIsland=NULL;
									SystemState.UDMode=UDMode_None;
									MyFlagClear(SystemState.Task,SystemTask_UpIsland);//����ϵ������־λ
								}	
							}

						break;
				}
			
				KeyBoardCtr();//���̿���
				break;
			case SystemMode_Disable://����ʧ��
				if(RC_Data.ch1<-600 &&RC_Data.ch2<-600 &&RC_Data.ch3>600 &&RC_Data.ch4<-600)
				{
					SystemState.Mode=SystemMode_Set_Zero;
					while(RC_Data.switch_right==2)
					{
						vTaskDelay(100/ portTICK_RATE_MS);
					}
					xTaskCreate( vMotorGetZero_Task, "MotorZero_Task", 200, NULL, 3, NULL );//�������
				}
				break;
		}

		
		
		SetLimitAndTransToMotor();//������ݸ����
		vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	}
}

//==============================================================
//
//		�����������(�ǵø���ң����״̬ Ҫ��Ȼң�����Ὣ�ٶ�ֵ����)
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
		if((SystemState.OutLine_Flag&0x7ff)==0)//�ȴ�ң������������������
				break;
		if(RC_Data.switch_right==2)break;//ң����ȡ���˵ȴ�
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	while(RC_Data.switch_right==2)//�ȴ�ң����̧���Ҳఴ��
	{
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	if(SystemState.Time<10000)
		vTaskDelayUntil(&xLastWakeTime,2000/ portTICK_RATE_MS);
	
	SystemState.Mode=SystemMode_Set_Zero;//�趨�豸����У׼״̬
	SystemState.Enable|=SystemState_Enable_Lift;//ʹ����������

	MoveMotor_EN_ALLSET(Motor_En_Calibration);//�������е��̵����У׼
	YuntaiMotor_EN_ALLSET(Motor_En_Calibration);//����������̨�����У׼
	LiftMotor_EN_ALLSET(Motor_En_Enable);//���������������ʹ��
	
	SystemState.Robot.EleBreak=1000;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);

	//������ҪУ׼�ĵ��Ϊ�ٶȿ���
	MotorA.pid_control_type=MotorSPD_Ctr;
	MotorB.pid_control_type=MotorSPD_Ctr;
	MotorC.pid_control_type=MotorSPD_Ctr;
	MotorD.pid_control_type=MotorSPD_Ctr;
	
	//�趨�����ٶ�
	MotorA.Com.Spd=-100;
	MotorB.Com.Spd=100;
	MotorC.Com.Spd=100;
	MotorD.Com.Spd=-100;

	for(num=MotorA_No;num<MotorTotal_No;num++)//��С��ת��ֵ
	{
		Motors[num]->Protect.OutPut_Stall=Motors[num]->Protect.OutPut_Stall/2;
	}
	
	while(1)
	{
		for(num=MotorA_No;num<=MotorD_No;num++)//�ж϶�ת ʧ�ܵ��������λ��
		{
			if(GetMotorStall(Motors[num]->State.State))//�ж϶�ת
			{
				Motor_Zero(Motors[num]);//�������
				Motors[num]->Com.En=Motor_En_Calibration;//�����У׼
				Motors[num]->pid_control_type=MotorLOC_Ctr;//�ָ����Ϊλ�ÿ���
			}
		}
		tem_flag=0;
		for(num=MotorA_No;num<=MotorD_No;num++)//������Ƿ�У׼���
		{
			if(Motors[num]->Com.En!=Motor_En_Calibration)tem_flag++;
		}
		if(!tem_flag)break;//�������У׼ �����������н׶�
		
		if(RC_Data.switch_right==2)break;//ң����ȡ���˹���
		
		vTaskDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);	
	}

	for(num=MotorA_No;num<MotorTotal_No;num++)//�����ת�����ֵ
	{
		Motors[num]->Protect.OutPut_Stall=Motors[num]->Protect.OutPut_Stall*2;
	}
	
	MotorA.pid_control_type=MotorLOC_Ctr;
	MotorB.pid_control_type=MotorLOC_Ctr;
	MotorC.pid_control_type=MotorLOC_Ctr;
	MotorC.State.Loc=5;
	MotorD.pid_control_type=MotorLOC_Ctr;
	
	SystemState.Mode=SystemMode_Remote_LiftControl;//����ϵͳ״̬Ϊң��������ģʽ

	My_print(">>MotorGetZero....OK\r\n");
	
	vTaskDelete(NULL);//�˳�����
}

//LED��˸
void vLED_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int Delay_t=500;
	while(1)
	{
		GetTaskPeriod(LED_Task_No);//��ȡ���м��
		
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

//LED2��˸
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
//�ⲿLED
void vSW2812_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	char s[100]={0};
	while(1)
	{
		memset(s,0,100);
		if(MyFlagGet(SystemState.Task,SystemTask_UpIsland))//�ϵ�����
		{
			s[0]='<';s[1]=7;s[2]='>';
			memcpy(s+3,"\xff\xff\xff\x01\x11\x0a\x0d",7);
			UART_SendToQueue(&huart4,s);
		}
		else if(MyFlagGet(SystemState.Task,SystemTask_DownIsland))//�µ�����
		{
			s[0]='<';s[1]=7;s[2]='>';
			memcpy(s+3,"\xff\xff\xff\x01\x21\x0a\x0d",7);
			UART_SendToQueue(&huart4,s);
		}
		else if(MyFlagGet(SystemState.Task,SystemTask_GetBullet))//ȡ������
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

//����������
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
		GetTaskPeriod(Beep_Task_No);//��ȡ���м��
		
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
//		���µ����ȡ�ӵ�����    ���µ������ٶȿ�����Ҫ�ж�ռ��
//
//
//===============================================================
//��ȡ�ӵ�
#define GetBullet_degree 200
void vTask_GetBullet(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	MyFlagSet(SystemState.Task,SystemTask_GetBullet);//��־��ȡ��ҩ���־λ
	
	MotorC.Com.En=Motor_En_Calibration;//ʹ�ܵ��
	//=======================================================
	SystemState.Robot.GetBulletFrame=1000;//������
	SystemState.Robot.Catch=0;//���Ӵ�
	SystemState.Robot.Lift_A_Loc=GetBullet_degree;//��ת��GetBullet_degree
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ���ת������
	{
		if(GetMotorLocNear(MotorC.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	SystemState.Robot.Catch=1000;//���ӱպ�
	vTaskDelayUntil(&xLastWakeTime,300/ portTICK_RATE_MS);
	
	//=======================================================
	SystemState.Robot.Lift_A_Loc=10;//��ת��30
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ���ת������
	{
		if(GetMotorLocNear(MotorC.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
	
	//=======================================================
	SystemState.Robot.Lift_A_Loc=185;//��ת��GetBullet_degree
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	//MotorC.LOC_PID.moter_pid.Ulimit+=200;
	
	while(1)//�ȴ���ת������
	{
		if(GetMotorLocNear(MotorC.State.State))
			break;
		
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	//MotorC.LOC_PID.moter_pid.Ulimit-=200;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	//=======================================================
	SystemState.Robot.Catch=0;//���Ӵ�
	SystemState.Robot.Lift_A_Loc=5;//��ת��5(��λ)
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ���ת������
	{
		if(GetMotorLocNear(MotorC.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	MotorC.Com.En=Motor_En_Disable;//ʧ�ܵ��
	vTaskDelayUntil(&xLastWakeTime,200/ portTICK_RATE_MS);
	MotorC.Com.En=Motor_En_Calibration;//ʹ�ܵ��
	
	SystemState.Robot.GetBulletFrame=0;//���������
	SystemState.GetBulletFinish=1;
	MyFlagClear(SystemState.Task,SystemTask_GetBullet);//�����ȡ�����־λ
	SystemState.TaskHandle.hTask_getbullet=NULL;//�����ȡ�ӵ�������
	vTaskDelete(NULL);//�˳�����
}

//�ϵ�����(��Ҫ��ռ��������Ȩ)
void vTask_UpIsland(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	MyFlagSet(SystemState.Task,SystemTask_UpIsland);
	SystemState.UDMode=UDMode_UpIsland;
	My_print("UpIsland Task Start....\r\n");
	
	//����������߶�
	SystemState.Robot.Lift_Z_Loc=305;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ���������������
	{
		if(GetMotorLocNear(MotorA.State.State)
			&&GetMotorLocNear(MotorB.State.State))
			break;
		if(MotorA.State.Loc>285 || MotorB.State.Loc>285)break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	//ǰ������˴�������⵽λ��
	Motor5.pid_control_type=MotorSPD_Ctr;//С�ֵ������Ϊ�ٶȿ���
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
			Motor_Zero(&Motor5);//�������
			Motor_Zero(&Motor6);//�������
			break;
		}
		vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	}	
	
	//���	
	SystemState.Robot.Lift_Z_Loc=40;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	
	while(1)//�ȴ���������������
	{
		if(GetMotorLocNear(MotorA.State.State)
			&&GetMotorLocNear(MotorB.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	MyFlagClear(SystemState.Task,SystemTask_UpIsland);//����ϵ������־λ
	SystemState.UDMode=UDMode_None;//������µ�ģʽ
	SystemState.TaskHandle.hTask_UpIsland=NULL;//����ϵ�������
	My_print("---UpIsland Task Finish.\r\n");
	SystemState.GetBulletFinish=0;//ȡ����ɱ�־����
	vTaskDelete(NULL);//�˳�����
}

//�µ�
void vTask_DownIsland(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	SystemState.UDMode=UDMode_DownIsland;
	MyFlagSet(SystemState.Task,SystemTask_DownIsland);
	My_print("DownIsland Task Start....\r\n");
	//����λ��
	while(1)
	{
		if(SystemState.DistSensor.Dis_RB<20 && SystemState.DistSensor.Dis_LB>20)//����ȥ��
		{
			SystemState.Robot.Move_Y_SPD=0;
			SystemState.Robot.Move_Z_SPD=+300;
		}
		if(SystemState.DistSensor.Dis_RB>20 && SystemState.DistSensor.Dis_LB<20)//�Һ��ȥ��
		{
			SystemState.Robot.Move_Y_SPD=0;
			SystemState.Robot.Move_Z_SPD=-300;
		}
		if(SystemState.DistSensor.Dis_RB<20 && SystemState.DistSensor.Dis_LB<20)//���˴�������������
		{
			SystemState.Robot.Move_Y_SPD=-100;
			SystemState.Robot.Move_Z_SPD=0;
		}
		if(SystemState.DistSensor.Dis_RB>20 && SystemState.DistSensor.Dis_LB>20)//���˴������������
		{
			SystemState.Robot.Move_Y_SPD=0;
			SystemState.Robot.Move_Z_SPD=0;
			break;
		}
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	//����������߶�
	SystemState.Robot.Lift_Z_Loc=300;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ���������������
	{
		if(GetMotorLocNear(MotorA.State.State)
			&&GetMotorLocNear(MotorB.State.State))
			break;
		if(MotorA.State.Loc>285 || MotorB.State.Loc>285)
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	//���˵�ǰ�˴�������⵽λ��
	SystemState.Robot.Move_Y_SPD=-250;
	SystemState.Robot.Lift_Y_SPD=-150;
	while(1)
	{
		if(SystemState.DistSensor.Dis_RF>20 && SystemState.DistSensor.Dis_LF>20)
		{
			SystemState.Robot.Move_Y_SPD=0;
			SystemState.Robot.Lift_Y_Loc=0;
			SystemState.Robot.Lift_Y_SPD=0;
			Motor_Zero(&Motor5);//�������
			Motor_Zero(&Motor6);//�������
			break;
		}
		vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	}
	Motor5.pid_control_type=MotorLOC_Ctr;//С�ֵ������Ϊλ�ÿ���
	Motor6.pid_control_type=MotorLOC_Ctr;
	
	SystemState.Robot.Lift_Y_Loc=-20;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ�С��ת��λ��
	{
		if(GetMotorLocNear(Motor5.State.State)
			&&GetMotorLocNear(Motor6.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	SystemState.Robot.Lift_Y_Loc=0;
	Motor_Zero(&Motor5);//�������
	Motor_Zero(&Motor6);//�������
	
	Motor5.pid_control_type=MotorSPD_Ctr;//С�ֵ������Ϊ�ٶȿ���
	Motor6.pid_control_type=MotorSPD_Ctr;
	//���	
	SystemState.Robot.Lift_Z_Loc=10;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ���������������
	{
		if(GetMotorLocNear(MotorA.State.State)
			&&GetMotorLocNear(MotorB.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
	
	MyFlagClear(SystemState.Task,SystemTask_DownIsland);//����µ������־λ
	SystemState.UDMode=UDMode_None;//������µ�ģʽ
	SystemState.TaskHandle.hTask_DownIsland=NULL;//�����ȡ�ӵ�������
	My_print("---DownIsland Task Finish.\r\n");
	SystemState.GetBulletFinish=0;//ȡ����ɱ�־����
	vTaskDelete(NULL);//�˳�����
}

#define AutoMove_Dis 325
//�Զ��ƶ�����
void vAutoMove_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	float M1_Loc=Motor1.State.Loc;
	
	int* dir=(int*)pvParameters;
	MyFlagSet(SystemState.Task,SystemTask_AutoMove);//�����Զ��ƶ������־
	
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
	
	MyFlagClear(SystemState.Task,SystemTask_AutoMove);//����Զ��ƶ������־
	SystemState.TaskHandle.hTask_automove=NULL;
	
	SystemState.GetBulletFinish=0;//ȡ����ɱ�־����
	vTaskDelete(NULL);//�˳�����
}

//�Զ���׼����  (û������) ��Ҫ�޸Ĵ���
void vAutoAim_Task(void *pvParameters)
{
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	char s[20]={0};
	while(1)
	{
		GetTaskPeriod(Test_Task_No);//��ȡ���м��
		
		memset(s,0,20);
		Camera_UART_Send_Buff((unsigned char*)s,CALL_Command,0xffffffff);
		if(RC_Data.mouse.press_right||(RC_Data.switch_right==1 &&RC_Data.switch_left==2))
			My_print(s);
		vTaskDelayUntil(&xLastWakeTime,30/ portTICK_RATE_MS);

	}
}

//================================================================
//****************************************************************
//****************************���Ź�****************************
//****************************************************************
//=================================================================

void vFEEDDOG_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	vTaskDelayUntil(&xLastWakeTime,2000/ portTICK_RATE_MS);
	__HAL_IWDG_START(&hiwdg);//�������Ź�
	
	while(1)
	{
		//__HAL_IWDG_RELOAD_COUNTER(&hiwdg);//���¿��Ź�
		
		if(MyFlagGet(SystemState.OutLine_Flag,RemoteCtr_No+MotorTotal_No))//���ң��������
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
//****************************��������****************************
//****************************************************************
//=================================================================
int g_mem_min=0;//��С�ڴ�ʣ��
int g_mem_n=0;//��ǰ�ڴ�ʣ��
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
  //vTaskDelete(NULL);//�˳�����
}



//=====================================================================================
//                                 ��������
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
			s[3*i+0]=0;//����ɫ����
			s[3*i+1]=100;//����ɫ����
		}
	}
	
	for(i=Motor5_No;i<MotorTotal_No;i++)
	{
		if(MyFlagGet(SystemState.OutLine_Flag,i))
		{
			s[3*(i-Motor5_No)+0]=0;//����ɫ����
			s[3*(i-Motor5_No)+2]=100;//����ɫ����
		}
	}
	
	if(MyFlagGet(SystemState.OutLine_Flag,RemoteCtr_No+MotorTotal_No))//ң����δ����
	{
		s[21]=0;s[23]=100;
	}
	
	if(MyFlagGet(SystemState.OutLine_Flag,DisMesur_No+MotorTotal_No))//����
	{
		s[18]=0;s[20]=100;
	}
		
	if(MyFlagGet(SystemState.OutLine_Flag,YuntaiCtr_No+MotorTotal_No))//��̨���ư�
	{
		s[15]=0;s[17]=100;
	}
}


//�޷�������� vRobotCtr�е���
void SetLimitAndTransToMotor()
{
	//���������޷�
		if(SystemState.Robot.Lift_Z_Loc<10)SystemState.Robot.Lift_Z_Loc=10;if(SystemState.Robot.Lift_Z_Loc>Lift_Limite)SystemState.Robot.Lift_Z_Loc=Lift_Limite;
		if(SystemState.Robot.Lift_A_Loc<0)SystemState.Robot.Lift_A_Loc=0;if(SystemState.Robot.Lift_A_Loc>200)SystemState.Robot.Lift_A_Loc=200;
		
		SpeedDistribute();//�ٶȷֽⲢ���������
	
				
		MotorA.Com.Loc=SystemState.Robot.Lift_Z_Loc;//̧��
		MotorB.Com.Loc=-SystemState.Robot.Lift_Z_Loc;//̧��

		MotorC.Com.Loc=-SystemState.Robot.Lift_A_Loc;//��ת
		MotorD.Com.Loc=	SystemState.Robot.Guide_Frame;//�����ܽǶ�
		
		Motor5.Com.Spd=-SystemState.Robot.Lift_Y_SPD;
		Motor6.Com.Spd= SystemState.Robot.Lift_Y_SPD;
		Motor5.Com.Loc=-SystemState.Robot.Lift_Y_Loc;//�ջ�ǰ������
		Motor6.Com.Loc= SystemState.Robot.Lift_Y_Loc;//�ջ�ǰ������
	
		if(MotorA.Com.Loc<0)MotorA.Com.Loc=0;if(MotorA.Com.Loc> Lift_Limite)MotorA.Com.Loc= Lift_Limite;
		if(MotorB.Com.Loc>0)MotorB.Com.Loc=0;if(MotorB.Com.Loc<-Lift_Limite)MotorB.Com.Loc=-Lift_Limite;
		if(MotorC.Com.Loc>0)MotorC.Com.Loc=0;if(MotorC.Com.Loc<-200)MotorC.Com.Loc=-200;
		if(MotorD.Com.Loc<0)MotorD.Com.Loc=0;if(MotorD.Com.Loc>135)MotorD.Com.Loc=135;
		
		if(SystemState.Robot.YunTai.Com_Pich>60)SystemState.Robot.YunTai.Com_Pich=60;
		if(SystemState.Robot.YunTai.Com_Pich<-70)SystemState.Robot.YunTai.Com_Pich=-70;
		
		if(SystemState.Robot.YunTai.Com_Pich>240)SystemState.Robot.YunTai.Com_Pich=240;
		if(SystemState.Robot.YunTai.Com_Pich<-180)SystemState.Robot.YunTai.Com_Pich=-180;
}


short BoardKeyCode=0;//���μ�����
short BoardKeyCode_Pre=0;//�ϴμ�����
char Mouse_L=0;//������״̬
char Mouse_L_Pre=0;//�ϴ�������״̬
char Mouse_R=0;//������״̬
char Mouse_R_Pre=0;//�ϴ�������״̬

#define MOVE_MaxSPD 2500 //�ٶ����ֵ
#define Move_ACC 40 //���Ӽ���


//���̿���   �ٻ����˿����е���
void KeyBoardCtr()
{
	float SPD_k=1;//�ٶȿ���ϵ��
	static int Times_n=0;
	float Shift_Key=0;//Shift����
	float CTR_Key=0;//CTR����
	short V_Key=0;//V����
	short G_Key=0;//G����
	static float SPD_Max=MOVE_MaxSPD;//�ٶ����ֵ
	static float SPD_W_Max=MOVE_MaxSPD;//�ٶ����ֵ
	float SPD_ACC=Move_ACC;//�Ӽ���
	
	static float Key_Q_t=0;//�ϴΰ���Q��ʱ��
	static float Key_E_t=0;//�ϴΰ���E��ʱ��
	static float Key_C_t=0;//�ϴΰ���C��ʱ��
	static float Key_V_t=0;//�ϴΰ���V��ʱ��
	static float Key_Z_t=0;//�ϴΰ���Z��ʱ��
	static float Key_R_t=0;//�ϴΰ���R��ʱ��
	static float Key_F_t=0;//�ϴΰ���F��ʱ��
	static float Mouse_L_t=0;//�ϴΰ�����������ʱ��
	static float Mouse_R_t=0;//�ϴΰ�������Ҽ���ʱ��
	
	//int AutoMoveDir=0;
	
	BoardKeyCode_Pre=BoardKeyCode;//�����ϴΰ���״̬��
	BoardKeyCode=RC_Data.keyBoard.key_code;//��ȡ��ǰң�ر���
	Mouse_L_Pre=Mouse_L;//�����ϴ�������ֵ
	Mouse_L=RC_Data.mouse.press_left;//��ȡ����������ֵ
	
	Mouse_R_Pre=Mouse_R;//�����ϴ�������ֵ
	Mouse_R=RC_Data.mouse.press_right;//��ȡ��������Ҽ�

	//=============================================================================== CTR ����======�����˶�
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_CTRL)==Key_Down)
			CTR_Key=1;

	//=============================================================================== SHIFT ����======����
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_SHIFT)==Key_Down)
	{
		Shift_Key=1;//Shift����
	}	
	//=============================================================================== V ����======= ����
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_V)==Key_Down)
	{
		V_Key=1;//V������
	}	
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_V)==Key_Fall)
	{
		if(GetSystemTimer()-Key_V_t<300)//˫��V
		{
			if(SystemState.Robot.YunTai.Direct==Forward)
				SystemState.Robot.YunTai.Direct=Backward;
			else
				SystemState.Robot.YunTai.Direct=Forward;
		}

		
		Key_V_t=GetSystemTimer();
	}
	
	//=============================================================================== G ����======= ������
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_G)==Key_Down)
	{
		G_Key=1;
	}
	
	//=============================================================================== ���
	
	if(SystemState.UDMode==UDMode_None &&	(!V_Key) )//�����µ����߰���V��ʱ�����������Ƴ���ת��
	{
		SystemState.Robot.Move_Z_SPD=RC_Data.mouse.x*30;//������
	}
	else
	{
		if(SystemState.UDMode==UDMode_UpIsland || V_Key)
			SystemState.Robot.Move_Z_SPD=0;
	}
	
	//==================== ��̨�Ƕ� ����
	if(V_Key)
	{
		SystemState.Robot.YunTai.Com_Yaw+=RC_Data.mouse.x*0.03;//��̨Yaw�Ƕ�
	}
	else
	{
		if(SystemState.Robot.YunTai.Direct==Forward)
			SystemState.Robot.YunTai.Com_Yaw=0;
		else
			SystemState.Robot.YunTai.Com_Yaw=-180;
	}
	SystemState.Robot.YunTai.Com_Pich-=RC_Data.mouse.y*0.02;//��̨Pich�Ƕ�
	
	
	if(DBUS_CheckButtonState(Mouse_L,Mouse_L_Pre,0x01)==Key_Fall){ //������
		if(GetSystemTimer()-Mouse_L_t<300)//˫��������
		{
				SystemState.Robot.ReliefFinger=1000;//��Ԯ���ͷ�
		}
		else
		{
				SystemState.Robot.ReliefFinger=0;//��Ԯ��
		}
		Mouse_L_t=GetSystemTimer();//�������L��
	}
	
	
	if(DBUS_CheckButtonState(Mouse_R,Mouse_R_Pre,0x01)==Key_Fall){ //����Ҽ�
		if(GetSystemTimer()-Mouse_R_t<300)//˫������Ҽ�
		{
			if(CTR_Key)
				SystemState.Robot.ReliefFrame=1000;//��Ԯ��ָ�ͷ�
			else
			{
				SystemState.Robot.ReliefFrame=1000;//��Ԯ��ָ�ͷ�
				SystemState.Robot.ReliefFrame2=1000;//��Ԯ��ָ�ͷ�(����)
			}
				
		}
		else
		{
				SystemState.Robot.ReliefFrame=0;//��Ԯ��ָ�ջ�
				SystemState.Robot.ReliefFrame2=0;//��Ԯ��ָ�ջ�
		}
		Mouse_R_t=GetSystemTimer();//�������R��
	}
	


	if(SystemState.UDMode!=UDMode_None || MyFlagGet(SystemState.Task,SystemTask_AutoMove))//��������µ�ģʽ �������Զ�����ģʽ ��Ҫ���ΰ���
		goto CTR_Other;
	//=============================================================================== W ����======
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_W)==Key_Down){//���W����
		if(Shift_Key)//���Shift����
		{
			if(SystemState.Robot.Lift_Z_Loc<100)//Shift��������
			{
				SPD_W_Max=MOVE_MaxSPD*1.5;//��������ٶ�
				SPD_ACC=Move_ACC*2;
			}
			SystemState.Robot.Move_Y_SPD+=SPD_ACC * SystemState.Robot.YunTai.Direct;
		}
		else if(G_Key)//G����   ������ֹ����
		{
			if(SystemState.Robot.YunTai.Direct==Forward)//�����ǰ������ǰ
			{
				if((SystemState.DistSensor.Dis_LF<20||SystemState.DistSensor.Dis_LF==255) //ǰ��������������ֹ����
					&& (SystemState.DistSensor.Dis_RF<20 || SystemState.DistSensor.Dis_RF==255))
				{
					SystemState.Robot.Move_Y_SPD=200 * SystemState.Robot.YunTai.Direct;
				}
				else
					SystemState.Robot.Move_Y_SPD=0;
			}else
			{
				if((SystemState.DistSensor.Dis_LB<20 || SystemState.DistSensor.Dis_LB==255)  //��������������ֹ����
					&& (SystemState.DistSensor.Dis_RB<20 || SystemState.DistSensor.Dis_RB==255))
				{
					SystemState.Robot.Move_Y_SPD=200 * SystemState.Robot.YunTai.Direct;
				}
				else
					SystemState.Robot.Move_Y_SPD=0;
			}
				
		}
		else if(CTR_Key)//CTR����
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
		
		//��ɲ��
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
	//=============================================================================== S ����======
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
		//��ɲ��
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
	//=============================================================================== WS ��δ����=
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_W)==Key_Up && 
		 DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_S)==Key_Up)
	{
		if(My_abs(SystemState.Robot.Move_Y_SPD)>SPD_ACC)
			SystemState.Robot.Move_Y_SPD=(My_abs(SystemState.Robot.Move_Y_SPD)-SPD_ACC*SPD_k)*IsPositive(SystemState.Robot.Move_Y_SPD);
		else
			SystemState.Robot.Move_Y_SPD=0;
	}
	//=============================================================================== A ����======
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
		
	//=============================================================================== D ����======
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

	//=============================================================================== AD ��δ����=
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_A)!=Key_Down && 
		 DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_D)!=Key_Down)
	{
		if(My_abs(SystemState.Robot.Move_X_SPD)>SPD_ACC)
			SystemState.Robot.Move_X_SPD=(My_abs(SystemState.Robot.Move_X_SPD)-SPD_ACC*SPD_k)*IsPositive(SystemState.Robot.Move_X_SPD);
		else
			SystemState.Robot.Move_X_SPD=0;
	}
	
	
	//=============================================================================== R ����====== ����
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_R)==Key_Down&&(!CTR_Key)){
			SystemState.Robot.Lift_Z_Loc+=3;
	}
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_R)==Key_Fall && CTR_Key)
	{
		if(GetSystemTimer()-Key_R_t<300)// CTR+˫��R
		{
			if(SystemState.Robot.YunTai.Direct==Forward)
			{
				if(!MyFlagGet(SystemState.Task,SystemTask_UpIsland))
				{
					xTaskCreate( vTask_UpIsland, "UpIsland_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_UpIsland);//�����ϵ�����
					goto CTR_Other;
				}		
			}
			if(SystemState.Robot.YunTai.Direct==Backward)
			{
				if(!MyFlagGet(SystemState.Task,SystemTask_DownIsland))
				{
					xTaskCreate( vTask_DownIsland, "DownIsland_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_DownIsland);//�����µ�����
					goto CTR_Other;
				}	
			}	
		}
		Key_R_t=GetSystemTimer();
	}
	//=============================================================================== F ����====== �½�
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_F)==Key_Down &&(!CTR_Key)){
			SystemState.Robot.Lift_Z_Loc-=3;
	}
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_F)==Key_Fall && CTR_Key)
	{
		if(GetSystemTimer()-Key_F_t<300)// CTR+˫��F
		{							
			if(!MyFlagGet(SystemState.Task,SystemTask_DownIsland))
			{
				xTaskCreate( vTask_DownIsland, "DownIsland_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_DownIsland);//�����µ�����
				goto CTR_Other;
			}
		}
		Key_F_t=GetSystemTimer();
	}
	//=============================================================================== Q ����====== ��ת����
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_Q)==Key_Fall){
		if(CTR_Key)
		{
			if(GetSystemTimer()-Key_Q_t<300)//˫��Q
			{
				SystemState.Robot.Lift_A_Loc=190;//�㵹
			}
			else
			{
				SystemState.Robot.Lift_A_Loc=0;//��λ
			}	
		}
		Key_Q_t=GetSystemTimer();
	}
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_Q)==Key_Down)
	{
		if(!CTR_Key)
			SystemState.Robot.Move_Z_SPD=-2500;//��ʱ����ת
	}
	//=============================================================================== E ����=======ץȡ
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_E)==Key_Fall){
		if(CTR_Key)
		{
			if(GetSystemTimer()-Key_E_t<300)//˫��E
			{
				SystemState.Robot.Catch=1000;//ץȡ
				
			}
			else
			{
				SystemState.Robot.Catch=0;//����
				
			}
		}
		Key_E_t=GetSystemTimer();
	}
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_E)==Key_Down)
	{
		if(!CTR_Key)
			SystemState.Robot.Move_Z_SPD=2500;//˳ʱ����ת
	}
	
CTR_Other:
	//=============================================================================== Z ����=======ץȡλ��1
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_Z)==Key_Fall){
		if(GetSystemTimer()-Key_Z_t<300 && (!CTR_Key))//˫��Z
		{
			SystemState.Robot.GetBulletFrame=1000;
		}
		else
		{
			if(CTR_Key)//CTR+Z �����ָ�
			{
				//����ȡ������
				if(SystemState.UDMode!=UDMode_None)
				{
					if(MyFlagGet(SystemState.Task,SystemTask_DownIsland))//������µ� ȡ���µ����� ����ϵͳ���µ�״̬���
					{
						if(SystemState.TaskHandle.hTask_DownIsland)
							vTaskDelete(SystemState.TaskHandle.hTask_DownIsland);//�˳�����
						SystemState.TaskHandle.hTask_DownIsland=NULL;
						SystemState.UDMode=UDMode_None;
						MyFlagClear(SystemState.Task,SystemTask_DownIsland);//����µ������־λ
					}
					if(MyFlagGet(SystemState.Task,SystemTask_UpIsland))//������ϵ� ȡ���ϵ����� ����ϵͳ���µ�״̬���
					{
						if(SystemState.TaskHandle.hTask_UpIsland)
							vTaskDelete(SystemState.TaskHandle.hTask_UpIsland);//�˳�����
						SystemState.TaskHandle.hTask_UpIsland=NULL;
						SystemState.UDMode=UDMode_None;
						MyFlagClear(SystemState.Task,SystemTask_UpIsland);//����ϵ������־λ
					}
				}
				else if(MyFlagGet(SystemState.Task,SystemTask_GetBullet))//�����ȡ��
				{
					if(SystemState.TaskHandle.hTask_getbullet)
						vTaskDelete(SystemState.TaskHandle.hTask_getbullet);//�˳�����
					SystemState.TaskHandle.hTask_getbullet=NULL;
					MyFlagClear(SystemState.Task,SystemTask_GetBullet);//ȡ����־λ
				}
				else if(MyFlagGet(SystemState.Task,SystemTask_AutoMove))
				{
					if(SystemState.TaskHandle.hTask_automove)
						vTaskDelete(SystemState.TaskHandle.hTask_automove);//�˳�����
					SystemState.TaskHandle.hTask_automove=NULL;
					MyFlagClear(SystemState.Task,SystemTask_AutoMove);//�Զ��ƶ���־λ
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
			else//����Z
			{
				SystemState.Robot.GetBulletFrame=0;
			}
		}
		Key_Z_t=GetSystemTimer();
	}
	
	//=============================================================================== X ����=======��ȡ�ӵ�
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_X)==Key_Fall)
	{
		if(!CTR_Key)
		{
			if(!MyFlagGet(SystemState.Task,SystemTask_GetBullet))
			{
				xTaskCreate( vTask_GetBullet, "GetBullet_Task", 200, NULL, 3, &SystemState.TaskHandle.hTask_getbullet );//������ȡ����
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
	//=============================================================================== C ����=======�ų��ӵ�
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_C)==Key_Fall)
	{
		if(GetSystemTimer()-Key_C_t<300)//˫��C
		{
			SystemState.Robot.Magazine=1000;//�ų��ӵ�
			SystemState.Robot.Guide_Frame=110;//�ӵ���������ת
		}
		else//����C
		{
			SystemState.Robot.Magazine=0;//���ֹر�
			SystemState.Robot.Guide_Frame=0;//�ӵ���������ת
		}
		
		Key_C_t=GetSystemTimer();
		
	}
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_C)==Key_Down || SystemState.Robot.Guide_Frame>50)
	{
		
		SystemState.Robot.Supplyer=1;//����վͨ�Ż���
		if(Times_n%10==0)
			CAN_Send_Message(&hcan2, 0x300,"GCC\x00\x00\x00\x00\x00");//���͹�ͨ����վ������
	}else
		SystemState.Robot.Supplyer=0;
	

	Times_n++;
	if(Times_n==100)
		Times_n=0;
}

//��ȡ���봫������ֵ
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
//									���ڴ�����
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
		LRF_Analy((unsigned char*)s);//�������
	}
}

