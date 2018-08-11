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
								SystemState.Robot.Pump=0;
							break;
							case 3:  //���̿���
								SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
								SystemState.Mode=SystemMode_Remote_MoveControl;
							break;
							case 2:  //������������ �к�
								SystemState.Enable=SystemState.Enable|SystemState_Enable_Move|SystemState_Enable_Lift;
								SystemState.Mode=SystemMode_Remote_LiftControl;
							
								SystemState.Robot.Catch=1000;//�к�
								SystemState.Robot.Pump=1000;
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
				SystemState.Robot.Lift_Z_Loc+= 0.01*RC_Data.ch2;//צ������

			break;
			
			case SystemMode_Remote_LiftControl:
				
				SystemState.Robot.Move_Y_SPD=0;
			  SystemState.Robot.Move_X_SPD=0;
			  SystemState.Robot.Move_Z_SPD=0;
				
				SystemState.Robot.Lift_Z_Loc+= 0.01*RC_Data.ch2;//צ������
				SystemState.Robot.Lift_X_Loc+= 0.01*RC_Data.ch4;//צ��ǰ��
				SystemState.Robot.Move_X_SPD = 4*RC_Data.ch3;//���̺���
				SystemState.Robot.Lift_A_Loc+= 0.01*RC_Data.ch1;//������ת��ȡ
			
			
			break;
			
			case SystemMode_KeyBoard_Control:
				switch(RC_Data.switch_left)//����ģʽ�µ�ҡ�˿���
				{
					case 3: //��߲��м�
							if(RC_Data.ch4<-600)SystemState.Robot.Magazine=0;
							if(RC_Data.ch4>600)SystemState.Robot.Magazine=1000;//����բ�Ŵ�
							if(RC_Data.ch3<-600)SystemState.Robot.Magazine_Shell=0;
							if(RC_Data.ch3>600)SystemState.Robot.Magazine_Shell=1000;//���ָǴ�
							
							if(RC_Data.ch2<-600)SystemState.Robot.Electromagnet=0;
							if(RC_Data.ch2>600)SystemState.Robot.Electromagnet=1000;//���������
							if(RC_Data.ch2<-200)SystemState.Robot.ReliefFrame=0;
							if(RC_Data.ch2>200)SystemState.Robot.ReliefFrame=1000;//��Ԯ�����
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
		if(SystemState.Robot.Lift_Z_Loc>100)SystemState.Robot.Magazine_Shell=1000;//���ָ��������ʱ��� ���µ�ʱ��ر�
					else SystemState.Robot.Magazine_Shell=0;
		
		
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
	
	if(SystemState.Time<10000)
		vTaskDelayUntil(&xLastWakeTime,2000/ portTICK_RATE_MS);
	
	SystemState.Mode=SystemMode_Set_Zero;//�趨�豸����У׼״̬
	SystemState.Enable|=SystemState_Enable_Lift;//ʹ����������

	MoveMotor_EN_ALLSET(Motor_En_Enable);//�������е��̵��ʹ��
	LiftMotor_EN_ALLSET(Motor_En_Enable);//���������������ʹ��
	
	MotorA.Com.Spd=50;
	MotorB.Com.Spd=-50;
	MotorC.Com.Spd=-50;
	MotorD.Com.Spd=50;
	MotorE.Com.Spd=-50;
	MotorF.Com.Spd=-150;
	
	while(1)
	{
		for(num=MotorA_No;num<MotorTotal_No;num++)//�ж϶�ת ʧ�ܵ��������λ��
		{
			if(GetMotorStall(Motors[num]->State.State))//�ж϶�ת
			{
				Motors[num]->Com.En=Motor_En_Disable;//���ʧ��
				Motors[num]->Motor_output=0;//������Ϊ0
			}
		}
		tem_flag=0;
		for(num=MotorA_No;num<MotorTotal_No;num++)//������Ƿ�ʧ��
		{
			if(Motors[num]->Com.En)tem_flag++;
		}
		if(!tem_flag)break;//�����ʧ�� �������е�������е��׶���  �����������н׶�
		
		if(RC_Data.switch_right==2)break;//ң����ȡ���˹���
		
		vTaskDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);	
	}
	
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);//��ʱ100ms �ȴ�Ƥ���ظ���Ȼ����
	
	for(num=MotorA_No;num<MotorTotal_No;num++)//ʹ�����е��
	{
		Motor_Zero(Motors[num]);//�������
		Motors[num]->Com.Spd=0;//����ٶȿ���Ϊ0
		Motors[num]->Com.Loc=0;//���λ�ÿ���Ϊ0
		Motors[num]->SPD_PID.moter_pid.Ulimit = Motor_output_limit_UD;//��������޷�
		Motors[num]->Com.En=Motor_En_Enable;//���ʹ��
	}
	SystemState.Robot.Lift_Z_Loc=0;
	SystemState.Robot.Lift_X_Loc=0;
	SystemState.Robot.Lift_A_Loc=180;
	
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

//��ȡ�ӵ�
void vTask_GetBullet(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	SystemState.Robot.Catch=1000;//���ӱպ�
	SystemState.Robot.Pump=1000;//������
	vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);//�ȴ�����
	
	//===============================��������110mm
	SystemState.Robot.Lift_Z_Loc+=130;
	if(SystemState.Robot.Lift_Z_Loc>Lift_First_Limite + Lift_Second_Limite)
		SystemState.Robot.Lift_Z_Loc=Lift_First_Limite + Lift_Second_Limite;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ����
	{
		if(GetMotorLocNear(MotorA.State.State)&&
			GetMotorLocNear(MotorB.State.State)&&
			GetMotorLocNear(MotorC.State.State)&&
			GetMotorLocNear(MotorD.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	
	//===============================�ջ�
	SystemState.Robot.Lift_X_Loc=0;//
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ����
	{
		/*if(My_abs(MotorE.State.Loc)<180)
		SystemState.Robot.Lift_A_Loc=My_abs(MotorE.State.Loc)*0.56f+80.0f;*/
		
		if(GetMotorLocNear(MotorE.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	
	//===============================��ת����
	SystemState.Robot.Lift_A_Loc=70;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ����
	{
		if(GetMotorLocNear(MotorF.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	vTaskDelayUntil(&xLastWakeTime,2000/ portTICK_RATE_MS);//�ȴ��ӵ�����

	//===============================�Ƴ�
	SystemState.Robot.Lift_X_Loc=200;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ����
	{
		if(My_abs(MotorE.State.Loc)<180)
		SystemState.Robot.Lift_A_Loc=My_abs(MotorE.State.Loc)*0.56f+80.0f;
		
		if(GetMotorLocNear(MotorE.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	//================================�½�
	SystemState.Robot.Lift_Z_Loc-=70;
	vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	while(1)//�ȴ����
	{
		if(GetMotorLocNear(MotorA.State.State)&&
			GetMotorLocNear(MotorB.State.State)&&
			GetMotorLocNear(MotorC.State.State)&&
			GetMotorLocNear(MotorD.State.State))
			break;
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}
	//===============================��������
	SystemState.Robot.Lift_A_Loc=180;//�����λ
	SystemState.Robot.Catch=0;//���Ӵ�
	SystemState.Robot.Pump=0;//�ر�����
	vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);//�ȴ��������
	
	
	//===============================��λ
	SystemState.Robot.Lift_X_Loc=0;//�ջ�
	SystemState.Robot.Lift_Z_Loc+=70;
	SystemState.Robot.Lift_A_Loc=180;//�����λ
	vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	while(1)//�ȴ����
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
	MyFlagClear(SystemState.Task,SystemTask_GetBullet);//�����ȡ�����־λ
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
	
	while(1)
	{

		//g_mem_min=xPortGetMinimumEverFreeHeapSize();
		//g_mem_n=xPortGetFreeHeapSize();
		
		vTaskDelayUntil(&xLastWakeTime,200/ portTICK_RATE_MS);
	}
  //vTaskDelete(NULL);//�˳�����
}



//=====================================================================================
//                                 ��������
//
//
//=====================================================================================
//�޷�������� vRobotCtr�е���
void SetLimitAndTransToMotor()
{
		float fist_lift=0;
		float second_lift=0;
	//���������޷�
		if(SystemState.Robot.Lift_Z_Loc<0)SystemState.Robot.Lift_Z_Loc=0;if(SystemState.Robot.Lift_Z_Loc>Lift_First_Limite+Lift_Second_Limite)SystemState.Robot.Lift_Z_Loc=Lift_First_Limite+Lift_Second_Limite;
		if(SystemState.Robot.Lift_X_Loc<0)SystemState.Robot.Lift_X_Loc=0;if(SystemState.Robot.Lift_X_Loc>200)SystemState.Robot.Lift_X_Loc=200;
		if(SystemState.Robot.Lift_A_Loc<0)SystemState.Robot.Lift_A_Loc=0;if(SystemState.Robot.Lift_A_Loc>200)SystemState.Robot.Lift_A_Loc=200;
		
		SpeedDistribute();//�ٶȷֽⲢ���������
	
		fist_lift=SystemState.Robot.Lift_Z_Loc * Lift_First_Limite / ( Lift_First_Limite + Lift_Second_Limite );
		second_lift=SystemState.Robot.Lift_Z_Loc * Lift_Second_Limite / ( Lift_First_Limite + Lift_Second_Limite );
		
		MotorA.Com.Loc=-fist_lift;//һ��̧��
		MotorB.Com.Loc=fist_lift;//һ��̧��
		MotorC.Com.Loc=second_lift;//����̧��
		MotorD.Com.Loc=-second_lift;//����̧��
		
		MotorE.Com.Loc= SystemState.Robot.Lift_X_Loc;//ץȡǰ��
		MotorF.Com.Loc= SystemState.Robot.Lift_A_Loc;//������ת
			
		if(MotorA.Com.Loc>0)MotorA.Com.Loc=0;if(MotorA.Com.Loc<-Lift_First_Limite)MotorA.Com.Loc=-Lift_First_Limite;
		if(MotorB.Com.Loc<0)MotorB.Com.Loc=0;if(MotorB.Com.Loc> Lift_First_Limite)MotorB.Com.Loc= Lift_First_Limite;
		if(MotorC.Com.Loc<0)MotorC.Com.Loc=0;if(MotorC.Com.Loc> Lift_Second_Limite)MotorC.Com.Loc= Lift_Second_Limite;
		if(MotorD.Com.Loc>0)MotorD.Com.Loc=0;if(MotorD.Com.Loc<-Lift_Second_Limite)MotorD.Com.Loc=-Lift_Second_Limite;
		if(MotorE.Com.Loc<0)MotorE.Com.Loc=0;if(MotorE.Com.Loc>190)MotorE.Com.Loc=190;
		if(MotorF.Com.Loc>200)MotorF.Com.Loc=200;if(MotorF.Com.Loc<0)MotorF.Com.Loc=0;
}


short BoardKeyCode=0;//���μ�����
short BoardKeyCode_Pre=0;//�ϴμ�����
char Mouse_L=0;//������״̬
char Mouse_L_Pre=0;//�ϴ�������״̬
char Mouse_R=0;//������״̬
char Mouse_R_Pre=0;//�ϴ�������״̬

#define MOVE_MaxSPD 2500 //�ٶ����ֵ
#define Move_ACC 40 //���Ӽ���

float g_HeighGetBLoc=512;//��λ�ӳ�λ��
float g_LowGetBLoc=205;//��λ�ӳ�λ��
float g_LaserDist=0.44f;//�������λ��

//���̿���   �ٻ����˿����е���
void KeyBoardCtr()
{
	float SPD_k=1;//�ٶȿ���ϵ��

	float Shift_Key=0;//Shift����
	float CTR_Key=0;//CTR����
	static float SPD_Max=MOVE_MaxSPD;//�ٶ����ֵ
	static float SPD_W_Max=MOVE_MaxSPD;//�ٶ����ֵ
	float SPD_ACC=Move_ACC;//�Ӽ���
	
	static float Key_Q_t=0;//�ϴΰ���Q��ʱ��
	static float Key_E_t=0;//�ϴΰ���E��ʱ��
	static float Key_C_t=0;//�ϴΰ���C��ʱ��
	static float Key_V_t=0;//�ϴΰ���C��ʱ��
	static float Key_Z_t=0;//�ϴΰ���C��ʱ��
	static float Mouse_L_t=0;//�ϴΰ�����������ʱ��
	
	
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
	
	//=============================================================================== ���
	SystemState.Robot.Move_Z_SPD=RC_Data.mouse.x*30;//������
	
	if(DBUS_CheckButtonState(Mouse_L,Mouse_L_Pre,0x01)==Key_Fall){ //������
		if(GetSystemTimer()-Mouse_L_t<300)//˫��������
		{
				SystemState.Robot.ReliefFrame=1000;//��Ԯ���ͷ�
		}
		else
		{
				SystemState.Robot.ReliefFrame=0;//��Ԯ���ͷ�
		}
		Mouse_L_t=GetSystemTimer();//�������L��
	}
	
	
	if(RC_Data.mouse.press_right||RC_Data.switch_left==2)//��������Ҽ�
	{
		SPD_k=0;//����ʧ�� ���û�ԭ��1
		SystemState.Robot.Camara_LED=1000;//����
		
		
		SystemState.Robot.Move_X_SPD=SystemState.Sensor.Camera_X;
		SystemState.Robot.Lift_Z_Loc+=SystemState.Sensor.Camera_Y*0.02f;
		goto CTR_Other;
	}
	else
	{
		SystemState.Robot.Camara_LED=0;//����
	}

	
	//=============================================================================== W ����======
	
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
	//=============================================================================== S ����======
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
		
	//=============================================================================== D ����======
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

	//=============================================================================== AD ��δ����=
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_A)!=Key_Down && 
		 DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_D)!=Key_Down)
	{
		if(My_abs(SystemState.Robot.Move_X_SPD)>SPD_ACC)
			SystemState.Robot.Move_X_SPD=(My_abs(SystemState.Robot.Move_X_SPD)-SPD_ACC*SPD_k)*IsPositive(SystemState.Robot.Move_X_SPD);
		else
			SystemState.Robot.Move_X_SPD=0;
	}
	
CTR_Other:	
	//=============================================================================== R ����====== ����
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_R)==Key_Down){
		if(!CTR_Key)
			SystemState.Robot.Lift_Z_Loc+=4;
		else
			SystemState.Robot.Lift_X_Loc=200;
	}
	//=============================================================================== F ����====== �½�
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_F)==Key_Down){
		if(!CTR_Key)
			SystemState.Robot.Lift_Z_Loc-=1.5f;
		else
			SystemState.Robot.Lift_X_Loc=0;
	}
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_F)==Key_Fall){
		if(Shift_Key)
		{
			//�������ڸ�λ�ӳָ��� �����ڵ�λ�ӳָ���ʱ ����ʹ�ÿ����½�
			if((SystemState.Robot.Lift_Z_Loc > g_HeighGetBLoc-130)||
				((SystemState.Robot.Lift_Z_Loc > g_LowGetBLoc-130)&&(SystemState.Robot.Lift_Z_Loc < g_LowGetBLoc+20)))
				SystemState.Robot.Lift_Z_Loc-=30.0f;
		}
			
	}
	//=============================================================================== Q ����====== ��ת����
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_Q)==Key_Fall){
		if(CTR_Key)
		{
			if(GetSystemTimer()-Key_Q_t<300)//˫��Q
			{
				SystemState.Robot.Lift_A_Loc=80;//�㵹
			}
			else
			{
				SystemState.Robot.Lift_A_Loc=180;//��λ
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
				SystemState.Robot.Pump=1000;//ץȡ
			}
			else
			{
				SystemState.Robot.Catch=0;//����
				SystemState.Robot.Pump=0;//����
			}
		}
		Key_E_t=GetSystemTimer();
	}
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_E)==Key_Down)
	{
		if(!CTR_Key)
			SystemState.Robot.Move_Z_SPD=2500;//˳ʱ����ת
	}
	//=============================================================================== Z ����=======ץȡλ��1
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_Z)==Key_Fall){
		if(GetSystemTimer()-Key_Z_t<300 && (!CTR_Key))//˫��Z
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
				if(!SystemState.Robot.Magazine)//������ִ�
				{
					if(SystemState.Robot.Lift_X_Loc!=0)
					{
						SystemState.Robot.Lift_X_Loc=0;
					}
					else
					{
						SystemState.Robot.Lift_Z_Loc=10;
						SystemState.Robot.Lift_A_Loc=180;
						SystemState.Robot.Catch=0;//�����ͷ�
						SystemState.Robot.Pump=0;//�رձ�
						SystemState.Robot.ReliefFrame=0;//�ջؾ�Ԯ��
						SystemState.Robot.Magazine=0;//���ֹر�
					}
				}
				else
					SystemState.Robot.Magazine=0;//�رյ���
			}
		}
		Key_Z_t=GetSystemTimer();//����Q��
	}
	
	//=============================================================================== X ����=======��ȡ�ӵ�
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_X)==Key_Fall)
	{
		if(!MyFlagGet(SystemState.Task,SystemTask_GetBullet))
		{
			MyFlagSet(SystemState.Task,SystemTask_GetBullet);
			xTaskCreate( vTask_GetBullet, "GetBullet_Task", 200, NULL, 3, NULL );//������ȡ����
		}
	}
		
	//=============================================================================== C ����=======�ų��ӵ�
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_C)==Key_Fall)
	{
		if(GetSystemTimer()-Key_C_t<300)//˫��C
		{
			SystemState.Robot.Magazine=1000;//�ų��ӵ�
		}
		else//����C
		{
			SystemState.Robot.Magazine=0;//���ֹر�
		}
		
		Key_C_t=GetSystemTimer();
	}
	
	//=============================================================================== V ����======= ��Ѫ��
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_V)==Key_Fall)
	{
		if(GetSystemTimer()-Key_V_t<300)//˫��V
		{
			SystemState.Robot.AddHPCard=1000;//��Ѫ���ͷ�
		}
		Key_V_t=GetSystemTimer();
	}
	
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_V)==Key_Up)
	{
		SystemState.Robot.AddHPCard=0;//��Ѫ����λ
	}
	//=============================================================================== G ����======= ������
	if(DBUS_CheckButtonState(BoardKeyCode,BoardKeyCode_Pre,KEY_G)==Key_Down)
	{
		StartTOMeasure();//��ʼ����
		if(LaserRF.State==LRF_Mesure_OK)
		{
			if(SystemState.Robot.Move_Y_SPD<1000)//��������²������Զ�����
			{
				SystemState.Robot.Move_X_SPD=(LaserRF.Dist-g_LaserDist)*(-2500);
				if(SystemState.Robot.Move_X_SPD> 500) SystemState.Robot.Move_X_SPD= 500;
				if(SystemState.Robot.Move_X_SPD<-500) SystemState.Robot.Move_X_SPD=-500;
			}
		}
	}
	else
	{
		StopMeasure();//ֹͣ����
	}

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

