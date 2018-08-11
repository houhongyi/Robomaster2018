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

void CTR_CruiseMode(void);//Ѳ��ģʽ����
void CTR_AttackMode(float attack_Loc);//����ģʽ����
void CTR_FoolishMode(float attack_Loc);//���㹥��ģʽ����
void CTR_DefenseMode(void);// ����ģʽ
void CTR_YuntaiCTRMode(void);//��̨����ģʽ
void CTR_CrazyMode(void);//����ģʽ


void CTR_WIFIMode(void);//WIFI�ٶȿ���ģʽ

void GarderModeChange(void);//ģʽ�л�

//�����˿��ƺ���
void vRoboteCtr_Task(void *pvParameters)//���Ƴ���
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	GarderMode Mode_pre=Mode_Cruise;
	float Attack_Loc=1000;
	while(1)
	{
		
		if(!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+RemoteCtr_No))))//���ң������
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
			Motor1.pid_control_type=MotorSPD_Ctr;//ң�������� ����Ϊ�ٶȿ���
			Motor2.pid_control_type=MotorSPD_Ctr;
			Motor1.Com.Spd=RC_Data.ch4*8;
			Motor2.Com.Spd=RC_Data.ch4*8;
		}
		else//ң�����ر�
		{
			if( SystemState.Time<60000 && MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+YUNTAICtrBoard_No)))//�ȴ���̨����
			{
					vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
					continue;
			}
			SystemState.Enable=SystemState_Enable;
			GarderModeChange();//ģʽ�л�
			//============================����״̬ ���ĵ�����Ʒ�ʽ
			switch(SystemState.Robote.RunState)
			{
				case State_AutoCTR:
					switch(SystemState.Robote.Mode)
					{
						case Mode_YuntaiCTR:
							Motor1.pid_control_type=MotorSPD_Ctr;// ��̨���Ʒ�ʽ
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
				
					if(GetSystemTime()-SystemState.Wifi_ctrTime>1000)//û��WIFI���� 1s ���л�Ϊ�Զ�����ģʽ
						SystemState.Robote.RunState=State_AutoCTR;
					
					break;
				default:
					
					break;					
			}
			//============================����ģʽ
			if(SystemState.Robote.RunState==State_WifiCTR)//�����WIFI����ģʽ
			{
				CTR_WIFIMode();
			}
			else
			{
				switch(SystemState.Robote.Mode)// ģʽ�л��� WIFI������ ���� ������̨ͨ�Ź����и���
				{
					case Mode_Cali:
						break;
					case Mode_Cruise:
							CTR_CruiseMode();//Ѳ��ģʽ
						break;
					case Mode_Attack:
							if(Mode_pre!=Mode_Attack)
								Attack_Loc=Motor1.State.Loc;
								CTR_AttackMode(Attack_Loc);//����ģʽ
						break;
					case Mode_Foolish:
						if(Mode_pre!=Mode_Attack)
								Attack_Loc=Motor1.State.Loc;
								CTR_FoolishMode(Attack_Loc);//���㹥��ģʽ
						break;
					case Mode_Defense:
							CTR_DefenseMode();//  ����ģʽ
						break;
					case Mode_Crazy:
						CTR_CrazyMode();//����ģʽ
						break;
					case  Mode_YuntaiCTR://��̨����ģʽ
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


//�������
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
			Motor1.State.Loc=4500; //���ȫ�� 5486
			Motor2.State.Loc=4500;
		}
			
		else
			SystemState.Robote.Limite_Left=1000;
		
		if(LB & LB_Pre)
		{
			SystemState.Robote.Limite_Right=400;
				Motor1.State.Loc=400; //���ȫ�� 5486
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
			if(deviceNo==Judgment_hurt_No+MotorTotal_No)//����������ϵͳ�˺�
				continue;
			if(deviceNo==RemoteCtr_No+MotorTotal_No)//�������ң��
				continue;
			if(deviceNo==WIFIBoard_No+MotorTotal_No)//�������WIFI
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

//��̨ң����Ϣ����
void vRemoteSenTOYuntai_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	unsigned char data[8]={0};
	RC_Data_Short tem_Data_Short={0};
	while(1)
	{
		if(!MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+RemoteCtr_No)))//��ȡң����״̬
		{
			tem_Data_Short.ch1=RC_Data.ch1;
			tem_Data_Short.ch2=RC_Data.ch2;
			tem_Data_Short.switch_left=RC_Data.switch_left;
			tem_Data_Short.switch_right=RC_Data.switch_right;
			tem_Data_Short.State=0;
			tem_Data_Short.Bullet_Remain=1;//��������ó�
			RemoteMsg_CanSent(data,&tem_Data_Short);
			Can_Sent_msgToQue(2,CAN_YUNTAI_Ctr_RXID,data);
		}
		
		//============================== ����̨���� ������λ��
		
		if(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_hurt_No)))//��ȡ����ϵͳ�˺���������Ƿ����
			JudgeMent.RobotHurt.armorType=8;//���100ms��װ��û���ܵ�������װ������

		//  [0:1]����ֵ ��λ��ǰ   [2:5]λ����Ϣ [��8λ �θ�] [7]������Ȩ
		memset(data,0,8);
		data[0]=(unsigned char)(JudgeMent.PoerHeatData.shooterHeat0>>8);//����ֵ
		data[1]=(unsigned char)(JudgeMent.PoerHeatData.shooterHeat0&0xff);
		
		data[2]=(unsigned char)(((int)Motor1.State.Loc>>24)&0xff);//M1λ��
		data[3]=(unsigned char)(((int)Motor1.State.Loc>>16)&0xff);
		data[4]=(unsigned char)(((int)Motor1.State.Loc>>8)&0xff);
		data[5]=(unsigned char)( (int)Motor1.State.Loc&0xff);
		
		data[6]=SystemState.Robote.GarderFire<<4;//������Ȩ
		if(JudgeMent.RobotHurt.hurType==0)//ֻ���ܵ�������װ�����ͲŻᷢ��
			data[6]=data[6] | (JudgeMent.RobotHurt.armorType);
		else
			data[6]=data[6] | 0x08;//װ��û���ܵ����� ����װ�׺�8
		
		if(MyFlagGet(SystemState.OutLine_Flag,Motor1_No) || MyFlagGet(SystemState.OutLine_Flag,Motor2_No))
			MyFlagSet(data[7],1);//�������ʧ��
		
		Can_Sent_msgToQue(2,CAN_YUNTAI_Ctr2_RXID,data);
		
		if(!MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))//����ϵͳ�Ƿ�����
		{
			memset(data,0,8);
			data[0]=(unsigned char)(JudgeMent.GameRobotState.remainHP>>8);//����ʣ��Ѫ��
			data[1]=(unsigned char)(JudgeMent.GameRobotState.remainHP&0xff);
			Can_Sent_msgToQue(2,CAN_YUNTAI_Ctr3_RXID,data);
		}
		
		vTaskDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);
	}
}



//���ڴ���
void UARTAnalys(UART_HandleTypeDef *huart,char* s,int len)
{
	if(huart==&huart1)
	{
		judgement_data_Division((uint8_t *)s);//����ϵͳ����
	}
	
	if(huart==&huart4)//WIFI����
	{
		switch(WifiDataRec((unsigned char*)s,&Wifi))
		{
			case Wifi_PingCom://Ping����
					 if(Wifi.RxAdr==WifiState_Gaurder)//�жϽ��շ�
						Wifi_PingASK(Wifi.RxAdr,Wifi.TxAdr,Wifi.Data.ID);//�ظ�PingASK
				break;
			 
			case Wifi_Success://����ģʽ
				
					if(Wifi.RxAdr!=WifiState_Gaurder)//�жϽ��շ�
						break;
					if(Wifi.Data.Com!=WifiCom_GarderRemoteCTR)//��������ڱ�ģ�����
						break;
					SystemState.Robote.RunState=State_WifiCTR;//����ģʽ�л�ΪWIFI����ģʽ
					//SystemState.Robote.Mode=(GarderMode)Wifi.Data.Mode;//����ģʽΪ��������ģʽ
					SystemState.Wifi_ctrTime=GetSystemTime();//��ȡ����WIFI����ʱ��
				break;
			default:
				break;
		}
	}
}

//========================================================
//
//									��������
//
//
//==========================================================
int n=0;
int g_mem_min=0;//��Сʣ���ڴ�
int g_mem_n=0;//��ǰʣ���ڴ�
void vTest_Task(void *pvParameters)//���Գ���
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	char printdata[30]={0};
	while(1)
	{
		g_mem_min=xPortGetMinimumEverFreeHeapSize();//��ȡ��С�ڴ�ʣ��
		g_mem_n=xPortGetFreeHeapSize();//��ȡ��ǰ�ڴ�ʣ��
		
		if(SystemState.Robote.RunState==State_WifiCTR)//�����WIFI����ģʽ
			{
				My_print("State_WifiCTR\r\n");
			}
			else
			{
				switch(SystemState.Robote.Mode)// ģʽ�л��� WIFI������ ���� ������̨ͨ�Ź����и���
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
					case  Mode_YuntaiCTR://��̨����ģʽ
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
//												��������			
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
//									�ڲ�����
//
//
//==========================================================


//=====================================
//����̨��ȡ�Ƿ��ֵо�
//��һλ �Ƿ��ֵо� (0:δ���� 1������)
//
void GetMsgFromYunTai(char*s)
{
	SystemState.Robote.FindEnemy=(GarderFindEnemyDef)(s[0]& 0x01);//�Ƿ��ֵо�
	SystemState.Robote.YuntaiApplay=(YuntaiApplayDef)((s[0]>>1) & 0x01);//��̨������Ƶ���
	SystemState.Robote.Yuntai_CTR_APP_SPD=(short)s[1]<<8|s[2];//��̨���������ٶȽ���
}

//����̨��ȡ����2006���ת�ٵ���
void Get2006MotorMoveFromYunTai(char*s)
{
	SystemState.Robote.M2006_A_Current=(short)s[4]<<8|s[5];
	SystemState.Robote.M2006_B_Current=(short)s[6]<<8|s[7];
}

float g_BitByHeroTime=0; //��Ӣ�۹���ʱ��
unsigned short g_HP_Pre=0;//�ϴ�Ѫ��
//�ڱ�ģʽ�л�
void GarderModeChange()
{
	if(g_HP_Pre-JudgeMent.GameRobotState.remainHP>490)
		g_BitByHeroTime=GetSystemTime();//��¼��ǰ��Ӣ�ۻ���ʱ��
	g_HP_Pre=JudgeMent.GameRobotState.remainHP;//����ʱ��
	switch(SystemState.Robote.RunState)
	{
		case State_AutoCTR:
				
				if(JudgeMent.GameRobotState.remainHP<=1500 ||  (GetSystemTime()-g_BitByHeroTime<5000))//���Ѫ������1000 ���߱�Ӣ�۹��� �л�Ϊ����ģʽ   ������ȼ�
				{
						SystemState.Robote.Mode=Mode_Crazy;//�л�Ϊ����ģʽ 
						return;
				}
				else
				{	
					if(SystemState.Robote.Mode==Mode_Crazy)//���Ѫ������1000 ������ģʽ�ͷŵ�Ѳ��ģʽ
						SystemState.Robote.Mode=Mode_Cruise;
				}
			
				if(Enemy_Finded==SystemState.Robote.FindEnemy)//�ҵ��о�
				{
					if(SystemState.Robote.Mode==Mode_Cruise)
					{
						SystemState.Robote.Mode=Mode_Attack;//ת��Ϊ����ģʽ
					}
				}
				else
				{
					if(SystemState.Robote.Mode==Mode_Attack || SystemState.Robote.Mode==Mode_Foolish)
						SystemState.Robote.Mode=Mode_Cruise;//û�з��ֵо� �ͷŻ�Ѳ��ģʽ
				}
						
				//������̨����
				if(SystemState.Robote.YuntaiApplay==YuntaiCTR_Applay  //��̨�������� �Ҳ��Ǳ���ģʽ���Ƿ���ģʽ
						&& SystemState.Robote.Mode!=Mode_Defense 
						&& SystemState.Robote.Mode!=Mode_Crazy)
					SystemState.Robote.Mode=Mode_YuntaiCTR;//����Ϊ��̨����
				else
				{
						if(SystemState.Robote.Mode==Mode_YuntaiCTR)//�������̨���ٿ��ƽ�����ģʽ�ͷŵ�Ѳ��ģʽ
							SystemState.Robote.Mode=Mode_Cruise;
				}
			break;
		
		case State_WifiCTR://wifiģʽ�л���WIFI���մ��������︳ֵ
			break;
		default:
			break;
	}
}


// �������� �Ҳ�Ϊ���� 0  ���ΪԶ�� 4500+  ��������Ϊ��
#define FarDist 4400  //Զ������λ��
#define NearMidDist 2200  //�����в�����λ��
#define NearDist 500  //��������λ��
#define Cruise_SPD 500 // Ѳ���ٶ�

#define Attack_SPD 1 //����ģʽ�µ��ٶ�
#define Attack_Dist 2 //����ģʽ�µ������ƶ��ľ���

void CTR_CruiseMode()//Ѳ��ģʽ����
{
	static char Run_Dir=0;// 0:��Զ������ 1:���������
	
	if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//�������ϵͳ�����Լ���������������90s���߽����ս�У�
		SystemState.Robote.GarderFire=Fire_Permite;//������
	else
		SystemState.Robote.GarderFire=Fire_Deny;//��ֹ����
	
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
	
	//ͨ�����������Ʒ���  SystemState.Robote.Limite_Left
	if(Motor1.State.Loc>FarDist-100)Run_Dir=1;
	//if(SystemState.Robote.Limite_Left < 500 || Motor1.State.Loc>8000)Run_Dir=1;
	if(Motor1.State.Loc<NearDist+100)Run_Dir=0;
	
}

void CTR_AttackMode(float attack_Loc)//����ģʽ����
{
	static char Run_Dir=0;// 0:��Զ������ 1:���������
	
	if(attack_Loc+Attack_Dist>FarDist)attack_Loc=FarDist-Attack_Dist;
	if(attack_Loc-Attack_Dist<NearDist)attack_Loc=NearDist+Attack_Dist;
	
	if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//�������ϵͳ�����Լ���������������90s���߽����ս�У�
		SystemState.Robote.GarderFire=Fire_Permite;//������
	else
		SystemState.Robote.GarderFire=Fire_Deny;//��ֹ����
	
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

void CTR_FoolishMode(float attack_Loc)//���㹥��ģʽ����
{
	if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//�������ϵͳ�����Լ���������������90s���߽����ս�У�
		SystemState.Robote.GarderFire=Fire_Permite;//������
	else
		SystemState.Robote.GarderFire=Fire_Deny;//��ֹ����
	
	Motor1.Com.Loc=attack_Loc;
	Motor2.Com.Loc=attack_Loc;
}

void CTR_DefenseMode()
{
	static char Run_Dir=0;// 0:��Զ������ 1:���������
	
	SystemState.Robote.GarderFire=Fire_Deny;//��ֹ����
	
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
//����ģʽ����
void CTR_CrazyMode()
{
	static char Run_Dir=0;// 0:��Զ������ 1:���������
	static char Run_Loc=0;// 0Ϊ�Ҳൽ�յ�  1Ϊ��ൽ�յ�
	static char Run_Dir_pre=1;

	
	if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//�������ϵͳ�����Լ���������������90s���߽����ս�У�
		SystemState.Robote.GarderFire=Fire_Permite;//������
	else
		SystemState.Robote.GarderFire=Fire_Deny;//��ֹ����
	
	if(Run_Loc==0)//���Ҳൽ�е�
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
	else //����ൽ�е�
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
		
	
	if(Run_Loc==0)//���Ҳൽ�е�
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

	if(Crazy_n<=0)//ͨ��ѭ����������0ʱ ��λ����
	{
		Crazy_n=SystemState.Time%8+1;//��ȡ��ǰʱ���5����������ѭ���������趨
		if(Run_Loc==0)
			Run_Loc=1;
		else
			Run_Loc=0;
	}
}

//WIFI����ģʽ  ���Ҳ���0
void CTR_WIFIMode()
{
	short KeyCode=Wifi.Data.ch1;
	
	if(DBUS_CheckButtonState(KeyCode,KeyCode,KEY_W)==Key_Down)//����ǰ��
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
	
	if(DBUS_CheckButtonState(KeyCode,KeyCode,KEY_Z)==Key_Down)//ģʽ�л�Ϊ����
	{
			SystemState.Robote.Mode=Mode_Cruise;//Ѳ��ģʽ
	}
	
	if(DBUS_CheckButtonState(KeyCode,KeyCode,KEY_Q)==Key_Down)//ģʽ�л�Ϊ����ģʽ
	{
			SystemState.Robote.Mode=Mode_Defense;
	}
	
	if(SystemState.Robote.Mode==Mode_Defense)//����Ƿ���ģʽ
	{
		SystemState.Robote.GarderFire=Fire_Deny;//��ֹ����
	}
	else
	{
		if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//�������ϵͳ�����Լ���������������90s���߽����ս�У�
		SystemState.Robote.GarderFire=Fire_Permite;//������
	else
		SystemState.Robote.GarderFire=Fire_Deny;//��ֹ����
	}

}

//��̨����ģʽ
void CTR_YuntaiCTRMode()
{
	if((!(MyFlagGet(SystemState.OutLine_Flag,(MotorTotal_No+Judgment_No)))) && 
		(SystemState.Time>90000 || JudgeMent.GameRobotState.gameProgress==4))//�������ϵͳ�����Լ���������������90s���߽����ս�У�
		SystemState.Robote.GarderFire=Fire_Permite;//������
	else
		SystemState.Robote.GarderFire=Fire_Deny;//��ֹ����
	if(Motor1.State.Loc>FarDist & SystemState.Robote.Yuntai_CTR_APP_SPD>0)//����Զ���ٶ�
	{
		Motor1.Com.Spd=0;
		Motor2.Com.Spd=0;
	}
	else if(Motor1.State.Loc<NearDist & SystemState.Robote.Yuntai_CTR_APP_SPD<0)//���ƽ����ٶ�
	{
		Motor1.Com.Spd=0;
		Motor2.Com.Spd=0;
	}
	else
	{
		if(my_abs(SystemState.Robote.Yuntai_CTR_APP_SPD)>3000)//�ٶ��޷�
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
