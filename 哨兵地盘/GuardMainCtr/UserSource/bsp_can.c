/**
  ******************************************************************************
  * @file    bsp_can.c
  * @author  
  * @version V1.0
  * @date    2016/12/26
  * @brief   
  * 
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "cmsis_os.h"
#include "string.h"

// othor Lib
#include "SystemState.h"
#include "bsp_motor.h"
#include "Remote.h"

xQueueHandle Can_TXData_QueHandle;//Can�������ݶ���
int CanSentID=0;
int CanRecID=0;

//=====================================================
//							��Ҫ�ⲿʵ��
//
//=====================================================
//��ȡCanMsg��Ϣ
__weak void GetCanMsg(char *s)
{
	UNUSED(s);
}

//��������������
__weak void SentCanheartMsg_MotorLoc(unsigned char* s)
{
	UNUSED(s);
}
__weak void MesureTimePeriod()
{
	char* s;
	UNUSED(s);
}
 //����̨��ȡ�Ƿ��ֵо�
__weak void GetMsgFromYunTai(char*s)
{
	UNUSED(s);
}

//����̨��ȡ����2006�������
__weak void Get2006MotorMoveFromYunTai(char*s)
{
	UNUSED(s);
}

//=====================================================
//						  �ڲ�����
//
//=====================================================
//��ȡ���ӷ���ID
void GetSW_ID()
{
	unsigned char SW1=0,SW2=0;
	if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)==GPIO_PIN_RESET)SW1=1;
	if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)==GPIO_PIN_RESET)SW2=1;

	
	CanSentID = StepMOTOR_BACK + (SW1 | SW2<<1);
	CanRecID  = StepMOTOR + (SW1 | SW2<<1 );
}

short g_testoutput=0;

//CAN�жϵĻص�����
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	//��CANι��
	//HAL_IWDG_Refresh(&hiwdg);
		
	if(_hcan == &hcan1)
  {
    switch(_hcan->pRxMsg->StdId)
    {
			case MOTOR_A_BACK: //�ڱ���Motor1 ID  Ϊ ���̳��� Motor3
				RefreshMotorOutLineTime(Motor1_No);
				GetMotorState(_hcan->pRxMsg->Data, &Motor1);
				break;
			case MOTOR_B_BACK://�ڱ���Motor2 ID  Ϊ ���̳��� Motor4
				RefreshMotorOutLineTime(Motor2_No);
				GetMotorState(_hcan->pRxMsg->Data, &Motor2);
				break;

			case MOTOR_Yaw_BACK:// Yaw�����̨͸��
				Can_Sent_msgToQueFromISR(2,_hcan->pRxMsg->StdId,_hcan->pRxMsg->Data);
				RefreshDeviceOutLineTime(YAW6623_No);
			break;
			
			case MOTOR2006_A_BACK:// ���������̨͸�� 
				Can_Sent_msgToQueFromISR(2,_hcan->pRxMsg->StdId,_hcan->pRxMsg->Data);
				RefreshDeviceOutLineTime(Motor2006CTR_No);
			break;
			
			case MOTOR2006_B_BACK:// ���������̨͸��
				Can_Sent_msgToQueFromISR(2,_hcan->pRxMsg->StdId,_hcan->pRxMsg->Data);
			break;

      default:
					
        break;
    }
  }
	
	if(_hcan == &hcan2)
  {
    switch(_hcan->pRxMsg->StdId)
    {
			case CAN_YUNTAI_Ctr_BACK :
				GetMsgFromYunTai((char*)_hcan->pRxMsg->Data);//����̨��ȡ��Ϣ
				RefreshDeviceOutLineTime(YUNTAICtrBoard_No);
				break;
			case MOTOR_Yaw_Move:
				Can_Sent_msgToQueFromISR(1,_hcan->pRxMsg->StdId,_hcan->pRxMsg->Data);
				g_testoutput=(short)_hcan->pRxMsg->Data[0]<<8|_hcan->pRxMsg->Data[1];
				break;
			
			case MOTOR_Move://��̨���Ʋ������
				Get2006MotorMoveFromYunTai((char*)_hcan->pRxMsg->Data);//����̨��ȡ����2006���ת�� Motor2006CTR_No
				RefreshDeviceOutLineTime(Motor2006CTR_No);
				break;
      default:
				
        break;
    }
  }
	__HAL_CAN_ENABLE_IT(_hcan, CAN_IT_FMP0);
}


//����can����Ϣ
HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, CAN_Message_ID _id, uint8_t* _message)
{
  HAL_StatusTypeDef halStatus;
	
  _hcan->pTxMsg->RTR = CAN_RTR_DATA;
  _hcan->pTxMsg->IDE = CAN_ID_STD;
  _hcan->pTxMsg->StdId = _id;
	_hcan->pTxMsg->DLC = 8;

	memcpy(&_hcan->pTxMsg->Data[0],_message,8);
	
	//ʱ�䲻��̫С����Ȼ��timeout
	halStatus = HAL_CAN_Transmit(_hcan, 0x24);
  
	
	if(_id==0x1ff)MesureTimePeriod();
	
  return(halStatus);
}

//�������ݵ�Can���Ͷ���
BaseType_t Can_Sent_msgToQue(short can_num,short ID, unsigned char* date)
{
	CAN_HandleTypeDef* canHandle=NULL;
	BaseType_t queueStatus;
	char dateToSent[15]={0};
	//int*p=(int*)dateToSent;
	//short*q=(short*)&dateToSent[4];
	if(can_num==1)canHandle=&hcan1;
	if(can_num==2)canHandle=&hcan2;
	*(int*)dateToSent=(int)canHandle;
	*(short*)&dateToSent[4]=ID;
	
  memcpy(&dateToSent[6],date,8);
	queueStatus=xQueueSendToBack(Can_TXData_QueHandle,dateToSent,0);
	return queueStatus;
}

//�������ݵ�Can���Ͷ���
BaseType_t Can_Sent_msgToQueFromISR(short can_num,short ID, unsigned char* date)
{
	CAN_HandleTypeDef* canHandle=NULL;
	BaseType_t queueStatus;
	char dateToSent[15]={0};
	//int*p=(int*)dateToSent;
	//short*q=(short*)&dateToSent[4];
	if(can_num==1)canHandle=&hcan1;
	if(can_num==2)canHandle=&hcan2;
	*(int*)dateToSent=(int)canHandle;
	*(short*)&dateToSent[4]=ID;
	
  memcpy(&dateToSent[6],date,8);
	queueStatus=xQueueSendToBackFromISR(Can_TXData_QueHandle,dateToSent,0);
	return queueStatus;
}

//Can��������
void vCanSent_Task(void *pvParameters)
{
	char s[14]={0};
	while(1)
	{
		xQueueReceive(Can_TXData_QueHandle,s,portMAX_DELAY);
		CAN_Send_Message((CAN_HandleTypeDef*)(*(int*)s), ( CAN_Message_ID)*((int*)&s[4]), (unsigned char*)&s[6]);
	}
}

//Can����������
void vCanHeartSent_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	//unsigned char data[8]={0};
	while(1)
	{
		//SentCanheartMsg_MotorLoc(data);
		//Can_Sent_msgToQue(1,CanSentID, data);
		vTaskDelayUntil(&xLastWakeTime,10/ portTICK_RATE_MS);
	}

}
//=====================================================
//							  �ⲿ����
//
//=====================================================
/**
  * @brief  ��can.c��HAL_CAN_MspInit�е��ã�can���˲���������
  * @param  CAN_HandleTypeDef* hcan
  * @retval ��ӵ� can.c �� HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle) can1��ʼ� ���
  */
void My_CAN1_FilterConfig(CAN_HandleTypeDef* _hcan)
{
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;
	static CanTxMsgTypeDef		TxMessage;
  static CanRxMsgTypeDef 		RxMessage;
	
	CAN_FilterConfigStructure.FilterNumber = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	//CAN_FilterConfigStructure.FilterIdHigh= (((uint32_t)(CAN_MAIN_CONTROL_RECEIVE_ID)<<21)&0xFFFF0000)>>16;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	//CAN_FilterConfigStructure.FilterMaskIdHigh = 0xFFE0;//1111 1111 1110 0000
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.BankNumber = 14;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;
	
	//��ʼ�����ɹ�������ѭ����̫�ã���Ȼ����һ�㲻����ʲô���⣩
	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
		while(1);
	}

	_hcan->pTxMsg = &TxMessage;
	_hcan->pRxMsg = &RxMessage;
}

void My_CAN2_FilterConfig(CAN_HandleTypeDef* _hcan)
{
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;
	static CanTxMsgTypeDef		TxMessage;
  static CanRxMsgTypeDef 		RxMessage;
	
	CAN_FilterConfigStructure.FilterNumber = 14;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	//CAN_FilterConfigStructure.FilterIdHigh= (((uint32_t)(CAN_MAIN_CONTROL_RECEIVE_ID)<<21)&0xFFFF0000)>>16;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	//CAN_FilterConfigStructure.FilterMaskIdHigh = 0xFFE0;//1111 1111 1110 0000
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.BankNumber = 14;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;
	
	//��ʼ�����ɹ�������ѭ����̫�ã���Ȼ����һ�㲻����ʲô���⣩
	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
		while(1);
	}

	_hcan->pTxMsg = &TxMessage;
	_hcan->pRxMsg = &RxMessage;
}

//Can��ʼ�� �ɹ�����1 ʧ�ܷ���0
int Can_Inite()
{
	GetSW_ID();//��ȡ���뿪��ID
	Can_TXData_QueHandle=xQueueCreate(20,14);//��Ϣ���� ���Ϊ20 ÿ����Ԫ14�ֽ�
	if(!Can_TXData_QueHandle)return 0;
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);//��can�ж�
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);//��can�ж�
	if(!xTaskCreate( vCanSent_Task, "CanSent_Task", 200, NULL, 4, NULL ))return 0;//Can��������
	//if(!xTaskCreate( vCanHeartSent_Task, "Canheart_Task", 200, NULL, 3, NULL ))return 0;//Can��������
	printf("Can Inite OK.\r\n");
	return 1;
}
