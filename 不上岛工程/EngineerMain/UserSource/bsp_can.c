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
#include "iwdg.h"

xQueueHandle Can_TXData_QueHandle;//Can·¢ËÍÊý¾Ý¶ÓÁÐ
int CanSentID=0;
int CanRecID=0;
int g_CanSent_n=0;//CANÏûÏ¢·¢ËÍ¸öÊý
//=====================================================
//							ÐèÒªÍâ²¿ÊµÏÖ
//
//=====================================================


//=====================================================
//						  ÄÚ²¿º¯Êý
//
//=====================================================


/**
  * @brief  CANÖÐ¶ÏµÄ»Øµ÷º¯Êý
  * @param  CAN_HandleTypeDef* _hcan
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	//ÓÃCANÎ¹¹·
	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);//¸üÐÂ¿´ÃÅ¹·
  if(_hcan == &hcan1)
  {
    switch(_hcan->pRxMsg->StdId)
    {
			case MOTOR_1_BACK:
					GetMotorState(_hcan->pRxMsg->Data,&Motor1);
			    g_Time_MotorOutLine[Motor1_No]=GetSystemTimer();
				break;
			case MOTOR_2_BACK:
					GetMotorState(_hcan->pRxMsg->Data,&Motor2);
					g_Time_MotorOutLine[Motor2_No]=GetSystemTimer();
				break;
			case MOTOR_3_BACK:
					GetMotorState(_hcan->pRxMsg->Data,&Motor3);
					g_Time_MotorOutLine[Motor3_No]=GetSystemTimer();
				break;
			case MOTOR_4_BACK:
					GetMotorState(_hcan->pRxMsg->Data,&Motor4);
					g_Time_MotorOutLine[Motor4_No]=GetSystemTimer();
				break;
			case MOTOR_A_BACK:
					GetMotorState(_hcan->pRxMsg->Data,&MotorA);
					g_Time_MotorOutLine[MotorA_No]=GetSystemTimer();
				break;
			case MOTOR_B_BACK:
					GetMotorState(_hcan->pRxMsg->Data,&MotorB);
					g_Time_MotorOutLine[MotorB_No]=GetSystemTimer();
				break;
			
			case CAN_EXTEND2_SENT_ID:
					RefreshDeviceOutLineTime(ExtendCtr2_No);//À©Õ¹°åÍ¨Ñ¶Ê±¼ä¼ÇÂ¼
				break;
			
      default:
        break;
    }
  }
	
	if(_hcan == &hcan2)
  {
    switch(_hcan->pRxMsg->StdId)
    {
			case MOTOR_C_BACK:
					GetMotorState(_hcan->pRxMsg->Data,&MotorC);
					g_Time_MotorOutLine[MotorC_No]=GetSystemTimer();
				break;

			case MOTOR_D_BACK:
					GetMotorState(_hcan->pRxMsg->Data,&MotorD);
					g_Time_MotorOutLine[MotorD_No]=GetSystemTimer();
				break;

			case MOTOR_E_BACK:
					GetMotorState(_hcan->pRxMsg->Data,&MotorE);
					g_Time_MotorOutLine[MotorE_No]=GetSystemTimer();
				break;
			
			case MOTOR_F_BACK:
					GetMotorState(_hcan->pRxMsg->Data,&MotorF);
					g_Time_MotorOutLine[MotorF_No]=GetSystemTimer();
				break;
			
			case CAN_EXTEND_SENT_ID:
					RefreshDeviceOutLineTime(ExtendCtr_No);//À©Õ¹°åÍ¨Ñ¶Ê±¼ä¼ÇÂ¼
				break;
			
			
      default:
				
        break;
    }
  }
	
	__HAL_CAN_ENABLE_IT(_hcan, CAN_IT_FMP0);
}


//·¢ËÍcanµÄÐÅÏ¢
HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, CAN_Message_ID _id, uint8_t* _message)
{
  HAL_StatusTypeDef halStatus;
	
  _hcan->pTxMsg->RTR = CAN_RTR_DATA;
  _hcan->pTxMsg->IDE = CAN_ID_STD;
  _hcan->pTxMsg->StdId = _id;
	_hcan->pTxMsg->DLC = 8;

	memcpy(&_hcan->pTxMsg->Data[0],_message,8);
	
	//Ê±¼ä²»ÄÜÌ«Ð¡£¬²»È»»átimeout
	halStatus = HAL_CAN_Transmit(_hcan, 0x24);
  
	
	if(_id==0x1ff)MesureTimePeriod();
	
  return(halStatus);
}

//·¢ËÍÊý¾Ýµ½Can·¢ËÍ¶ÓÁÐ
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

//·¢ËÍÊý¾Ýµ½Can·¢ËÍ¶ÓÁÐ
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

//Can·¢ËÍÈÎÎñ
void vCanSent_Task(void *pvParameters)
{
	char s[14]={0};
	while(1)
	{
		xQueueReceive(Can_TXData_QueHandle,s,portMAX_DELAY);
		CAN_Send_Message((CAN_HandleTypeDef*)(*(int*)s), ( CAN_Message_ID)*((int*)&s[4]), (unsigned char*)&s[6]);
		g_CanSent_n++;
	}
}

//CanÐÄÌø°ü·¢ËÍ
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
//							  Íâ²¿µ÷ÓÃ
//
//=====================================================

//Can³õÊ¼»¯ ³É¹¦·µ»Ø1 Ê§°Ü·µ»Ø0
int Can_Inite()
{
	Can_TXData_QueHandle=xQueueCreate(20,14);//ÐÅÏ¢¶ÓÁÐ Éî¶ÈÎª20 Ã¿¸öµ¥Ôª14×Ö½Ú
	if(!Can_TXData_QueHandle)return 0;
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);//´ò¿ªcanÖÐ¶Ï
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);//´ò¿ªcanÖÐ¶Ï
	if(!xTaskCreate( vCanSent_Task, "CanSent_Task", 200, NULL, 4, NULL ))return 0;//Can·¢ËÍÈÎÎñ
	printf("Can Inite OK.\r\n");
	return 1;
}

/**
  * @brief  ÔÚcan.cµÄHAL_CAN_MspInitÖÐµ÷ÓÃ£¬canµÄÂË²¨Æ÷µÄÅäÖÃ
  * @param  CAN_HandleTypeDef* hcan
  * @retval Ìí¼Óµ½ can.c ÖÐ HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle) can1³õÊ¼» ×îºó
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
	
	//³õÊ¼»¯²»³É¹¦½øÈëËÀÑ­»·²»Ì«ºÃ£¨ËäÈ»ÕâÀïÒ»°ã²»»áÓÐÊ²Ã´ÎÊÌâ£©
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
	
	//³õÊ¼»¯²»³É¹¦½øÈëËÀÑ­»·²»Ì«ºÃ£¨ËäÈ»ÕâÀïÒ»°ã²»»áÓÐÊ²Ã´ÎÊÌâ£©
	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
		while(1);
	}

	_hcan->pTxMsg = &TxMessage;
	_hcan->pRxMsg = &RxMessage;
}


