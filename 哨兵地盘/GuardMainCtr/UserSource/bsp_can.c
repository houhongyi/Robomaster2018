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

xQueueHandle Can_TXData_QueHandle;//Can·¢ËÍÊý¾Ý¶ÓÁÐ
int CanSentID=0;
int CanRecID=0;

//=====================================================
//							ÐèÒªÍâ²¿ÊµÏÖ
//
//=====================================================
//»ñÈ¡CanMsgÐÅÏ¢
__weak void GetCanMsg(char *s)
{
	UNUSED(s);
}

//·¢ËÍÐÄÌø°üÄÚÈÝ
__weak void SentCanheartMsg_MotorLoc(unsigned char* s)
{
	UNUSED(s);
}
__weak void MesureTimePeriod()
{
	char* s;
	UNUSED(s);
}
 //´ÓÔÆÌ¨»ñÈ¡ÊÇ·ñ·¢ÏÖµÐ¾ü
__weak void GetMsgFromYunTai(char*s)
{
	UNUSED(s);
}

//´ÓÔÆÌ¨»ñÈ¡¿ØÖÆ2006µç»úµçÁ÷
__weak void Get2006MotorMoveFromYunTai(char*s)
{
	UNUSED(s);
}

//=====================================================
//						  ÄÚ²¿º¯Êý
//
//=====================================================
//»ñÈ¡°å×Ó·¢ËÍID
void GetSW_ID()
{
	unsigned char SW1=0,SW2=0;
	if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)==GPIO_PIN_RESET)SW1=1;
	if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)==GPIO_PIN_RESET)SW2=1;

	
	CanSentID = StepMOTOR_BACK + (SW1 | SW2<<1);
	CanRecID  = StepMOTOR + (SW1 | SW2<<1 );
}

short g_testoutput=0;

//CANÖÐ¶ÏµÄ»Øµ÷º¯Êý
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	//ÓÃCANÎ¹¹·
	//HAL_IWDG_Refresh(&hiwdg);
		
	if(_hcan == &hcan1)
  {
    switch(_hcan->pRxMsg->StdId)
    {
			case MOTOR_A_BACK: //ÉÚ±øµÄMotor1 ID  Îª ¹¤³Ì³µµÄ Motor3
				RefreshMotorOutLineTime(Motor1_No);
				GetMotorState(_hcan->pRxMsg->Data, &Motor1);
				break;
			case MOTOR_B_BACK://ÉÚ±øµÄMotor2 ID  Îª ¹¤³Ì³µµÄ Motor4
				RefreshMotorOutLineTime(Motor2_No);
				GetMotorState(_hcan->pRxMsg->Data, &Motor2);
				break;

			case MOTOR_Yaw_BACK:// Yawµç»úÔÆÌ¨Í¸´«
				Can_Sent_msgToQueFromISR(2,_hcan->pRxMsg->StdId,_hcan->pRxMsg->Data);
				RefreshDeviceOutLineTime(YAW6623_No);
			break;
			
			case MOTOR2006_A_BACK:// ²¦µ¯µç»úÔÆÌ¨Í¸´« 
				Can_Sent_msgToQueFromISR(2,_hcan->pRxMsg->StdId,_hcan->pRxMsg->Data);
				RefreshDeviceOutLineTime(Motor2006CTR_No);
			break;
			
			case MOTOR2006_B_BACK:// ²¦µ¯µç»úÔÆÌ¨Í¸´«
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
				GetMsgFromYunTai((char*)_hcan->pRxMsg->Data);//´ÓÔÆÌ¨»ñÈ¡ÐÅÏ¢
				RefreshDeviceOutLineTime(YUNTAICtrBoard_No);
				break;
			case MOTOR_Yaw_Move:
				Can_Sent_msgToQueFromISR(1,_hcan->pRxMsg->StdId,_hcan->pRxMsg->Data);
				g_testoutput=(short)_hcan->pRxMsg->Data[0]<<8|_hcan->pRxMsg->Data[1];
				break;
			
			case MOTOR_Move://ÔÆÌ¨¿ØÖÆ²¦µ¯µç»ú
				Get2006MotorMoveFromYunTai((char*)_hcan->pRxMsg->Data);//´ÓÔÆÌ¨»ñÈ¡¿ØÖÆ2006µç»ú×ªËÙ Motor2006CTR_No
				RefreshDeviceOutLineTime(Motor2006CTR_No);
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

//Can³õÊ¼»¯ ³É¹¦·µ»Ø1 Ê§°Ü·µ»Ø0
int Can_Inite()
{
	GetSW_ID();//»ñÈ¡²¦Âë¿ª¹ØID
	Can_TXData_QueHandle=xQueueCreate(20,14);//ÐÅÏ¢¶ÓÁÐ Éî¶ÈÎª20 Ã¿¸öµ¥Ôª14×Ö½Ú
	if(!Can_TXData_QueHandle)return 0;
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);//´ò¿ªcanÖÐ¶Ï
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);//´ò¿ªcanÖÐ¶Ï
	if(!xTaskCreate( vCanSent_Task, "CanSent_Task", 200, NULL, 4, NULL ))return 0;//Can·¢ËÍÈÎÎñ
	//if(!xTaskCreate( vCanHeartSent_Task, "Canheart_Task", 200, NULL, 3, NULL ))return 0;//CanÐÄÌø·¢ËÍ
	printf("Can Inite OK.\r\n");
	return 1;
}
