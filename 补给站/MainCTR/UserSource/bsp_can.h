#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "can.h"
#include "cmsis_os.h"

/** 
  * @brief  CAN发送或是接收的ID
  */
typedef enum
{
	ZERO,
	Mos_CTR_Rev_ID =0x08, //主控发往 补给站MOS
	Mos_CTR_Back_ID =0x018, //补给站Mos 发往主控
	
	MOTOR_3510_BACK=0x201,

}CAN_Message_ID;

void My_CAN1_FilterConfig(CAN_HandleTypeDef* _hcan);
void My_CAN2_FilterConfig(CAN_HandleTypeDef* _hcan);
HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, CAN_Message_ID, uint8_t* _message);
BaseType_t Can_Sent_msgToQue(short can_num,short ID, unsigned char* date);//发送数据到Can发送队列

int Can_Inite(void);//Can初始化 成功返回1 失败返回0

extern xQueueHandle Can_TXData_QueHandle;//Can发送数据队列
extern int g_CanSent_n;//CAN消息发送个数
#endif
