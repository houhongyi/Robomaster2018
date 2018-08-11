#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "can.h"
#include "cmsis_os.h"

/** 
  * @brief  CAN���ͻ��ǽ��յ�ID
  */
typedef enum
{
	ZERO,
	Mos_CTR_Rev_ID =0x08, //���ط��� ����վMOS
	Mos_CTR_Back_ID =0x018, //����վMos ��������
	
	MOTOR_3510_BACK=0x201,

}CAN_Message_ID;

void My_CAN1_FilterConfig(CAN_HandleTypeDef* _hcan);
void My_CAN2_FilterConfig(CAN_HandleTypeDef* _hcan);
HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, CAN_Message_ID, uint8_t* _message);
BaseType_t Can_Sent_msgToQue(short can_num,short ID, unsigned char* date);//�������ݵ�Can���Ͷ���

int Can_Inite(void);//Can��ʼ�� �ɹ�����1 ʧ�ܷ���0

extern xQueueHandle Can_TXData_QueHandle;//Can�������ݶ���
extern int g_CanSent_n;//CAN��Ϣ���͸���
#endif
