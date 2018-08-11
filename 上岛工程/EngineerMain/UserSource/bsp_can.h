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
	MOTOR_A,
	MOTOR_B,
	MOTOR_C,
	MOTOR_D,
	MOTOR_CATCH=0x08,
	
	MOTOR_1_BACK=0x205,
	MOTOR_2_BACK=0x206,
	MOTOR_3_BACK=0x207,
	MOTOR_4_BACK=0x208,
	
	MOTOR_A_BACK=0x201,
	MOTOR_B_BACK=0x202,
	MOTOR_C_BACK=0x203,
	MOTOR_D_BACK=0x204,
	MOTOR_5_BACK=0x207,
	MOTOR_6_BACK=0x208,
	 
	CAN_MAIN_CONTROL_RECEIVE_ID   = 0x110,  //���ذ����ID
	CAN_EXTEND_RECEIVE_ID         = 0x101,  //Mos1������ID
	CAN_EXTEND_SENT_ID            = 0x111,  //Mos1��巢��ID
	
	CAN_EXTEND2_RECEIVE_ID        = 0x102,  //Mos2������ID
	CAN_EXTEND2_SENT_ID           = 0x112,  //Mos2��巢��ID
	
	CAN_DisMesure_RECEIVE_ID      = 0x150,  //�������ID
	CAN_DisMesure_SENT_ID         = 0x160,  //���巢��ID
	
	CAN_YunTai_RECEIVE_ID         = 0x120,  //��̨�����ID	����->��̨
	CAN_YunTai_SENT_ID            = 0x125,  //��̨�巢��ID  ��̨->����
	
	CAN_YunTaiUart_RECEIVE_ID     = 0x121,  //��̨�崮��ת������ID	����->��̨
	CAN_YunTaiUart_SENT_ID        = 0x126,  //��̨�崮��ת������ID  ��̨->����

}CAN_Message_ID;

void My_CAN1_FilterConfig(CAN_HandleTypeDef* _hcan);
void My_CAN2_FilterConfig(CAN_HandleTypeDef* _hcan);
HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, CAN_Message_ID, uint8_t* _message);
BaseType_t Can_Sent_msgToQue(short can_num,short ID, unsigned char* date);//�������ݵ�Can���Ͷ���

int Can_Inite(void);//Can��ʼ�� �ɹ�����1 ʧ�ܷ���0

extern xQueueHandle Can_TXData_QueHandle;//Can�������ݶ���
extern int g_CanSent_n;//CAN��Ϣ���͸���
#endif
