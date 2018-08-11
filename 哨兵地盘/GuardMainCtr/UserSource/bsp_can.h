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
	MOTOR_Yaw_BACK=0x205,
	MOTOR_2_BACK=0x206,
	MOTOR_3_BACK=0x207,
	MOTOR_4_BACK=0x208,
	
	MOTOR_A_BACK=0x201,
	MOTOR_B_BACK=0x202,
	MOTOR_C_BACK=0x203,
	MOTOR_D_BACK=0x204,
	MOTOR_E_BACK=0x201,
	MOTOR_F_BACK=0x202,
	MOTOR2006_A_BACK=0x203,
	MOTOR2006_B_BACK=0x204,

	StepMOTOR      = 0x210, //����������ư�
	StepMOTOR_BACK = 0x220, //����������ư巵��ID
	
	CAN_MAIN_CONTROL_RECEIVE_ID   = 0x110,  //���ذ����ID
	CAN_EXTEND_RECEIVE_ID         = 0x101,  //��չ������ID
	CAN_EXTEND_SENT_ID            = 0x111,  //��չ��巢��ID
	
	CAN_YUNTAI_Ctr_RXID           = 0x230, //��̨�����ID
	CAN_YUNTAI_Ctr2_RXID          = 0x232, //��̨��2����ID
	CAN_YUNTAI_Ctr3_RXID          = 0x233, //��̨��3����ID
	
	CAN_YUNTAI_Ctr_BACK           = 0x231, //��̨�巵��ID
	
	MOTOR_Yaw_Move=0x1ff,
	MOTOR_Move =0x200,
	MOTOR_Lift =0x200

}CAN_Message_ID;

void My_CAN1_FilterConfig(CAN_HandleTypeDef* _hcan);
void My_CAN2_FilterConfig(CAN_HandleTypeDef* _hcan);
HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, CAN_Message_ID, uint8_t* _message);
BaseType_t Can_Sent_msgToQue(short can_num,short ID, unsigned char* date);//�������ݵ�Can���Ͷ���

BaseType_t Can_Sent_msgToQueFromISR(short can_num,short ID, unsigned char* date);//�������ݵ�Can���Ͷ���

int Can_Inite(void);//Can��ʼ�� �ɹ�����1 ʧ�ܷ���0

extern xQueueHandle Can_TXData_QueHandle;//Can�������ݶ���
#endif
