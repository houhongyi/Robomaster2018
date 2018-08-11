#ifndef __WS2812_H
#define __WS2812_H

#include "stm32f1xx_hal.h"
#include "tim.h"

#define LED_5050_n 8	//LED�Ƹ���
#define LED_5050_Data_n        LED_5050_n*3  //LED�����ݳ���
#define LED_5050_Data_Bit_n    LED_5050_n*3*8  //LED�����ݳ���λ��

typedef enum
{
	NoneFlash=0,
	
	UpIsLandFlow_G=0x11,
	UpIsLandFlow_Y=0x12,
	UpIsLandFlow_R=0x13,
	
	DownIsLandFlow_G=0x21,
	DownIsLandFlow_Y=0x22,
	DownIsLandFlow_R=0x23,
	
	GetBullet_GBR=0x30,
	Breath_GBR=0x50,
	
	CustomFlash=0xFF,
	
}LED_FlashModeEnum;


typedef struct
{
	LED_FlashModeEnum LED_FlashMode;
	LED_FlashModeEnum LED_FlashMode_pre;
}LED_FlashMode;

typedef struct
{
	LED_FlashMode Mode;
	short Bit_n;//���͵ڼ�λ
	unsigned char Data[LED_5050_Data_n];//�������� // ˳��ΪGRB
	unsigned char Data_Breath[LED_5050_Data_n];//��������(������) // ˳��ΪGRB
	unsigned short Data_Pulse[LED_5050_Data_Bit_n+1];//��������������
	TIM_HandleTypeDef* htim;//PWMTimer
}LED_5050DriverDef;


extern LED_5050DriverDef LED_5050Driver;

char LED_5050DriverInite(void);//LED5050Driver��ʼ��
void LED_5050_Data2PWMTime(void);//��5050�������ݰ�λ����ΪPWM������
void LED_5050SentStar(void);//LED_5050 ���ݷ���


void LED_Data_Fill_All(unsigned char G,unsigned char R,unsigned char B);//��ɫ���
void LED_Data_Fill(LED_FlashModeEnum mode,unsigned char* s,unsigned char l);//������ɫ���
void LED_Flash(void);
void LED_UART_Divice(unsigned char*s,short len);//LED���ڽ��ս���
#endif
