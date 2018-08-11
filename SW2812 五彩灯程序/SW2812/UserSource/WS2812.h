#ifndef __WS2812_H
#define __WS2812_H

#include "stm32f1xx_hal.h"
#include "tim.h"

#define LED_5050_n 8	//LED灯个数
#define LED_5050_Data_n        LED_5050_n*3  //LED灯数据长度
#define LED_5050_Data_Bit_n    LED_5050_n*3*8  //LED灯数据长度位数

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
	short Bit_n;//发送第几位
	unsigned char Data[LED_5050_Data_n];//亮度数据 // 顺序为GRB
	unsigned char Data_Breath[LED_5050_Data_n];//亮度数据(呼吸用) // 顺序为GRB
	unsigned short Data_Pulse[LED_5050_Data_Bit_n+1];//亮度数据脉宽长度
	TIM_HandleTypeDef* htim;//PWMTimer
}LED_5050DriverDef;


extern LED_5050DriverDef LED_5050Driver;

char LED_5050DriverInite(void);//LED5050Driver初始化
void LED_5050_Data2PWMTime(void);//将5050亮度数据按位翻译为PWM脉宽长度
void LED_5050SentStar(void);//LED_5050 数据发送


void LED_Data_Fill_All(unsigned char G,unsigned char R,unsigned char B);//颜色填充
void LED_Data_Fill(LED_FlashModeEnum mode,unsigned char* s,unsigned char l);//串口颜色填充
void LED_Flash(void);
void LED_UART_Divice(unsigned char*s,short len);//LED串口接收解析
#endif
