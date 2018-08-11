#ifndef __REMOTE_H
#define __REMOTE_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"

#define KEY_V		0x4000
#define KEY_C		0x2000
#define KEY_X		0x1000
#define KEY_Z		0x0800
#define KEY_G		0x0400
#define KEY_F		0x0200
#define KEY_R		0x0100
#define KEY_E		0x0080
#define KEY_Q		0x0040
#define KEY_CTRL	0x0020
#define KEY_SHIFT	0x0010
#define KEY_D		0x0008
#define KEY_A		0x0004
#define KEY_S		0x0002
#define KEY_W		0x0001
 
/** 
  * @brief  遥控数据结构体
  */
typedef struct{
	//遥控器通道
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	uint8_t switch_left;
	uint8_t switch_right;
	//鼠标
	struct{
		int16_t x;
		int16_t y;
		int16_t z;
		
		uint8_t press_left;
		uint8_t press_right;
		uint8_t jumppress_left;
		uint8_t jumppress_right;
	}mouse;
	struct 
	{
		uint16_t key_code;              //原始键值
    uint16_t jumpkey_code;          //跳变后的键值
	}keyBoard;
/*************************************************************************************
   * 键盘通道:15   14   13   12   11   10    9   8    7    6     5    4    3    2    1
   *          V    C    X	   Z    G    F    R    E   Q   CTRL  SHIFT  D    A    S    W
**************************************************************************************/
}RC_TypeDef;


typedef enum
{
	Key_Down,		//按下
	Key_Raise,	//上升
	Key_Up,			//未按下
	Key_Fall,		//下降
}KeyBoardStateDef; //按键状态


typedef struct
{
	int16_t ch1;
	int16_t ch2;
	uint8_t switch_left;
	uint8_t switch_right;
	unsigned char Bullet_Remain;
	unsigned char State;
}RC_Data_Short;

extern RC_TypeDef RC_Data,Last_RC_Data;
extern RC_Data_Short test_Data_Short;

void GetRemoteCtrMsg(RC_TypeDef* rc, uint8_t* buff);//获取遥控器量
void RemoteMsg_CanSent(unsigned char*s,RC_Data_Short* temRC_Data);//can发送遥控器量
char RemoteMsg_CanRec(unsigned char*s,RC_Data_Short* temRC_Data);
KeyBoardStateDef DBUS_CheckButtonState(uint16_t Keycode,uint16_t Keycode_Pre,uint16_t Key);
#endif

