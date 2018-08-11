
#include "Remote.h"

//======= other Lib
#include "SystemState.h"

RC_TypeDef RC_Data={0},Last_RC_Data={0};

/**
  * @brief  处理接收到的遥控数据
  * @param  RC_Type* rc 经过简单处理后的遥控器鼠标键盘数据
  * @param  uint8_t* buff 缓冲区中的数据
  * @retval None
  */

void GetRemoteCtrMsg(RC_TypeDef* rc, uint8_t* buff)
{
	//上次键盘赋值 用于消除抖动
	Last_RC_Data=*rc;
	
	//加上错误判断
	uint8_t error=(buff[5] >> 4)& 0x0003 ;
	if(error<1 || error>3)
	{}
	else
	{
	rc->ch1 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	rc->ch4 -= 1024;
	
	rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =  (buff[5] >> 4)& 0x0003 ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	rc->mouse.press_left 	= buff[12];
	rc->mouse.press_right = buff[13];
	
	rc->keyBoard.key_code = buff[14] | buff[15] << 8; //key borad code
		
	RefreshDeviceOutLineTime(RemoteCtr_No);
   }
}

/**
  * @brief  按键跳变检测
  * @param  当前系统时间
  * @retval void
  */
void DBUS_ButtonCheckJump(RC_TypeDef* rc,RC_TypeDef* lastrc)
{
		#define ClearShackTick 50 //这里看看防止的位置
    uint8_t index;
    uint32_t CurrentTick=HAL_GetTick();
    //上次对应按键被按下时的系统时间
    static portTickType LastButtonJumpTick[15] = {0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0};
    static portTickType LastMouseButtonJumpTick[2] = {0, 0};
    
    for(index = 0; index < 15; index++)
    {   //消除抖动50ms
        if(CurrentTick - LastButtonJumpTick[index] >= ClearShackTick)
        {
            if((rc->keyBoard.key_code & (1 << index)) && (!(lastrc->keyBoard.key_code & (1 << index))))
            {
                rc->keyBoard.jumpkey_code |= 1 << index;
                LastButtonJumpTick[index] = CurrentTick;
            }
        }
        else
        {
            rc->keyBoard.jumpkey_code &= ~(1 << index);
        }
    }
    
    if(CurrentTick - LastMouseButtonJumpTick[0] >= ClearShackTick)
    {
        if((rc->mouse.press_left) && (!(lastrc->mouse.press_left)))
        {
            rc->mouse.jumppress_left = 1;
            LastMouseButtonJumpTick[0] = CurrentTick;
        }
        else
        {
            rc->mouse.jumppress_left = 0;
        }
    }
    else
    {
        rc->mouse.jumppress_left = 0;
    }
    
    if(CurrentTick - LastMouseButtonJumpTick[1] >= ClearShackTick)
    {
        if((rc->mouse.press_right) && (!(lastrc->mouse.press_right)))
        {
            rc->mouse.jumppress_right = 1;
            LastMouseButtonJumpTick[0] = CurrentTick;
        }
        else
        {
            rc->mouse.jumppress_right = 0;
        }
    }
    else
    {
        rc->mouse.jumppress_right = 0;
    }
}
/**
  * @brief  判断一个键是否被按下
  * @param  要进行判断的按键，字符大写
  * @retval 1 按下        0 未按下
  */
uint8_t DBUS_CheckPush(RC_TypeDef* rc,uint16_t Key)
{
    if(rc->keyBoard.key_code & Key)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief  判断一个键状态
  * @param  本次状态码 上次状态码 要进行判断的按键
  * @retval 返回KeyBoardStateDef
  */

KeyBoardStateDef DBUS_CheckButtonState(uint16_t Keycode,uint16_t Keycode_Pre,uint16_t Key)
{
	 if(Keycode & Key)//本次按下
	 {
			if(Keycode_Pre & Key)//如果上次被按下
				return Key_Down;
			else //上次未被按下
				return Key_Fall;
	 }
	 else//本次没有被按下
	 {
			if(Keycode_Pre & Key)//如果上次被按下
				return Key_Raise;
			else //上次未被按下
				return Key_Up;
	 }
}

/**
  * @brief  判断一个键盘键是否发生按下跳变
  * @param  要进行判断的按键，字符大写
  * @retval 1 按下        0 未按下
  */
uint8_t DBUS_CheckJumpKey(RC_TypeDef* rc,uint16_t Key)
{
    if(rc->keyBoard.jumpkey_code & Key)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief  判断一个鼠标键是否发生按下跳变
  * @param  要进行判断的按键，字符大写
  * @retval 1 按下        0 未按下
  */
uint8_t DBUS_CheckJumpMouse(RC_TypeDef* rc,uint8_t Key)
{
    if(Key)
    {
        return rc->mouse.jumppress_left;
    }
    else
    {
        return rc->mouse.jumppress_right;
    }
}


