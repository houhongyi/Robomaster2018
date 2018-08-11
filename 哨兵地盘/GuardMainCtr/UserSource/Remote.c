
#include "Remote.h"

//======= other Lib
#include "SystemState.h"

RC_TypeDef RC_Data={0},Last_RC_Data={0};

/**
  * @brief  ������յ���ң������
  * @param  RC_Type* rc �����򵥴�����ң��������������
  * @param  uint8_t* buff �������е�����
  * @retval None
  */

void GetRemoteCtrMsg(RC_TypeDef* rc, uint8_t* buff)
{
	//�ϴμ��̸�ֵ ������������
	Last_RC_Data=*rc;
	
	//���ϴ����ж�
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
  * @brief  ����������
  * @param  ��ǰϵͳʱ��
  * @retval void
  */
void DBUS_ButtonCheckJump(RC_TypeDef* rc,RC_TypeDef* lastrc)
{
		#define ClearShackTick 50 //���￴����ֹ��λ��
    uint8_t index;
    uint32_t CurrentTick=HAL_GetTick();
    //�ϴζ�Ӧ����������ʱ��ϵͳʱ��
    static portTickType LastButtonJumpTick[15] = {0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0};
    static portTickType LastMouseButtonJumpTick[2] = {0, 0};
    
    for(index = 0; index < 15; index++)
    {   //��������50ms
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
  * @brief  �ж�һ�����Ƿ񱻰���
  * @param  Ҫ�����жϵİ������ַ���д
  * @retval 1 ����        0 δ����
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
  * @brief  �ж�һ����״̬
  * @param  ����״̬�� �ϴ�״̬�� Ҫ�����жϵİ���
  * @retval ����KeyBoardStateDef
  */

KeyBoardStateDef DBUS_CheckButtonState(uint16_t Keycode,uint16_t Keycode_Pre,uint16_t Key)
{
	 if(Keycode & Key)//���ΰ���
	 {
			if(Keycode_Pre & Key)//����ϴα�����
				return Key_Down;
			else //�ϴ�δ������
				return Key_Fall;
	 }
	 else//����û�б�����
	 {
			if(Keycode_Pre & Key)//����ϴα�����
				return Key_Raise;
			else //�ϴ�δ������
				return Key_Up;
	 }
}

/**
  * @brief  �ж�һ�����̼��Ƿ�����������
  * @param  Ҫ�����жϵİ������ַ���д
  * @retval 1 ����        0 δ����
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
  * @brief  �ж�һ�������Ƿ�����������
  * @param  Ҫ�����жϵİ������ַ���д
  * @retval 1 ����        0 δ����
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


//======================================�ڱ��� can����
#ifndef __REMOTE_H
#define __REMOTE_H
typedef struct
{
	int16_t ch1;
	int16_t ch2;
	uint8_t switch_left;
	uint8_t switch_right;
	unsigned char Bullet_Remain;
	unsigned char State;
}RC_Data_Short;
#endif

/**
  * @brief  can ����ң������
  * @param  
  * @retval 
  */
void RemoteMsg_CanSent(unsigned char*s,RC_Data_Short* temRC_Data)
{
	s[0]=temRC_Data->ch1>>8;//ch1���ֽ�
	s[1]=temRC_Data->ch1&0xff;//ch1���ֽ�
	s[2]=temRC_Data->ch2>>8;//ch2���ֽ�
	s[3]=temRC_Data->ch2&0xff;//ch2���ֽ�
	s[4]=temRC_Data->switch_right;//��߰��� 0~1
	s[4]=s[4]|(temRC_Data->switch_left<<2);//�ұ߰��� 2~3
	s[5]=temRC_Data->State;//״̬
	s[6]=temRC_Data->Bullet_Remain;//���ܴ򼸷��ӵ�
	s[7]=(((short)s[0])+s[1]+s[2]+s[3]+s[4]+s[5]+s[6]) & 0xFF; //�Ӻ�У��
}

RC_Data_Short test_Data_Short={0};
//can���ս��� �ɹ�����1 ʧ�ܷ���0
char RemoteMsg_CanRec(unsigned char*s,RC_Data_Short* temRC_Data)
{
	if(s[7] == (((short)s[0])+s[1]+s[2]+s[3]+s[4]+s[5]+s[6] & 0xFF) )
	{
		temRC_Data->ch1=((short)s[0])<<8 | s[1];
		temRC_Data->ch2=((short)s[2])<<8 | s[3];
		temRC_Data->switch_right=s[4]&0x03;
		temRC_Data->switch_left=s[4]>>2&0x03;
		temRC_Data->State=s[5];
		temRC_Data->Bullet_Remain=s[6];
		return 1;
	}
	else
		return 0;
}
