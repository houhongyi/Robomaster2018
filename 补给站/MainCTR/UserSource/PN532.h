#ifndef __PN532_H__
#define __PN532_H__

typedef enum
{
	PN532_Data_Dir=5,//����
	PN532_Data_COM=6,//��Ӧ����
	PN532_Data_TargetCard=7,//Ŀ�꿨
	PN532_Data_NumCard=8,//������
	PN532_Data_CardUID_n=12,//UIDλ��
	
}PN532_DataAnly;


typedef enum
{
	PN532_COM_WEAKUP,
	PN532_COM_FindCard,
	PN532_COM_ReadUID,
}PN532_ComEnum;

typedef enum
{
	PN532_State_Outline,
	PN532_State_Sleep,
	PN532_State_Weak,
	PN532_State_Finding,
	PN532_State_Finded,
	PN532_State_UIDReading,
	PN532_State_UIDReaded,
	PN532_State_Reading,
	PN532_State_Erro,
}PN532_StateEnum;

typedef struct
{
	PN532_StateEnum State;
	unsigned char UID[16];
	unsigned char UID_Len;
}PN532ReaderDef;


extern PN532ReaderDef PN532ReaderL,PN532ReaderR;

void PN532_Commen_Sent(PN532ReaderDef *hPN532,PN532_ComEnum Com);//PN532�����
void PN532_DataDevice(PN532ReaderDef *hPN532,unsigned char* data,short len);//PN532���ݷָ�
#endif
