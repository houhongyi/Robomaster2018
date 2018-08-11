#include "PN532.h"
#include <string.h>

//==========other Lib
#include "MyCom.h"
#include "SystemState.h"

PN532ReaderDef PN532ReaderL={0},PN532ReaderR={0};

//==========================================
//
//					�ⲿʵ�ֵĺ���
//
//==========================================

//PN532 UART���ͺ�˳
__weak void PN532_UART_Send(PN532ReaderDef *hPN532,unsigned char* d,short len)
{
	(void)d;
}

//==========================================
//
//					�ڲ�����
//
//==========================================


//��ͷ���У��
unsigned char NegCodeSumCheck(unsigned char*d,unsigned char len)
{
	unsigned char sum=0;
	char i=0;
	for(i=0;i<len;i++)
	{
		sum+=*(d+i);
	}
	return 0-sum;
}
	
//PN532���ݷ���
void PN532_Analyse(PN532ReaderDef *hPN532,unsigned char* data,short len)
{
	if(*(data+PN532_Data_COM)==0x15)
			hPN532->State=PN532_State_Weak;
	
	if(*(data+PN532_Data_COM)==0x4B)//Ѱ���ɹ�
	{
		hPN532->UID_Len=*(data+PN532_Data_CardUID_n);//��ȡUIDλ��
		memcpy(hPN532->UID,data+PN532_Data_CardUID_n+1,hPN532->UID_Len);//��ȡUID
		
		
		
		#ifdef __SysState_H__
		if(hPN532==&PN532ReaderL)//������Ƭ
			RefreshDeviceOutLineTime(NFC_Reader_LCard);
		if(hPN532==&PN532ReaderR)
			RefreshDeviceOutLineTime(NFC_Reader_RCard);
		#endif
	}
}


//==========================================
//
//					�ⲿ���ú���
//
//==========================================
void PN532_Commen_Sent(PN532ReaderDef *hPN532,PN532_ComEnum Com)
{
	unsigned char Comdata[50]={0};
	switch((unsigned short)Com)
	{
		case PN532_COM_WEAKUP://���Ѻ���
				memcpy(Comdata,"\x55\x55\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff\x03\xfd\xd4\x14\x01\x17\x00",24);
				PN532_UART_Send(hPN532,Comdata,24);
				hPN532->State=PN532_State_Weak;
			break;
		case PN532_COM_FindCard://Ѱ������
				memcpy(Comdata,"\x00\x00\xff\x04\xfc\xd4\x4a\x01\x00\xe1\x00",11);
				PN532_UART_Send(hPN532,Comdata,11);
				hPN532->State=PN532_State_Finding;
			break;
	}
}

//PN532���ݷָ�
void PN532_DataDevice(PN532ReaderDef *hPN532,unsigned char* data,short len)
{
	unsigned char*p=data;
	unsigned char fram_len;
	while( (p-data) <len)
	{
		if(*p==0x00 && *(p+1)==0x00 && *(p+2)==0xff && *(p+3)==0x00 && *(p+4)==0xff && *(p+5)==0x00) //֡ͷ
		{
#ifdef __SysState_H__
			if(hPN532==&PN532ReaderL)
				RefreshDeviceOutLineTime(NFC_Reader_L);
			if(hPN532==&PN532ReaderR)
				RefreshDeviceOutLineTime(NFC_Reader_R);
#endif
			p+=6-1;
		}
		else if(*p==0x00 && *(p+1)==0x00 && *(p+2)==0xff) 
		{
			fram_len=*(p+3);
			if(*(p+5+fram_len)==NegCodeSumCheck(p+5,fram_len))//
			{
				PN532_Analyse(hPN532,p,fram_len+7);
				p+=fram_len+7-1;
			}
		}
		p++;
	}
}

