#include "Wifi.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

//======= other Lib
#include "MyCom.h"

#ifndef __MyCom_H

	#define my_abs(x) ((x)>0?(x):-(x)) //ABS�궨��
	#define MyFlagSet(x,y) x=x|(0x00000001<<y) //���ñ�־λ  y�ڼ�λ
	#define MyFlagClear(x,y) x=x&~(0x00000001<<y)
	#define MyFlagGet(x,y) (x&(0x00000001<<y))
	

	//�ж��Ƿ�������  ��������1 ��������0
	int IsPositive(float x)
	{
		if(x>=0)return 1;
		else return -1;
	}

	//�ж��ַ������Ƿ����ģ���ַ�
	int IsStrInc(char* data,char* temp)
	{
		int Data_len=strlen(data);
		int Tem_len=strlen(temp);
		int i=0;int j=0;
		for(i=0;i<Data_len;i++)
		{
			if(data[i]==temp[0])
			{
				for(j=0;j<Tem_len;j++)
				{
					if(data[i+j]!=temp[j])break;
					if(j==Tem_len-1)return 1;//����
				}
			}
		}
		return 0;
	}

	//�ַ���תfloat
	float str2f(unsigned char *s)
	{
			float num=0;
			unsigned char *p;
			char Point_flag=0;//С�����־
		  float Positive=1.0f;//����λ
			while(1)
			{
					if(*s=='-')//�жϷ���
					{
						Positive=-1;
						s++;
						continue;
					}
					else
						{
						if((*s>='0' && *s<='9')||(*s=='.'))
						{
							if(*s!='.')
							{
								 if(!Point_flag)//δ�ҵ�С����
									num=num*10+(*s-'0');
									else//�ҵ�С����
									{
										num+=((float)(*s-'0'))/powf(10,(s-p));
									}
							}
							else 
							{
								Point_flag=1;//�ҵ�С����
								p=s;//��¼С�����λ��
							}
								
							s++;
						}
						else break;
					}
			}
			return Positive*num;
	}

	//��ȡ�м������
	float Getnum(unsigned char *s)
	{
		unsigned char sNum[10]={0};
		unsigned char*p=sNum;
		while(*s)
		{
				if(*s==0xff)break;
				if(((*s)>='0' &&(*s)<='9')||(*s)=='.')
				{
					*p=*s;
					p++;
				}
				s++;
		}
		return str2f(sNum);
	}

#endif

	


WifiDef Wifi={0};	
float Wifi_CTR_time[WifiStation_All]={0};
unsigned char Wifi_DataToSent[100]={0};
//=====================================================
//							  �ⲿ ��Ҫʵ�ֵĺ���
//
//=====================================================
//WIFI���ڷ��ͺ���
__weak void Wifi_UARTSent(unsigned char*s,short len)
{
	(void)s;
}

//��ȡ��ǰϵͳʱ�� ��λ��mm��
__weak float GetSystemTime()
{
	return 0;
}
//=====================================================
//							  �ڲ�����
//
//=====================================================

//wifidata У�����  ��ǰ�����д������ݼӺ����ַ����ļӺ� ��ID��ȡ�ࣩ
short WifidataCheck(WifiDataDef* wifidata)
{
	return (wifidata->ch1+wifidata->ch2+wifidata->ch3+wifidata->ch4+wifidata->Mode+wifidata->Com) - wifidata->ID;
}

void WifiDataTrans(unsigned char*s ,WifiDataDef* wifidata)
{
	//ID,CH1,CH2,CH3,CH4,Mode,Com,MSG,<CheckNum>
	short checkNum=WifidataCheck(wifidata);
	sprintf((char*)s,"%d,%d,%d,%d,%d,%d,%d,%s,%d",
	wifidata->ID,wifidata->ch1,wifidata->ch2,wifidata->ch3,wifidata->ch4,
	wifidata->Mode,wifidata->Com,wifidata->Msg,checkNum);
}

//�豸��Ȩ
//���� device(�豸����)  timenow(��ǰʱ��)
//���� 1��Ȩ�ɹ� 0��Ȩʧ��
char Equipment_authentication(WifiStationDef device,float timenow)
{
	int i=0;
	float time_nearst=0;
	
	if(device==0)return 1;//��������豸��Ϊ0 �����м�Ȩ
	
	for(i=WifiState_AUV;i<device;i++)
	{
		if(Wifi_CTR_time[i]>time_nearst)
			time_nearst=Wifi_CTR_time[i];//�ҵ����ȼ�����device�豸�����ͨ��ʱ��
	}
	
	if(timenow==0)//��ǰ����ʱ��Ϊ0 �����м�Ȩ
		return 1;
	
	if (timenow-time_nearst<100)//�����ȼ����豸�����100ms����ͨ��ʱ ��Ȩʧ��
		return 0;
	else
		return 1;
}

//=====================================================
//							  �ⲿ����
//
//=====================================================
unsigned short Wifi_SENTID=0;
//Ping����  
void Wifi_Ping(WifiStationDef device_tx,WifiStationDef device_rx)
{
	
	WifiDef temWifiTx={0};
	
	temWifiTx.Data.ID=Wifi_SENTID;
	temWifiTx.TxAdr=device_tx;
	temWifiTx.RxAdr=device_rx;
	temWifiTx.Data.Com=WifiCom_Ping;
	memset(Wifi_DataToSent,0,100);
	WifiSent(Wifi_DataToSent ,&temWifiTx,0);
}

//PingASK�ظ� 
void Wifi_PingASK(WifiStationDef device_tx,WifiStationDef device_rx,unsigned short ID)
{
	WifiDef temWifiTx={0};
	temWifiTx.TxAdr=device_tx;
	temWifiTx.RxAdr=device_rx;
	temWifiTx.Data.Com=WifiCom_PingASK;
	memset(Wifi_DataToSent,0,100);
	WifiSent(Wifi_DataToSent ,&temWifiTx,ID);
}



//������Ҫ�����ַ����׵�ַ(����Ϊ70�ֽڵĿռ�) �� ��Ҫ���͵�wifi�ṹ��
void WifiSent(unsigned char*s ,WifiDef* wifi,unsigned short ID)
{
	unsigned char data[100]={0};
	
	if(!ID)
	{
		Wifi_SENTID++;
		wifi->Data.ID=Wifi_SENTID;
	}
	else
		wifi->Data.ID=ID;
	
	WifiDataTrans(data ,&wifi->Data);
	sprintf((char*)s,"T:%02dR:%02d:%s,\r\n",wifi->TxAdr,wifi->RxAdr,data);
	Wifi_UARTSent(s,strlen((char*)s));//Wifi����
}	

//�ڴ��ڽ����е��� 
//	����1ΪУ��ͨ������ֱ��ʹ��Wifi�ṹ���е����� 
//	����0Ϊʧ��û�и���WiFi�ṹ��
Wifi_State WifiDataRec(unsigned char*s,WifiDef* wifi)
{
	static float pre_time=0;
	float time=0;
	
	WifiDef tem_wifi={0};
	short n=0;
	char dot_n=0;
	unsigned char str_ID[6]={0};
	unsigned char str_ch1[7]={0};
	unsigned char str_ch2[7]={0};
	unsigned char str_ch3[7]={0};
	unsigned char str_ch4[7]={0};
	unsigned char str_Mode[5]={0};
	unsigned char str_Com[5]={0};
	unsigned char str_Msg[10]={0};
	unsigned char str_checkNum[7]={0};
	unsigned char *q,*p;
	short checkNum=0;
	
	
	for(n=0;n<100;n++)//�ҵ� T ��λ��
	{
		if(*(s+n)==0&&*(s+n+1)==0)return Wifi_DataFail; //������������� \0\0��ֱ���˳�����ʧ��
		if(*(s+n)=='T' && *(s+n+4)=='R')// �ҵ�֡ͷ T��R
		{
			s+=n;// �ƶ�֡ͷ�� 'T' ��λ��
			break;
		}
	}
	tem_wifi.TxAdr=(WifiStationDef)((s[2]-'0')*10+s[3]-'0');
	tem_wifi.RxAdr=(WifiStationDef)((s[6]-'0')*10+s[7]-'0');
	s+=9;//�ƶ�֡ͷ��������ʼλ��
	p=s;
	for(n=0;n<100;n++)
	{
		if(*s==',')
		{
			q=s;
			switch(dot_n)
			{
				case 0:
					memcpy(str_ID,p,q-p);
					break;
				case 1:
					memcpy(str_ch1,p,q-p);
					break;
				case 2:
					memcpy(str_ch2,p,q-p);
					break;
				case 3:
					memcpy(str_ch3,p,q-p);
					break;
				case 4:
					memcpy(str_ch4,p,q-p);
					break;
				case 5:
					memcpy(str_Mode,p,q-p);
					break;
				case 6:
					memcpy(str_Com,p,q-p);
					break;
				case 7:
					memcpy(str_Msg,p,q-p);
					break;
				case 8:
					memcpy(str_checkNum,p,q-p);
					break;
					
			}
			dot_n++;//  ����ָʾ��һ
			p=s+1;
		}
		s++;
		if(*s=='\r' && *(s+1)=='\n')break;//�ҵ�֡β
		if(*s==0 && *(s+1)==0)return Wifi_DataFail;//���ݽṹ����
	}
	
	//����ת��
	tem_wifi.Data.ID=str2f(str_ID);
	tem_wifi.Data.ch1=str2f(str_ch1);
	tem_wifi.Data.ch2=str2f(str_ch2);
	tem_wifi.Data.ch3=str2f(str_ch3);
	tem_wifi.Data.ch4=str2f(str_ch4);
	tem_wifi.Data.Mode=str2f(str_Mode);
	tem_wifi.Data.Com=str2f(str_Com);
	sprintf(tem_wifi.Data.Msg,"%s",str_Msg);
	checkNum=str2f(str_checkNum);
	
	//����У��
	if(WifidataCheck(&tem_wifi.Data)==checkNum)
	{
		time=GetSystemTime();//��õ�ǰϵͳʱ��
		tem_wifi.Rx_fps=1000/(time-pre_time);//����ͨ��֡��
		pre_time=time;
		
		if(tem_wifi.Data.Com==WifiCom_Ping)//PING���� ����¼ͨѶʱ��
		{
			memcpy(wifi,&tem_wifi,sizeof(WifiDef));
			return Wifi_PingCom;
		}
		
		if(Equipment_authentication((WifiStationDef)tem_wifi.TxAdr,time))//��Ȩ
		{
			Wifi_CTR_time[tem_wifi.TxAdr]=time;//��¼���ε�ͨ��ʱ��
			memcpy(wifi,&tem_wifi,sizeof(WifiDef));
			return Wifi_Success;
		}
		else
			return Wifi_LowPriority;
	}
	else
		return Wifi_CheckFail;
}


//���˻������ڱ� ģʽ����
void UAVTOGaurd_ModeCTR(WifiMode_Def Mode)
{
	WifiDef temWifiTx={0};
	temWifiTx.TxAdr=WifiState_AUV;
	temWifiTx.RxAdr=WifiState_Gaurder;
	temWifiTx.Data.Com=WifiCom_GarderModeCTR;//����Ϊ �ڱ�ģʽ����
	temWifiTx.Data.Mode=Mode;//ģʽΪ����ģʽ
	memset(Wifi_DataToSent,0,100);
	WifiSent(Wifi_DataToSent ,&temWifiTx,0);//����
}

//���˻������ڱ� ģ�����
void UAVTOGaurd_RemoteCTR(short ch1,short ch2,short ch3,short ch4)
{
	WifiDef temWifiTx={0};
	temWifiTx.TxAdr=WifiState_AUV;
	temWifiTx.RxAdr=WifiState_Gaurder;
	temWifiTx.Data.Com=WifiCom_GarderRemoteCTR;//����Ϊ �ڱ�ģ�����
	temWifiTx.Data.ch1=ch1;
	temWifiTx.Data.ch2=ch2;
	temWifiTx.Data.ch3=ch3;
	temWifiTx.Data.ch4=ch4;
	memset(Wifi_DataToSent,0,100);
	WifiSent(Wifi_DataToSent ,&temWifiTx,0);//����
}

//���˻����Ʋ���վ ģʽ����
void UAVTOBASE_ModeCTR(WifiMode_Def Mode)
{

	WifiDef temWifiTx={0};
	temWifiTx.TxAdr=WifiState_AUV;
	temWifiTx.RxAdr=WifiStation_Base;
	temWifiTx.Data.Com=WifiCom_BaseCTR;//����Ϊ �ڱ�ģʽ����
	temWifiTx.Data.Mode=Mode;//ģʽΪ����ģʽ
	memset(Wifi_DataToSent,0,100);
	WifiSent(Wifi_DataToSent ,&temWifiTx,0);//����
}

