#include "Wifi.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

//======= other Lib
#include "MyCom.h"

#ifndef __MyCom_H

	#define my_abs(x) ((x)>0?(x):-(x)) //ABS宏定义
	#define MyFlagSet(x,y) x=x|(0x00000001<<y) //设置标志位  y第几位
	#define MyFlagClear(x,y) x=x&~(0x00000001<<y)
	#define MyFlagGet(x,y) (x&(0x00000001<<y))
	

	//判断是否是正数  正数返回1 负数返回0
	int IsPositive(float x)
	{
		if(x>=0)return 1;
		else return -1;
	}

	//判断字符串中是否包含模板字符
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
					if(j==Tem_len-1)return 1;//包含
				}
			}
		}
		return 0;
	}

	//字符串转float
	float str2f(unsigned char *s)
	{
			float num=0;
			unsigned char *p;
			char Point_flag=0;//小数点标志
		  float Positive=1.0f;//符号位
			while(1)
			{
					if(*s=='-')//判断符号
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
								 if(!Point_flag)//未找到小数点
									num=num*10+(*s-'0');
									else//找到小数点
									{
										num+=((float)(*s-'0'))/powf(10,(s-p));
									}
							}
							else 
							{
								Point_flag=1;//找到小数点
								p=s;//记录小数点的位置
							}
								
							s++;
						}
						else break;
					}
			}
			return Positive*num;
	}

	//截取中间的数字
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
//							  外部 需要实现的函数
//
//=====================================================
//WIFI串口发送函数
__weak void Wifi_UARTSent(unsigned char*s,short len)
{
	(void)s;
}

//获取当前系统时间 单位（mm）
__weak float GetSystemTime()
{
	return 0;
}
//=====================================================
//							  内部函数
//
//=====================================================

//wifidata 校验计算  （前面所有传送数据加和与字符串的加和 对ID号取余）
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

//设备鉴权
//传入 device(设备代码)  timenow(当前时间)
//返回 1鉴权成功 0鉴权失败
char Equipment_authentication(WifiStationDef device,float timenow)
{
	int i=0;
	float time_nearst=0;
	
	if(device==0)return 1;//如果传入设备号为0 不进行鉴权
	
	for(i=WifiState_AUV;i<device;i++)
	{
		if(Wifi_CTR_time[i]>time_nearst)
			time_nearst=Wifi_CTR_time[i];//找到优先级高于device设备的最近通信时间
	}
	
	if(timenow==0)//当前传入时间为0 不进行鉴权
		return 1;
	
	if (timenow-time_nearst<100)//高优先级的设备在最近100ms内有通信时 鉴权失败
		return 0;
	else
		return 1;
}

//=====================================================
//							  外部调用
//
//=====================================================
unsigned short Wifi_SENTID=0;
//Ping命令  
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

//PingASK回复 
void Wifi_PingASK(WifiStationDef device_tx,WifiStationDef device_rx,unsigned short ID)
{
	WifiDef temWifiTx={0};
	temWifiTx.TxAdr=device_tx;
	temWifiTx.RxAdr=device_rx;
	temWifiTx.Data.Com=WifiCom_PingASK;
	memset(Wifi_DataToSent,0,100);
	WifiSent(Wifi_DataToSent ,&temWifiTx,ID);
}



//传入需要填充的字符串首地址(至少为70字节的空间) 和 需要发送的wifi结构体
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
	Wifi_UARTSent(s,strlen((char*)s));//Wifi发送
}	

//在串口接收中调用 
//	返回1为校验通过可以直接使用Wifi结构体中的数据 
//	返回0为失败没有更新WiFi结构体
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
	
	
	for(n=0;n<100;n++)//找到 T 的位置
	{
		if(*(s+n)==0&&*(s+n+1)==0)return Wifi_DataFail; //如果出现了两个 \0\0则直接退出返回失败
		if(*(s+n)=='T' && *(s+n+4)=='R')// 找到帧头 T与R
		{
			s+=n;// 移动帧头到 'T' 的位置
			break;
		}
	}
	tem_wifi.TxAdr=(WifiStationDef)((s[2]-'0')*10+s[3]-'0');
	tem_wifi.RxAdr=(WifiStationDef)((s[6]-'0')*10+s[7]-'0');
	s+=9;//移动帧头到数据起始位置
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
			dot_n++;//  逗号指示加一
			p=s+1;
		}
		s++;
		if(*s=='\r' && *(s+1)=='\n')break;//找到帧尾
		if(*s==0 && *(s+1)==0)return Wifi_DataFail;//数据结构错误
	}
	
	//数据转换
	tem_wifi.Data.ID=str2f(str_ID);
	tem_wifi.Data.ch1=str2f(str_ch1);
	tem_wifi.Data.ch2=str2f(str_ch2);
	tem_wifi.Data.ch3=str2f(str_ch3);
	tem_wifi.Data.ch4=str2f(str_ch4);
	tem_wifi.Data.Mode=str2f(str_Mode);
	tem_wifi.Data.Com=str2f(str_Com);
	sprintf(tem_wifi.Data.Msg,"%s",str_Msg);
	checkNum=str2f(str_checkNum);
	
	//数据校验
	if(WifidataCheck(&tem_wifi.Data)==checkNum)
	{
		time=GetSystemTime();//获得当前系统时间
		tem_wifi.Rx_fps=1000/(time-pre_time);//计算通信帧率
		pre_time=time;
		
		if(tem_wifi.Data.Com==WifiCom_Ping)//PING命令 不记录通讯时间
		{
			memcpy(wifi,&tem_wifi,sizeof(WifiDef));
			return Wifi_PingCom;
		}
		
		if(Equipment_authentication((WifiStationDef)tem_wifi.TxAdr,time))//鉴权
		{
			Wifi_CTR_time[tem_wifi.TxAdr]=time;//记录本次的通信时间
			memcpy(wifi,&tem_wifi,sizeof(WifiDef));
			return Wifi_Success;
		}
		else
			return Wifi_LowPriority;
	}
	else
		return Wifi_CheckFail;
}


//无人机控制哨兵 模式控制
void UAVTOGaurd_ModeCTR(WifiMode_Def Mode)
{
	WifiDef temWifiTx={0};
	temWifiTx.TxAdr=WifiState_AUV;
	temWifiTx.RxAdr=WifiState_Gaurder;
	temWifiTx.Data.Com=WifiCom_GarderModeCTR;//命令为 哨兵模式控制
	temWifiTx.Data.Mode=Mode;//模式为传入模式
	memset(Wifi_DataToSent,0,100);
	WifiSent(Wifi_DataToSent ,&temWifiTx,0);//发送
}

//无人机控制哨兵 模拟控制
void UAVTOGaurd_RemoteCTR(short ch1,short ch2,short ch3,short ch4)
{
	WifiDef temWifiTx={0};
	temWifiTx.TxAdr=WifiState_AUV;
	temWifiTx.RxAdr=WifiState_Gaurder;
	temWifiTx.Data.Com=WifiCom_GarderRemoteCTR;//命令为 哨兵模拟控制
	temWifiTx.Data.ch1=ch1;
	temWifiTx.Data.ch2=ch2;
	temWifiTx.Data.ch3=ch3;
	temWifiTx.Data.ch4=ch4;
	memset(Wifi_DataToSent,0,100);
	WifiSent(Wifi_DataToSent ,&temWifiTx,0);//发送
}

//无人机控制补给站 模式控制
void UAVTOBASE_ModeCTR(WifiMode_Def Mode)
{

	WifiDef temWifiTx={0};
	temWifiTx.TxAdr=WifiState_AUV;
	temWifiTx.RxAdr=WifiStation_Base;
	temWifiTx.Data.Com=WifiCom_BaseCTR;//命令为 哨兵模式控制
	temWifiTx.Data.Mode=Mode;//模式为传入模式
	memset(Wifi_DataToSent,0,100);
	WifiSent(Wifi_DataToSent ,&temWifiTx,0);//发送
}

