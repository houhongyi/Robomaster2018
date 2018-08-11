#ifndef __WIFI_H
#define __WIFI_H



typedef enum
{
	WifiCom_Ping,//ping命令
	WifiCom_PingASK,//Ping回复
	WifiCom_GarderModeCTR,//哨兵模式控制
	WifiCom_GarderRemoteCTR,//哨兵模拟遥控
	
	WifiCom_BaseCTR,//补给站控制
	WifiCom_All
}WifiCom_Def;

typedef enum
{
	Wifi_GarderMode_Cali,//校准模式
	Wifi_GarderMode_Cruise,//巡航
	Wifi_GarderMode_Attack,//攻击模式
	Wifi_GarderMode_Foolish,//定点攻击模式
	Wifi_GarderMode_Defense,//防御模式
	Wifi_GarderMode_Crazy,//暴走模式
	
	Wifi_GarderMode_All,
	
	Wifi_BaseMode_Normal,//补给站
	Wifi_BaseMode_GetBuletFormEng_Wait,//补给站获取模式 等待
	Wifi_BaseMode_GetBuletFormEng,//补给站获取模式
	Wifi_BaseMode_GiveBuletToHero_Wait,//补给站给英雄加弹 等待
	Wifi_BaseMode_GiveBuletToHero,//补给站给英雄加弹
}WifiMode_Def;//哨兵运行模式


typedef enum
{
	Wifi_DataFail,//Wifi数据格式错误
	Wifi_CheckFail,//Wifi数据校验失败
	Wifi_Success,//Wifi数据接收成功 Wifi结构体已更新
	Wifi_LowPriority,//当前wifi数据没有执行权
	Wifi_PingCom,//Wifi Ping 命令
	
	Wifi_State_All
}Wifi_State;

typedef enum
{
	WifiState_AUV=1,//无人机
	WifiState_Gaurder=2,//哨兵
	WifiStation_Base=3,//补给站
	
	WifiStation_All
}WifiStationDef;

typedef struct{
	unsigned short ID;//发送序号
	short ch1;
	short ch2;
	short ch3;
	short ch4;
	char Mode;
	char Com;
	char Msg[10];
}WifiDataDef;

typedef struct{
	WifiStationDef RxAdr;
	WifiStationDef TxAdr;
	float Rx_fps;
	float Tx_fps;
	WifiDataDef Data;
}WifiDef;

extern WifiDef Wifi;	

void WifiSent(unsigned char*s ,WifiDef* wifi,unsigned short ID);//传入需要填充的字符串首地址(至少为100字节的空间); 需要发送的wifi结构体
Wifi_State WifiDataRec(unsigned char*s,WifiDef* wifi);//在串口接收中调用 返回1为校验通过 返回0为失败

void Wifi_Ping(WifiStationDef device_tx, WifiStationDef device_rx);//Ping命令
void Wifi_PingASK(WifiStationDef device_tx,WifiStationDef device_rx,unsigned short ID);//PingASK回复


void UAVTOGaurd_ModeCTR(WifiMode_Def Mode);//无人机控制哨兵 模式控制
void UAVTOGaurd_RemoteCTR(short ch1,short ch2,short ch3,short ch4);//无人机控制哨兵 模拟控制

void UAVTOBASE_ModeCTR(WifiMode_Def Mode);//无人机控制补给站 模式控制
#endif
