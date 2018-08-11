#ifndef __WIFI_H
#define __WIFI_H



typedef enum
{
	WifiCom_Ping,//ping����
	WifiCom_PingASK,//Ping�ظ�
	WifiCom_GarderModeCTR,//�ڱ�ģʽ����
	WifiCom_GarderRemoteCTR,//�ڱ�ģ��ң��
	
	WifiCom_BaseCTR,//����վ����
	WifiCom_All
}WifiCom_Def;

typedef enum
{
	Wifi_GarderMode_Cali,//У׼ģʽ
	Wifi_GarderMode_Cruise,//Ѳ��
	Wifi_GarderMode_Attack,//����ģʽ
	Wifi_GarderMode_Foolish,//���㹥��ģʽ
	Wifi_GarderMode_Defense,//����ģʽ
	Wifi_GarderMode_Crazy,//����ģʽ
	
	Wifi_GarderMode_All,
	
	Wifi_BaseMode_Normal,//����վ
	Wifi_BaseMode_GetBuletFormEng_Wait,//����վ��ȡģʽ �ȴ�
	Wifi_BaseMode_GetBuletFormEng,//����վ��ȡģʽ
	Wifi_BaseMode_GiveBuletToHero_Wait,//����վ��Ӣ�ۼӵ� �ȴ�
	Wifi_BaseMode_GiveBuletToHero,//����վ��Ӣ�ۼӵ�
}WifiMode_Def;//�ڱ�����ģʽ


typedef enum
{
	Wifi_DataFail,//Wifi���ݸ�ʽ����
	Wifi_CheckFail,//Wifi����У��ʧ��
	Wifi_Success,//Wifi���ݽ��ճɹ� Wifi�ṹ���Ѹ���
	Wifi_LowPriority,//��ǰwifi����û��ִ��Ȩ
	Wifi_PingCom,//Wifi Ping ����
	
	Wifi_State_All
}Wifi_State;

typedef enum
{
	WifiState_AUV=1,//���˻�
	WifiState_Gaurder=2,//�ڱ�
	WifiStation_Base=3,//����վ
	
	WifiStation_All
}WifiStationDef;

typedef struct{
	unsigned short ID;//�������
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

void WifiSent(unsigned char*s ,WifiDef* wifi,unsigned short ID);//������Ҫ�����ַ����׵�ַ(����Ϊ100�ֽڵĿռ�); ��Ҫ���͵�wifi�ṹ��
Wifi_State WifiDataRec(unsigned char*s,WifiDef* wifi);//�ڴ��ڽ����е��� ����1ΪУ��ͨ�� ����0Ϊʧ��

void Wifi_Ping(WifiStationDef device_tx, WifiStationDef device_rx);//Ping����
void Wifi_PingASK(WifiStationDef device_tx,WifiStationDef device_rx,unsigned short ID);//PingASK�ظ�


void UAVTOGaurd_ModeCTR(WifiMode_Def Mode);//���˻������ڱ� ģʽ����
void UAVTOGaurd_RemoteCTR(short ch1,short ch2,short ch3,short ch4);//���˻������ڱ� ģ�����

void UAVTOBASE_ModeCTR(WifiMode_Def Mode);//���˻����Ʋ���վ ģʽ����
#endif
