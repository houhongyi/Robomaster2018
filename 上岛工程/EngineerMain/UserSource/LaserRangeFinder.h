#ifndef __LaserRF_H__
#define __LaserRF_H__


#define str_LRF_SetRange_5M "\xFA\x04\x09\x05\xF4" //设置量程5m
#define str_LRF_SetRange_10M "\xFA\x04\x09\x0A\xEF" //设置量程10m
#define str_LRF_SetRange_30M "\xFA\x04\x09\x1E\xDB" //设置量程30m

#define str_LRF_SetRevolution_1mm "\xFA\x04\x0C\x01\xF5" //设置分辨率 1mm
#define str_LRF_SetRevolution_01mm "\xFA\x04\x0C\x02\xF4" //设置分辨率0.1mm

#define str_LRF_SetFrequence_5HZ "\xFA\x04\x0A\x05\xF3"  //设置频率
#define str_LRF_SetFrequence_10HZ "\xFA\x04\x0A\x0A\xEE" 
#define str_LRF_SetFrequence_20HZ "\xFA\x04\x0A\x14\xE4"

#define str_LRF_SetPoweONStart2Mesure "\xFA\x04\x0D\x01\xF4" //设定上电就测
#define str_LRF_SetPoweONStart2Mesure_NOT "\xFA\x04\x0D\x00\xF5" //设定上电就测

#define str_LRF_StarMesure_Once "\x80\x06\x02\x78" //单次测量
#define str_LRF_StarMesure "\x80\x06\x03\x77" //连续测量
#define str_LRF_PowerOFF "\x80\x04\x02\x7A" //关机命令


typedef enum
{
	LRF_DateCheck_Fail=0,
	LRF_DateCheck_OK,
}Date_CheckDef;


typedef enum
{
	LRF_Inite=0,//初始值
	LRF_Ready,//就绪状态
	LRF_BUSY, //忙
	
	LRF_SetRange_OK,//设置量程成功
	LRF_SetRange_Fail,
	
	LRF_SetFrequency_OK,//设置频率成功
	LRF_SetFrequency_Fail,
	
	LRF_SetRevolution_OK,//设置分辨率成功
	LRF_SetRevolution_Fail,
	
	LRF_Mesure_OK,//测量完成
	LRF_DataCheck_Fail,//测量数据校验失败
	
	LRF_SetOff_OK,//设置关机成功
	LRF_SetOff_Fail,
}LaserRFStateDef;

typedef struct 
{
	LaserRFStateDef State;
	float Dist;
}LaserRFDef;

extern LaserRFDef LaserRF;

char LaserRFInite(void);//初始化
void StartTOMeasure(void);//开始测量
void StopMeasure(void);//停止测量

LaserRFStateDef LRF_Analy(unsigned char*s);//数据解析  只写了连续测量获取

#endif
