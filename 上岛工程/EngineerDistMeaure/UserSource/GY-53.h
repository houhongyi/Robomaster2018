#ifndef _GY_53_H
#define _GY_53_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

typedef enum
{
	GY_53SumCheckFail,//校验和检验失败
	GY_53FrameCheckFail,//帧头检验失败
	GY_53DataUpDate,//数据更新
}GY_53StateDef;

typedef enum
{
	GY_53Com_ContinuMer,//连续测量
	GY_53Com_FastMer,//快速测量模式
}GY_53ComDef;


typedef enum
{
	GY_53Genaral=0, //一般测量模式
	GY_53LongMeur, //长距离测量模式
	GY_53FastMeur, //快速测量模式
	GY_53HighQuanlityMeur, //高分辨率模式
}GY_53ModeDef;

typedef struct
{
	unsigned short Dis;
	GY_53ModeDef Mode;
	float fps;
	float lastComuTim;
}GY_53Def;

extern GY_53Def GY_53_1,GY_53_2,GY_53_3,GY_53_4,GY_53_5,GY_53_6;

GY_53StateDef GY_53Analy(GY_53Def* temGY_53,char* c);
void GY_53Com(GY_53Def* temGY_53,GY_53ComDef Com);
char GY_53Inite(void);

#endif
