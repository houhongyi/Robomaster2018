#include <string.h>
#include "cmsis_os.h"

#include "LaserRangeFinder.h"
//========== other Lib
#include "SystemState.h"

LaserRFDef LaserRF={0};


#ifndef __MyCom_H
float My_abs(float x)
{
	if(x<0)return -x;
	else return x;
}

//判断是否是正数
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
				if(j==Tem_len-1)return 1;//??????
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
#endif


//================================================
//
//						需要外部实现的函数
//
//================================================

//UART串口底层发送
// *s发送首地址 Len发送长度
__weak void LRF_UARTSENT(char*s,int Len)
{
	(void)s;// UNUSED
}

//================================================
//
//								内部函数
//
//================================================

//计算CS校验  加和前面的数据 取反加1
unsigned char GetCheckDate(unsigned char*s ,unsigned char len)
{
	unsigned char i=0;
	unsigned char sum=0;
	for(i=0;i<len;i++)
	{
		sum+=*(s+i);
	}
	return (~sum)+1;
}


//获取数据
float LRFGetDist(char*s)
{
	char strDate[8]={0};
	memcpy(strDate,s,7);
	return str2f((unsigned char*)strDate);

}

//激光测距仪初始化任务
void vRTFInite_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	vTaskDelayUntil(&xLastWakeTime,1000/ portTICK_RATE_MS);
	while(1)
	{
		LRF_UARTSENT(str_LRF_SetRange_5M,5);//设置量程5M
		vTaskDelayUntil(&xLastWakeTime,1000/ portTICK_RATE_MS);
		if (LaserRF.State==LRF_SetRange_OK)
			break;
	}
	
	
	while(1)
	{
		LRF_UARTSENT(str_LRF_SetRevolution_1mm,5);//设置分辨率1mm
		vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
		if (LaserRF.State==LRF_SetRevolution_OK)
			break;
	}
	
	while(1)
	{
		LRF_UARTSENT(str_LRF_SetFrequence_10HZ,5);//设置频率
		vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
		if (LaserRF.State==LRF_SetFrequency_OK)
			break;
	}
	
	while(1)
	{
		LRF_UARTSENT(str_LRF_PowerOFF,4);//关机指令
		vTaskDelayUntil(&xLastWakeTime,1000/ portTICK_RATE_MS);
		if (LaserRF.State==LRF_Ready)
			break;
	}
	vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
	vTaskDelete(NULL);//退出任务
}


//================================================
//
//								外部调用
//
//================================================

//数据解析  只写了连续测量获取
LaserRFStateDef LRF_Analy(unsigned char*s)
{
	unsigned char check=0;
		if(IsStrInc((char*)s,"\x80\x06\x83"))//连续测量返回
		{
			check=GetCheckDate(s,10)+1;
			if(*(s+10)==check)//校验数据
			{
				LaserRF.Dist=LRFGetDist((char*)(s+3));//提取数据
				LaserRF.State=LRF_Mesure_OK;
				return LRF_Mesure_OK;
			}
			else
			{
				LaserRF.State=LRF_DataCheck_Fail;//校验失败
				return LRF_DataCheck_Fail;
			}				
		}
		
		if(IsStrInc((char*)s,"\x80\x04\x82\xFA"))//关机指令返回
		{
				LaserRF.State=LRF_Ready;//进入准备状态
				return LRF_Ready;
		}
		
		if(IsStrInc((char*)s,"\xFA\x04\x89\x79"))//设定量程成功
		{
				LaserRF.State=LRF_SetRange_OK;
				return LRF_SetRange_OK;
		}
		
		if(IsStrInc((char*)s,"\xFA\x84\x89\x01\xF8"))//设定量程失败
		{
				LaserRF.State=LRF_SetRange_Fail;
				return LRF_SetRange_Fail;
		}
		
		if(IsStrInc((char*)s,"\xFA\x04\x8A\x78"))//设定频率成功
		{
				LaserRF.State=LRF_SetFrequency_OK;
				return LRF_SetFrequency_OK;
		}
		
		if(IsStrInc((char*)s,"\xFA\x84\x8A\x01\xF7"))//设定频率失败
		{
				LaserRF.State=LRF_SetFrequency_Fail;
				return LRF_SetFrequency_Fail;
		}
		
		if(IsStrInc((char*)s,"\xFA\x04\x8C\x76"))//设定分辨率成功
		{
				LaserRF.State=LRF_SetRevolution_OK;
				return LRF_SetRevolution_OK;
		}
		if(IsStrInc((char*)s,"\xFA\x84\x8C\x01\xF5"))//设定分辨率失败
		{
				LaserRF.State=LRF_SetRevolution_Fail;
				return LRF_SetRevolution_Fail;
		}
		return 0xFF;
}

//初始化  1:成功启动初始化任务  0:初始化任务失败
char LaserRFInite()
{
	if(xTaskCreate(vRTFInite_Task, "vRTFInite_Task", 200, NULL, 3, NULL))
		return 1;
	else 
		return 0;
}


//开始测量
void StartTOMeasure()
{
	if(LaserRF.State==LRF_Ready)
	{
		LRF_UARTSENT(str_LRF_StarMesure,4);
		LaserRF.State=LRF_BUSY;
	}
}

//停止测量
void StopMeasure()
{
	if(LaserRF.State==LRF_BUSY || LaserRF.State==LRF_Mesure_OK)
		LRF_UARTSENT(str_LRF_PowerOFF,4);//关机指令
}
