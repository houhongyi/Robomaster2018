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

//�ж��Ƿ�������
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
				if(j==Tem_len-1)return 1;//??????
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
#endif


//================================================
//
//						��Ҫ�ⲿʵ�ֵĺ���
//
//================================================

//UART���ڵײ㷢��
// *s�����׵�ַ Len���ͳ���
__weak void LRF_UARTSENT(char*s,int Len)
{
	(void)s;// UNUSED
}

//================================================
//
//								�ڲ�����
//
//================================================

//����CSУ��  �Ӻ�ǰ������� ȡ����1
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


//��ȡ����
float LRFGetDist(char*s)
{
	char strDate[8]={0};
	memcpy(strDate,s,7);
	return str2f((unsigned char*)strDate);

}

//�������ǳ�ʼ������
void vRTFInite_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	vTaskDelayUntil(&xLastWakeTime,1000/ portTICK_RATE_MS);
	while(1)
	{
		LRF_UARTSENT(str_LRF_SetRange_5M,5);//��������5M
		vTaskDelayUntil(&xLastWakeTime,1000/ portTICK_RATE_MS);
		if (LaserRF.State==LRF_SetRange_OK)
			break;
	}
	
	
	while(1)
	{
		LRF_UARTSENT(str_LRF_SetRevolution_1mm,5);//���÷ֱ���1mm
		vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
		if (LaserRF.State==LRF_SetRevolution_OK)
			break;
	}
	
	while(1)
	{
		LRF_UARTSENT(str_LRF_SetFrequence_10HZ,5);//����Ƶ��
		vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
		if (LaserRF.State==LRF_SetFrequency_OK)
			break;
	}
	
	while(1)
	{
		LRF_UARTSENT(str_LRF_PowerOFF,4);//�ػ�ָ��
		vTaskDelayUntil(&xLastWakeTime,1000/ portTICK_RATE_MS);
		if (LaserRF.State==LRF_Ready)
			break;
	}
	vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
	vTaskDelete(NULL);//�˳�����
}


//================================================
//
//								�ⲿ����
//
//================================================

//���ݽ���  ֻд������������ȡ
LaserRFStateDef LRF_Analy(unsigned char*s)
{
	unsigned char check=0;
		if(IsStrInc((char*)s,"\x80\x06\x83"))//������������
		{
			check=GetCheckDate(s,10)+1;
			if(*(s+10)==check)//У������
			{
				LaserRF.Dist=LRFGetDist((char*)(s+3));//��ȡ����
				LaserRF.State=LRF_Mesure_OK;
				return LRF_Mesure_OK;
			}
			else
			{
				LaserRF.State=LRF_DataCheck_Fail;//У��ʧ��
				return LRF_DataCheck_Fail;
			}				
		}
		
		if(IsStrInc((char*)s,"\x80\x04\x82\xFA"))//�ػ�ָ���
		{
				LaserRF.State=LRF_Ready;//����׼��״̬
				return LRF_Ready;
		}
		
		if(IsStrInc((char*)s,"\xFA\x04\x89\x79"))//�趨���̳ɹ�
		{
				LaserRF.State=LRF_SetRange_OK;
				return LRF_SetRange_OK;
		}
		
		if(IsStrInc((char*)s,"\xFA\x84\x89\x01\xF8"))//�趨����ʧ��
		{
				LaserRF.State=LRF_SetRange_Fail;
				return LRF_SetRange_Fail;
		}
		
		if(IsStrInc((char*)s,"\xFA\x04\x8A\x78"))//�趨Ƶ�ʳɹ�
		{
				LaserRF.State=LRF_SetFrequency_OK;
				return LRF_SetFrequency_OK;
		}
		
		if(IsStrInc((char*)s,"\xFA\x84\x8A\x01\xF7"))//�趨Ƶ��ʧ��
		{
				LaserRF.State=LRF_SetFrequency_Fail;
				return LRF_SetFrequency_Fail;
		}
		
		if(IsStrInc((char*)s,"\xFA\x04\x8C\x76"))//�趨�ֱ��ʳɹ�
		{
				LaserRF.State=LRF_SetRevolution_OK;
				return LRF_SetRevolution_OK;
		}
		if(IsStrInc((char*)s,"\xFA\x84\x8C\x01\xF5"))//�趨�ֱ���ʧ��
		{
				LaserRF.State=LRF_SetRevolution_Fail;
				return LRF_SetRevolution_Fail;
		}
		return 0xFF;
}

//��ʼ��  1:�ɹ�������ʼ������  0:��ʼ������ʧ��
char LaserRFInite()
{
	if(xTaskCreate(vRTFInite_Task, "vRTFInite_Task", 200, NULL, 3, NULL))
		return 1;
	else 
		return 0;
}


//��ʼ����
void StartTOMeasure()
{
	if(LaserRF.State==LRF_Ready)
	{
		LRF_UARTSENT(str_LRF_StarMesure,4);
		LaserRF.State=LRF_BUSY;
	}
}

//ֹͣ����
void StopMeasure()
{
	if(LaserRF.State==LRF_BUSY || LaserRF.State==LRF_Mesure_OK)
		LRF_UARTSENT(str_LRF_PowerOFF,4);//�ػ�ָ��
}
