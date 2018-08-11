#include "GY-53.h"

#include "SystemState.h"

GY_53Def GY_53_1={0},GY_53_2={0},GY_53_3={0},GY_53_4={0},GY_53_5={0},GY_53_6={0};
//=====================================================
//							  �ⲿʵ��
//
//=====================================================
//���ڷ���
__weak void GY_53UartSent(GY_53Def* temGY_53,char* s)
{
	(void) s;
}

__weak float GetSystemTimer()
{
	return 0;
}
//=====================================================
//							  �ڲ�����
//
//=====================================================
char GY_53CheckSum(char*c)
{
	char sum=0;
	char n=7;
	while(n--)
	{
		sum+=*c;
		c++;
	}
	return (sum==*(c));
}

//GY-53���ó�ʼ����������
void vGY53Set_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		if(MyFlagGet(SystemState.OutLine_Flag,Dist1_No))
		{
			GY_53Com(&GY_53_1,GY_53Com_ContinuMer);
			vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
			GY_53Com(&GY_53_1,GY_53Com_FastMer);
		}
		
		if(MyFlagGet(SystemState.OutLine_Flag,Dist2_No))
		{
			GY_53Com(&GY_53_2,GY_53Com_ContinuMer);
			vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
			GY_53Com(&GY_53_2,GY_53Com_FastMer);
		}
		
		if(MyFlagGet(SystemState.OutLine_Flag,Dist3_No))
		{
			GY_53Com(&GY_53_3,GY_53Com_ContinuMer);
			vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
			GY_53Com(&GY_53_3,GY_53Com_FastMer);
		}
		
		if(MyFlagGet(SystemState.OutLine_Flag,Dist4_No))
		{
			GY_53Com(&GY_53_4,GY_53Com_ContinuMer);
			vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
			GY_53Com(&GY_53_4,GY_53Com_FastMer);
		}
		
		if(MyFlagGet(SystemState.OutLine_Flag,Dist5_No))
		{
			GY_53Com(&GY_53_5,GY_53Com_ContinuMer);
			vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
			GY_53Com(&GY_53_5,GY_53Com_FastMer);
		}
		
		if(MyFlagGet(SystemState.OutLine_Flag,Dist6_No))
		{
			GY_53Com(&GY_53_6,GY_53Com_ContinuMer);
			vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
			GY_53Com(&GY_53_6,GY_53Com_FastMer);
		}
		vTaskDelayUntil(&xLastWakeTime,300/ portTICK_RATE_MS);
	}
	vTaskDelete(NULL);//�˳�����
}


//=====================================================
//							  �ⲿ����
//
//=====================================================
char GY_53Inite()
{
	if(!xTaskCreate( vGY53Set_Task, "GY-53Set_Task", 200, NULL, 3, NULL ))return 0;
	return 1;
}

GY_53StateDef GY_53Analy(GY_53Def* temGY_53,char* c)
{
		if(GY_53CheckSum(c))
		{
			if(c[0]==0x5a && c[1]==0x5a)
			{
				temGY_53->Dis=c[4]<<8|c[5];
				switch(c[6])
				{
					case 0x00:
						temGY_53->Mode=GY_53LongMeur;
						break;
					case 0x01:
						temGY_53->Mode=GY_53FastMeur;
						break;
					case 0x02:
						temGY_53->Mode=GY_53HighQuanlityMeur;
						break;
					case 0x03:
						temGY_53->Mode=GY_53Genaral;
						break;
				}
				
				temGY_53->fps=1000.0f/(GetSystemTimer() - temGY_53->lastComuTim);
				temGY_53->lastComuTim=GetSystemTimer();
				
				return GY_53DataUpDate;
			}
			else
				return GY_53FrameCheckFail;
		}
		else
			return GY_53SumCheckFail;
}

void GY_53Com(GY_53Def* temGY_53,GY_53ComDef Com)
{
		switch(Com)
		{
			case GY_53Com_ContinuMer://��������
				GY_53UartSent(temGY_53,"\xA5\x45\xEA");
				break;
			case GY_53Com_FastMer://���ٲ���
				GY_53UartSent(temGY_53,"\xA5\x51\xF6");
				break;
		}
}


