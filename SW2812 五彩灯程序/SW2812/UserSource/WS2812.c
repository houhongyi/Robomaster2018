#include "WS2812.h"
#include "string.h"

LED_5050DriverDef LED_5050Driver={0};


void LED_5050SentStar()
{
	LED_5050_Data2PWMTime();
	HAL_TIM_PWM_Start_DMA(LED_5050Driver.htim,TIM_CHANNEL_1, (unsigned int *)LED_5050Driver.Data_Pulse, LED_5050_Data_Bit_n+1);
}

void LED_5050_Data2PWMTime()
{
	short n,m,k;
	LED_5050Driver.Bit_n=0;
	while(LED_5050Driver.Bit_n<LED_5050_Data_Bit_n)
	{
		for(n=0;n<LED_5050_Data_n;n++)
		{
			for(m=7;m>=0;m--)
			{
				k=8*n+m;
				if( (LED_5050Driver.Data[LED_5050Driver.Bit_n/8]>>(LED_5050Driver.Bit_n%8) )&0x01)//如果第 LED_5050Driver.Bit_n 位为 1
					LED_5050Driver.Data_Pulse[k]=71;// 0.85us高0.4us低为 1
				else
					LED_5050Driver.Data_Pulse[k]=34;// 0.4us高0.85us低为 0
			
				LED_5050Driver.Bit_n++;
			}
			
		}
	}
}

char LED_5050DriverInite()
{
	LED_5050Driver.htim=&htim3;
	LED_5050Driver.htim->Instance->CR1|=0x80;//缓冲设置
	return 1;
}



//**************************************************************************
//**************************************************************************
//											动画函数函数
//**************************************************************************
//**************************************************************************
void LED_Data_Fill_All(unsigned char G,unsigned char R,unsigned char B)
{
	int i=0;
	for(i=0;i<LED_5050_n;i++)
	{
		LED_5050Driver.Data_Breath[i*3]   = G;
		LED_5050Driver.Data_Breath[i*3+1] = R;
		LED_5050Driver.Data_Breath[i*3+2] = B;
		
		LED_5050Driver.Data[i*3]   = G;
		LED_5050Driver.Data[i*3+1] = R;
		LED_5050Driver.Data[i*3+2] = B;
	}
}

void LED_DataFill_Flow_B(unsigned char G,unsigned char R,unsigned char B)
{
	LED_Data_Fill_All(0,0,0);
	
	LED_5050Driver.Data[0]=G/10;//G
	LED_5050Driver.Data[1]=R/10;//R
	LED_5050Driver.Data[2]=B/10;//B
	
	LED_5050Driver.Data[3]=G/3;
	LED_5050Driver.Data[4]=R/3;
	LED_5050Driver.Data[5]=B/3;
	
	LED_5050Driver.Data[6]=G;
	LED_5050Driver.Data[7]=R;
	LED_5050Driver.Data[8]=B;
}

void LED_DataFill_Flow_F(unsigned char G,unsigned char R,unsigned char B)
{

	LED_Data_Fill_All(0,0,0);
	
	LED_5050Driver.Data[0]=G;//G
	LED_5050Driver.Data[1]=R;//R
	LED_5050Driver.Data[2]=B;//B
	
	LED_5050Driver.Data[3]=G/3;
	LED_5050Driver.Data[4]=R/3;
	LED_5050Driver.Data[5]=B/3;
	
	LED_5050Driver.Data[6]=G/10;
	LED_5050Driver.Data[7]=R/10;
	LED_5050Driver.Data[8]=B/10;
}

void LED_Flash_Flow_Front()
{
	unsigned char i=0;
	unsigned char d0,d1,d2;
	d0=LED_5050Driver.Data[0];
	d1=LED_5050Driver.Data[1];
	d2=LED_5050Driver.Data[2];

	for(i=0;i<LED_5050_n-1;i++)
	{
		LED_5050Driver.Data[i*3]   = LED_5050Driver.Data[i*3+3];
		LED_5050Driver.Data[i*3+1] = LED_5050Driver.Data[i*3+4];
		LED_5050Driver.Data[i*3+2] = LED_5050Driver.Data[i*3+5];
	}
	
	LED_5050Driver.Data[(LED_5050_n-1)*3]=d0;
	LED_5050Driver.Data[(LED_5050_n-1)*3+1]=d1;
	LED_5050Driver.Data[(LED_5050_n-1)*3+2]=d2;
}

void LED_Flash_Flow_Back()
{
	unsigned char i=0;
	unsigned char d0,d1,d2;
	d0=LED_5050Driver.Data[(LED_5050_n-1)*3];
	d1=LED_5050Driver.Data[(LED_5050_n-1)*3+1];
	d2=LED_5050Driver.Data[(LED_5050_n-1)*3+2];
	for(i=LED_5050_n;i>1;i--)
	{
		LED_5050Driver.Data[(i-1)*3]   = LED_5050Driver.Data[(i-1)*3-3];
		LED_5050Driver.Data[(i-1)*3+1] = LED_5050Driver.Data[(i-1)*3-2];
		LED_5050Driver.Data[(i-1)*3+2] = LED_5050Driver.Data[(i-1)*3-1];
	}
	
	LED_5050Driver.Data[0]=d0;
	LED_5050Driver.Data[1]=d1;
	LED_5050Driver.Data[2]=d2;
}

void LED_Flash_Breath()
{
	unsigned char i=0;
	static float k=0;
	static float Dir=0.1;
	for(i=0;i<LED_5050_n;i++)
	{
		LED_5050Driver.Data[i*3]   = LED_5050Driver.Data_Breath[i*3] * k;
		LED_5050Driver.Data[i*3+1] = LED_5050Driver.Data_Breath[i*3+1] * k;
		LED_5050Driver.Data[i*3+2] = LED_5050Driver.Data_Breath[i*3+2] * k;
	}
	
	k+=Dir;
	if(k<=0.11)Dir=0.1f;
	if(k>=0.91)Dir=-0.1f;
}

void LED_Flash_Flite()//闪烁
{
	unsigned char i=0;
	static float k=0;
	static float n=0;
	static float Dir=0.15;
	for(i=0;i<LED_5050_n;i++)
	{
		LED_5050Driver.Data[i*3]   = LED_5050Driver.Data_Breath[i*3] * k;
		LED_5050Driver.Data[i*3+1] = LED_5050Driver.Data_Breath[i*3+1] * k;
		LED_5050Driver.Data[i*3+2] = LED_5050Driver.Data_Breath[i*3+2] * k;
	}
	
	n+=Dir;
	if (n>0.9)k=1;
	else k=0;
	if(n>=1)n=0;

}

//LED动画
void LED_Flash()
{
	switch(LED_5050Driver.Mode.LED_FlashMode)
		{
			case UpIsLandFlow_G:
					LED_Flash_Flow_Front();
				break;
			case UpIsLandFlow_R:
					LED_Flash_Flow_Front();
				break;
			case UpIsLandFlow_Y:
					LED_Flash_Flow_Front();
				break;
			
			case DownIsLandFlow_G:
					LED_Flash_Flow_Back();
				break;
			case DownIsLandFlow_R:
					LED_Flash_Flow_Back();
				break;
			case DownIsLandFlow_Y:
					LED_Flash_Flow_Back();
				break;
			case GetBullet_GBR://获取子弹
				LED_Flash_Flite();//闪烁
				break;
			case Breath_GBR://自定义呼吸
					LED_Flash_Breath();
				break;
			case CustomFlash://自定义数据 没有动画效果
				break;
			
			default:LED_Flash_Breath();
				break;
		}
}


void LED_Data_Fill(LED_FlashModeEnum mode,unsigned char* s,unsigned char l)
{
	LED_5050Driver.Mode.LED_FlashMode=mode;
	if(LED_5050Driver.Mode.LED_FlashMode_pre!=LED_5050Driver.Mode.LED_FlashMode || 
		 LED_5050Driver.Mode.LED_FlashMode==CustomFlash ||
			LED_5050Driver.Mode.LED_FlashMode==Breath_GBR)
	{
		switch(LED_5050Driver.Mode.LED_FlashMode)
		{
			case UpIsLandFlow_G:
					LED_DataFill_Flow_F(200,0,0);
				break;
			case UpIsLandFlow_R:
				LED_DataFill_Flow_F(0,200,0);
				break;
			case UpIsLandFlow_Y:
				LED_DataFill_Flow_F(200,200,0);
				break;
			
			case DownIsLandFlow_G:
					LED_DataFill_Flow_B(200,0,0);
				break;
			case DownIsLandFlow_R:
				LED_DataFill_Flow_B(0,200,0);
				break;
			case DownIsLandFlow_Y:
				LED_DataFill_Flow_B(200,200,0);
				break;
			case Breath_GBR:
				memcpy(LED_5050Driver.Data_Breath,s,l-1);
				break;
			case GetBullet_GBR://获取子弹
				LED_Data_Fill_All(50,0,100);
			case CustomFlash :
				memcpy(LED_5050Driver.Data,s,l-1);
				break;
			default:
				LED_Data_Fill_All(100,100,100);
				break;
		}
	}
	LED_5050Driver.Mode.LED_FlashMode_pre=LED_5050Driver.Mode.LED_FlashMode;
}

void LED_UART_Anly(unsigned char*s)
{
	unsigned char datalen=s[3];
	unsigned char Com=s[4];
	unsigned char* Data=s+5;
	LED_Data_Fill((LED_FlashModeEnum)Com,Data,datalen);
}

void LED_UART_Divice(unsigned char*s,short len)//LED串口接收解析
{
	unsigned char*q=s;
	unsigned char*p=s;
	unsigned char l=0;
	while(p-q<len)
	{
		if(*p==0xff && *(p+1)==0xff && *(p+2)==0xff)//找到帧头
		{
			l=*(p+3);
			if(*(p+l+4)==0x0a&&*(p+l+5)==0x0d)
			{
				LED_UART_Anly(s);
				p+=l+5;
			}
		}
		p++;
	}
}
//**************************************************************************
//**************************************************************************
//											回调函数
//**************************************************************************
//**************************************************************************

//PWM  DMA发送完成回调函数

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(LED_5050Driver.htim,TIM_CHANNEL_1);
}


