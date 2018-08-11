#include <string.h>
#include "math.h"
#include "MyCom.h"

float My_abs(float x)
{
	if(x<0)return -x;
	else return x;
}

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
