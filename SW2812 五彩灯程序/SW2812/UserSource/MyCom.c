#include <string.h>
#include "math.h"
#include "MyCom.h"

float My_abs(float x)
{
	if(x<0)return -x;
	else return x;
}

//�ж��Ƿ�������  ��������1 ��������0
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
				if(j==Tem_len-1)return 1;//����
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

//��ȡ�м������
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
