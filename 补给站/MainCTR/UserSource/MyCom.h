#ifndef __MyCom_H
#define __MyCom_H

#define my_abs(x) ((x)>0?(x):-(x)) //ABS�궨��

#define MyFlagSet(x,y) x=x|(0x00000001<<y) //���ñ�־λ  y�ڼ�λ
#define MyFlagClear(x,y) x=x&~(0x00000001<<y)
#define MyFlagGet(x,y) (x&(0x00000001<<y))

int IsPositive(float x);//�ж��Ƿ�������  ��������1 ��������0
int IsStrInc(char* data,char* temp);//�ж��ַ������Ƿ����ģ���ַ�
float My_abs(float x);//float ABS
float str2f(unsigned char *s);//�ַ���תfloat

#endif
