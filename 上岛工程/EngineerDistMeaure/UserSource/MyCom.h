#ifndef __MyCom_H
#define __MyCom_H

#define my_abs(x) ((x)>0?(x):-(x)) //ABS宏定义

#define MyFlagSet(x,y) x=x|(0x00000001<<y) //设置标志位  y第几位
#define MyFlagClear(x,y) x=x&~(0x00000001<<y)
#define MyFlagGet(x,y) (x&(0x00000001<<y))

int IsPositive(float x);//判断是否是正数  正数返回1 负数返回0
int IsStrInc(char* data,char* temp);//判断字符串中是否包含模板字符
float My_abs(float x);//float ABS
float str2f(unsigned char *s);//字符串转float

#endif
