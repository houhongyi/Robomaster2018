#ifndef __BSP_PID_H__
#define __BSP_PID_H__

typedef struct 
{
	float P;
	float I;
	float D;
	float E;
	float PreE;
	float PrePreE;
	float U;
	float Intergral;
	float Ilimit;
	float Ulimit;
	//»ý·Ö·ÖÀë
	float E_max;


}PID;


typedef enum 
{
	Positional_PID = 0,
	Incremental_PID =1,
	
}PID_TYPE;

float PID_Control_normal(PID *pid, float Expected_value, float Actual_value, PID_TYPE Pid_type);

#endif 

