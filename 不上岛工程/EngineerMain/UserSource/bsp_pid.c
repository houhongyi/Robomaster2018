#include "bsp_pid.h"
#include <stdlib.h>


/**
* @brief  普通pid控制
* @param  float Expected_value 期望
* @param  float Actual_value 反馈
* @param  PID_TYPE Pid_type PID运算的类型
* @retval 
* @note   
*
*/

float PID_Control_normal(PID *pid,float Expected_value,float Actual_value, PID_TYPE Pid_type)
{

		pid->E = Expected_value - Actual_value;

		if(Pid_type)
		{
		
			pid->U += pid->P * (pid->E - pid->PreE) + 
							pid->I * (pid->E) + 
							pid->D * (pid->E - 2 * pid->PreE + pid->PrePreE);
		}
		else
		{
				pid->Intergral += pid->E;
				if (abs(pid->Intergral) >= pid->Ilimit)
						pid->Intergral = pid->Ilimit * (abs(pid->Intergral) / pid->Intergral);

				pid->U = pid->P * pid->E + 
							pid->I * pid->Intergral + 
							pid->D *(pid->E - pid->PreE);
		}

		if (abs(pid->U) >= pid->Ulimit)
				pid->U = pid->Ulimit * (abs(pid->U) / pid->U);
	
				pid->PrePreE = pid->PreE;
				pid->PreE = pid->E;

		return pid->U;

}

//积分分离,个人感觉位置pid比较适合，而增量式pid需要在系统稳定后，采用这种控制。
float PID_Control_Integral_separation(PID *pid, float Expected_value, float Actual_value, PID_TYPE Pid_type)
{
	unsigned int M = 1;

	pid->E = Expected_value - Actual_value;

	if (pid->E > pid->E_max)

		M = 0;
	else
		M = 1;

	if (Pid_type)
	{
		pid->U += pid->P * (pid->E - pid->PreE) +
			M * pid->I * (pid->E) +
			pid->D * (pid->E - 2 * pid->PreE + pid->PrePreE);
	}
	else
	{
		pid->Intergral += pid->E;
		if (abs(pid->Intergral) >= pid->Ilimit)
			pid->Intergral = pid->Ilimit * (abs(pid->Intergral) / pid->Intergral);

		pid->U = pid->P * pid->E +
			M * pid->I * pid->Intergral +
			pid->D *(pid->E - pid->PreE);


	}

	if (abs(pid->U) >= pid->Ulimit)
		pid->U = pid->Ulimit * (abs(pid->U) / pid->U);

	pid->PrePreE = pid->PreE;
	pid->PreE = pid->E;

	return pid->U;

}
