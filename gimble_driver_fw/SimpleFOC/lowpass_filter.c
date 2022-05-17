#include "main.h"

/******************************************************************************/
LowPassFilter  LPF_current_q,LPF_current_d,LPF_velocity;
/******************************************************************************/
void LPF_init(void)
{
	LPF_current_q.Tf=0.05;    
	LPF_current_q.y_prev=0;
	LPF_current_q.timestamp_prev=0;  //SysTick->VAL;
	
	LPF_current_d.Tf=0.05;
	LPF_current_d.y_prev=0;
	LPF_current_d.timestamp_prev=0;
	
	LPF_velocity.Tf=0.05;   //0.02 Tf太小电机容易抖动
	LPF_velocity.y_prev=0;
	LPF_velocity.timestamp_prev=0;
}
/******************************************************************************/
float LPFoperator(LowPassFilter* LPF,float x)
{
	unsigned long now_us;
	float dt, alpha, y;
	
	now_us = SysTick->VAL;
	if(now_us < LPF->timestamp_prev)
		dt = (float)(LPF->timestamp_prev - now_us)/9*1e-6f;
	else
		dt = (float)(0xFFFFFF - now_us + LPF->timestamp_prev)/9*1e-6f;
	
	LPF->timestamp_prev = now_us;
	if(dt > 0.3)   //时间过长，大概是程序刚启动初始化，直接返回
	{
		LPF->y_prev = x;
		return x;
	}
	
	alpha = LPF->Tf/(LPF->Tf + dt);
	y = alpha*LPF->y_prev + (1.0f - alpha)*x;
	LPF->y_prev = y;
	
	return y;
}
/******************************************************************************/
