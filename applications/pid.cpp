#include "pid.h"
#include <math.h>
struct pid_all pid_andy;
void PID_init(struct _pid_speed *pid,float kp,float ki,float kd) 
{
	pid->err=0;
	pid->last_err=0;
	pid->DeadBand=2;
	//pid->IntegralLimit=533700;
	pid->MaxOutput=1000000;
	pid->kp=kp;
	pid->ki=ki;
	pid->kd=kd;

}
// void PID_init_pi(struct _pid_pi *pid,float kp,float ki,float kd)
// {
// 	pid->err=0;
// 	pid->last_err=0;
// 	pid->DeadBand=1;
// 	pid->MaxOutput=373;
// 	pid->kp=kp;
// 	pid->ki=ki;
// 	pid->kd=kd;
// }
float PID_realize_speed(struct _pid_speed *pid,float targetSpeed,float realitysPeed )
{
	pid->now_speed=realitysPeed;
	pid->target=targetSpeed;
	pid->measure = realitysPeed;						//目标速度		
	pid->last_err  = pid->err;					//更新前一次误差
	pid->err = pid->target - pid->measure;		 //计算当前误差
    
    
	pid->last_output = pid->output;
	if((abs(pid->err) > pid->DeadBand))		//是否进入死区，如果进入则直接跳过，返回上一次的output结果
	{
        
		pid->pout = pid->kp * pid->err;			
		pid->iout += (pid->ki * pid->err);			//注意是加等于
		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
        
		//积分是否超出限制
		// if(pid->iout > pid->IntegralLimit)
		// 	pid->iout = pid->IntegralLimit;
		// if(pid->iout < - pid->IntegralLimit)
		// 	pid->iout = - pid->IntegralLimit;
		
        
		//pid输出和
		pid->output = pid->pout + pid->iout + pid->dout;
		

		//限制输出的大小
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}
	return pid->output;
}


// float PID_realize_pi(struct _pid_pi *pid,float targetpi,float realityspi )
// {
// 	pid->target=targetpi;
//    pid->measure = realityspi;						//目标速度

		
// 	pid->last_err  = pid->err;					//更新前一次误差
// 	pid->err = pid->target - pid->measure;		 //计算当前误差
    
    
// 	pid->last_output = pid->output;
    
	
// 	if((abs(pid->err) > pid->DeadBand))		//是否进入死区，如果进入则直接跳过，返回上一次的output结果
// 	{
        
// 		pid->pout = pid->kp * pid->err;			
// 		pid->iout += (pid->ki * pid->err);			//注意是加等于
// 		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
        
// 		// //积分是否超出限制
// 		// if(pid->iout > pid->IntegralLimit)
// 		// 	pid->iout = pid->IntegralLimit;
// 		// if(pid->iout < - pid->IntegralLimit)
// 		// 	pid->iout = - pid->IntegralLimit;
		
        
// 		//pid输出和
// 		pid->output = pid->pout + pid->iout + pid->dout;
		

// 		//限制输出的大小
// 		if(pid->output>pid->MaxOutput)         
// 		{
// 			pid->output = pid->MaxOutput;
// 		}
// 		if(pid->output < -(pid->MaxOutput))
// 		{
// 			pid->output = -(pid->MaxOutput);
// 		}
	
// 	}
// 	return pid->output;
// }

void set_p_i_d(struct _pid_speed *pid, float p, float i, float d)
{
	pid->kp = p;    // 设置比例系数 P
	pid->ki = i;    // 设置积分系数 I
	pid->kd = d;    // 设置微分系数 D
}
