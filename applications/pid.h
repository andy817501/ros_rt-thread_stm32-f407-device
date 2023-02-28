#ifndef _PID_H_
#define _PID_H_
extern struct pid_all pid_andy;

extern "C"
{

struct _pid_speed {
	float now_speed;
	float target;							//目标值
	float kp;								//比例系数
	float ki;								//积分系数
	float kd;								//微分系数
	float   measure;						//测量值
	float   err;							//误差
	float   last_err;      	      			//上次误差
	float pout;								//比例项
	float iout;								//积分项
	float dout;								//微分项
	float output;							//本次输出
	float last_output;						//上次输出
	float MaxOutput;						//输出限幅
	float IntegralLimit;					//积分限幅
	float DeadBand;						    //死区（绝对值）
	float  Max_Err;							//最大误差
};
// struct _pid_pi {
// 	float now_speed;
// 	float target;							//目标值
// 	float kp;								//比例系数
// 	float ki;								//积分系数
// 	float kd;								//微分系数
// 	float   measure;						//测量值
// 	float   err;							//误差
// 	float   last_err;      	      			//上次误差
// 	float pout;								//比例项
// 	float iout;								//积分项
// 	float dout;								//微分项
// 	float output;							//本次输出
// 	float last_output;						//上次输出
// 	float MaxOutput;						//输出限幅
// 	float IntegralLimit;					//积分限幅
// 	float DeadBand;						    //死区（绝对值）
// 	float  Max_Err;							//最大误差
// };
struct pid_all{
struct _pid_speed whlee1;
struct _pid_speed whlee2;
struct _pid_speed whlee3;
struct _pid_speed whlee4;
// struct _pid_pi whlee1_pi;
// struct _pid_pi whlee2_pi;
// struct _pid_pi whlee3_pi;
// struct _pid_pi whlee4_pi;
};


void PID_init(struct _pid_speed *pid,float kp,float ki,float kd);
// void PID_init_pi(struct _pid_pi *pid,float kp,float ki,float kd);
float PID_realize_speed(struct _pid_speed *pid,float targetSpeed,float realitysPeed );
// float PID_realize_pi(struct _pid_pi *pid,float targetpi,float realityspi );
void set_p_i_d(struct _pid_speed *pid, float p, float i, float d);
}
#endif