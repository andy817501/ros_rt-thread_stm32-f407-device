#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <sensor.h>
#include "i2c.h"
#include "main.h"
#include "pid.h"
#include "andysensor.h"
#include "motors.h"
#include <stdio.h>
#include <finsh.h>


// #define DBG_SECTION_NAME "rot_speed"

#define DBG_LEVEL DBG_LOG
#include <rtdbg.h>
#define THREAD_STACK_SIZE   512
#define THREAD_PRIORITY     5
#define THREAD_TIMESLICE    4

rt_adc_device_t adc_dev;
struct mpu6xxx_device *i2c_bus;                     //6050控制句柄
float  max_distance1=0,max_distance2=0,max_distance3=0,max_distance4=0;
struct mpu6xxx_3axes accel, gyro;
float pitch, roll, yaw;           //欧拉角
short count=0;     //pid运算次数
char str[32];             //调试使用到时候去掉
float now1_dis=0,now2_dis,now3_dis,now4_dis;
 struct andysensor*  andy = (struct andysensor*)rt_malloc(sizeof( struct andysensor));
 struct msg_speed*msg_speed=(struct msg_speed*)rt_malloc(sizeof( struct msg_speed));
#define MPU6050_I2C_BUS_NAME          "i2c1"  /* 传感器连接的I2C总线设备名称 */
#define MPU6050_ADDR             0x68

#define HWTIMER_DEV_NAME   "timer11"     /* 定时器名称 */
#define PULSE_ENCODER_DEV_NAME     "pulse2"    /* 脉冲编码器名称 */
#define PULSE_ENCODER_DEV_NAME_2   "pulse3"
#define PULSE_ENCODER_DEV_NAME_3   "pulse4"
#define PULSE_ENCODER_DEV_NAME_4   "pulse5"      
#define ENCODER_RESOLUTION 13    /*编码器一圈的物理脉冲数*/
#define ENCODER_MULTIPLE 4       /*编码器倍频，通过定时器的编码器模式设置*/
#define MOTOR_REDUCTION_RATIO 30 /*电机的减速比*/
/*电机转一圈总的脉冲数(定时器能读到的脉冲数) = 编码器物理脉冲数*编码器倍频*电机减速比 */
#define TOTAL_RESOLUTION    ( ENCODER_RESOLUTION*ENCODER_MULTIPLE*MOTOR_REDUCTION_RATIO )
rt_device_t pulse_encoder_dev = RT_NULL;   /* 脉冲编码器设备句柄 */
rt_device_t pulse_encoder_dev2 = RT_NULL;
rt_device_t pulse_encoder_dev3 = RT_NULL;
rt_device_t pulse_encoder_dev4 = RT_NULL;

#define PERIOD_TIME  0.01  //采样周期s
#define PULSE_PER     4    //每转采样一次，有四个脉冲（N）
    
 rt_mailbox_t mb_speed_pub=RT_NULL; 
extern MotorControl mtr;        //Motor




void MPU6050_DataGet(void)
{
    if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
    {

        // sprintf(str,"pitch=%d\r\n",(int)pitch);
        // rt_kprintf(str);
        // sprintf(str,"roll=%d\r\n",(int)roll);
        // rt_kprintf(str);
        // sprintf(str,"yaw=%d\r\n",(int)yaw);
        // rt_kprintf(str);
        andy->pitch=pitch;
        andy->roll=roll;
        andy->yaw=yaw;
      mpu6xxx_get_gyro(i2c_bus, &gyro);

        andy->angular_x=(float)gyro.x*0.0017453293;
        // sprintf(str,"gyro.x=%.2f\r\n",andy->angular_x);
        // rt_kprintf(str);
        andy->angular_y=(float)gyro.y*0.0017453293;
    //     sprintf(str,"gyro.y=%.2f\r\n",andy->angular_y);
    //   rt_kprintf(str);
        andy->angular_z=(float)gyro.z*0.0017453293;
    //   sprintf(str,"gyro.z=%.2f\r\n",andy->angular_z);
    //   rt_kprintf(str);
      mpu6xxx_get_accel(i2c_bus,&accel);

      andy->linear_x=(float)accel.x/1000*9.79;
      andy->linear_y=(float)accel.y/1000*9.79;
      andy->linear_z=(float)accel.z/1000*9.79;
    //   sprintf(str,"accel.x=%.2f\r\n",andy->linear_x);
    //   rt_kprintf(str);
    //   sprintf(str,"accel.y=%.2f\r\n",andy->linear_y);
    //   rt_kprintf(str);
    //   sprintf(str,"accel.z=%.2f\r\n",andy->linear_z);
    //   rt_kprintf(str);

    }
}

// static void thread1_entry(void* parameter)
// {

//     while (1)
//     {
//         MPU6050_DataGet();
//     }
// }   

// int mpu6050_sample(void)
// {
//     static rt_thread_t tid1 = RT_NULL;

//     /* 创建线程  */
//     tid1 = rt_thread_create("thread1", thread1_entry,
//     RT_NULL,
//     THREAD_STACK_SIZE,
//     THREAD_PRIORITY, THREAD_TIMESLICE);
//     /* 如果获得线程控制块，启动这个线程 */
//     if (tid1 != RT_NULL)
//         rt_thread_startup(tid1);

//     /* 创建线程  */

//     return 0;
// }
/* 导出到 msh 命令列表中 */
//MSH_CMD_EXPORT(mpu6050_sample, mpu6050 sample);
//INIT_COMPONENT_EXPORT(mpu6050_sample);

// void mpu6050_get_date_andy()
// {
    
//       mpu6xxx_get_gyro(i2c_bus, &gyro);
//       sprintf(str,"gyro.x=%d\r\n",gyro.x);
//       rt_kprintf(str);

//       sprintf(str,"gyro.y=%d\r\n",gyro.y);
//       rt_kprintf(str);

//       sprintf(str,"gyro.z=%d\r\n",gyro.z);
//       rt_kprintf(str);
//       mpu6xxx_get_accel(i2c_bus,&accel);

//       sprintf(str,"accel.x=%d\r\n",accel.x);
//       rt_kprintf(str);
//       sprintf(str,"accel.y=%d\r\n",accel.y);
//       rt_kprintf(str);
//       sprintf(str,"accel.z=%d\r\n",accel.z);
//       rt_kprintf(str);
// }



//计算公式：
//每毫秒采样的脉冲数：一定周期(ms)内采样的总脉冲数/周期      
//4分频，一圈4个脉冲
//1ms的圈数：一定周期(ms)内采样的总脉冲数/（周期 *4）
//转速（圈/分钟）：一定周期(ms)内采样的总脉冲数/（周期 *4）    乘以60*1000（分钟）
 
/* 定时器超时回调函数 */
static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
     rt_int32_t count1,count2,count3,count4;
    /* 读取脉冲编码器计数值 */
    rt_device_read(pulse_encoder_dev, 0, &count1, 1);
    rt_device_read(pulse_encoder_dev2, 0, &count2, 1);
    rt_device_read(pulse_encoder_dev3, 0, &count3, 1);
    rt_device_read(pulse_encoder_dev4, 0, &count4, 1);
    /* 清空脉冲编码器计数值 */
    rt_device_control(pulse_encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
    rt_device_control(pulse_encoder_dev2, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
    rt_device_control(pulse_encoder_dev3, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
    rt_device_control(pulse_encoder_dev4, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);

    if(count1>=30000) {count1=(count1-32767.5);}
    if(count2>=30000) {count2=count2-32767.5;}
    if(count3>=30000) {count3=(count3-32767.5);}
    if(count4>=30000) {count4=(count4-32767.5);}

    float speed1 = 60*1000*(float)count1/(TOTAL_RESOLUTION*10);
    float speed2 = 60*1000*(float)count2/(TOTAL_RESOLUTION*10);
    float speed3 = 60*1000*(float)count3/(TOTAL_RESOLUTION*10);
    float speed4 = 60*1000*(float)count4/(TOTAL_RESOLUTION*10);       //多少转每分钟
    
    float  distance1 = (speed1/60)*PERIOD_TIME*2*WHEEL_PI*WHEEL_R;
     max_distance1=max_distance1+distance1;
    float  distance2 = (speed2/60)*PERIOD_TIME*2*WHEEL_PI*WHEEL_R;
    max_distance2=max_distance2+distance2;
    float  distance3 = (speed3/60)*PERIOD_TIME*2*WHEEL_PI*WHEEL_R;
     max_distance3=max_distance3+distance3;
    float  distance4 = (speed4/60)*PERIOD_TIME*2*WHEEL_PI*WHEEL_R;
     max_distance4=max_distance4+distance4;  



    // now1_dis=now1_dis+(msg_speed->v1)/60*PERIOD_TIME*2*WHEEL_PI*WHEEL_R;
    // now2_dis=now2_dis+(msg_speed->v2)/60*PERIOD_TIME*2*WHEEL_PI*WHEEL_R;
    // now3_dis=now3_dis+(msg_speed->v3)/60*PERIOD_TIME*2*WHEEL_PI*WHEEL_R;
    // now4_dis=now4_dis+(msg_speed->v4)/60*PERIOD_TIME*2*WHEEL_PI*WHEEL_R;   

    // float set_speed1  = PID_realize_pi(&(pid_andy.whlee1_pi),now1_dis,max_distance1);
    // float set_speed2  = PID_realize_pi(&(pid_andy.whlee2_pi),now2_dis,max_distance2);
    // float set_speed3  = PID_realize_pi(&(pid_andy.whlee3_pi),now3_dis,max_distance3);
    // float set_speed4  = PID_realize_pi(&(pid_andy.whlee4_pi),now4_dis,max_distance4);

    float reallypwm1 = PID_realize_speed(&(pid_andy.whlee1),msg_speed->v1,speed1); 
     float reallypwm2 = PID_realize_speed(&(pid_andy.whlee2),msg_speed->v2,speed2);
     float reallypwm3 = PID_realize_speed(&(pid_andy.whlee3),msg_speed->v3,speed3);
     float reallypwm4 = PID_realize_speed(&(pid_andy.whlee4),msg_speed->v4,speed4);


    //  float reallypwm1 = PID_realize_speed(&(pid_andy.whlee1),127,speed1); 
    //  float reallypwm2 = PID_realize_speed(&(pid_andy.whlee2),127,speed2);
    //  float reallypwm3 = PID_realize_speed(&(pid_andy.whlee3),127,speed3);
    //  float reallypwm4 = PID_realize_speed(&(pid_andy.whlee4),127,speed4);


   // rt_kprintf("get count1:%d, speed1:%.2f, distance1:%.2f \n",count1,speed1,distance1); 
    // rt_kprintf("get count2:%d, speed2:%.2f, distance2:%.2f \n",count2,speed2,max_distance2); 
    // rt_kprintf("get count3:%d, speed3:%.2f, distance3:%.2f \n ",count3,speed3,max_distance3); 
    // rt_kprintf("get count4:%d, speed4:%.2f, distance4:%.2f  \n",count4,speed4,max_distance4); 
    // rt_kprintf("\n");


    // rt_kprintf("fk_date=%.2f   \n",speed1/60.0*WHEEL_R *WHEEL_PI*2);

    // float reallypwm1 = PID_realize_speed(&(pid_andy.whlee1),msg_speed->v1,speed1);
    // float reallypwm2 = PID_realize_speed(&(pid_andy.whlee2),msg_speed->v2,speed2);
    // float reallypwm3 =  PID_realize_speed(&(pid_andy.whlee3),msg_speed->v3,speed3);
    // float reallypwm4 = PID_realize_speed(&(pid_andy.whlee4),msg_speed->v4,speed4);


    mtr.move_andy(reallypwm1,1);
    mtr.move_andy(reallypwm2,2);
    mtr.move_andy(reallypwm3,3);
    mtr.move_andy(reallypwm4,4);
    
 


if(count++>=10) { 
    count=0;
    andy->speed1=speed1;
    andy->speed2=speed2;
    andy->speed3=speed3;
    andy->speed4=speed4;
    andy->max_distance1=max_distance1;
    andy->max_distance2=max_distance2;
    andy->max_distance3=max_distance3;
    andy->max_distance4=max_distance4;
    
rt_mb_send(mb_speed_pub, (rt_uint32_t)andy);
}
    return 0;
}
 
 
//脉冲计数初始化
static int pulse_encoder_init()
{
    rt_err_t ret = RT_EOK;
    
    rt_uint32_t index;
   
    /* 查找脉冲编码器设备 */
    pulse_encoder_dev = rt_device_find(PULSE_ENCODER_DEV_NAME);
    if (pulse_encoder_dev == RT_NULL)
    {
        LOG_E("pulse encoder sample run failed! can't find %s device!", PULSE_ENCODER_DEV_NAME);
        return RT_ERROR;
    }
 
    /* 以只读方式打开设备 */
    ret = rt_device_open(pulse_encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME);
        return ret;
    }

    /* 查找脉冲编码器设备  编码器2 定时器3*/
    pulse_encoder_dev2 = rt_device_find(PULSE_ENCODER_DEV_NAME_2);
    if (pulse_encoder_dev2 == RT_NULL)
    {
        LOG_E("pulse encoder sample run failed! can't find %s device!", PULSE_ENCODER_DEV_NAME_2);
        return RT_ERROR;
    }
 
    /* 以只读方式打开设备 */
    ret = rt_device_open(pulse_encoder_dev2, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME_2);
        return ret;
    } 

    /* 查找脉冲编码器设备  编码器3 定时器4*/
    pulse_encoder_dev3 = rt_device_find(PULSE_ENCODER_DEV_NAME_3);
    if (pulse_encoder_dev3 == RT_NULL)
    {
        LOG_E("pulse encoder sample run failed! can't find %s device!", PULSE_ENCODER_DEV_NAME_3);
        return RT_ERROR;
    }
 
    /* 以只读方式打开设备 */
    ret = rt_device_open(pulse_encoder_dev3, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME_3);
        return ret;
    } 

    /* 查找脉冲编码器设备  编码器3 定时器4*/
    pulse_encoder_dev4 = rt_device_find(PULSE_ENCODER_DEV_NAME_4);
    if (pulse_encoder_dev4 == RT_NULL)
    {
        LOG_E("pulse encoder sample run failed! can't find %s device!", PULSE_ENCODER_DEV_NAME_4);
        return RT_ERROR;
    }
 
    /* 以只读方式打开设备 */
    ret = rt_device_open(pulse_encoder_dev4, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME_4);
        return ret;
    } 
    return ret;
}
 
//定时周期初始化
static int hwtimer_init()
{
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_s;      /* 定时器超时值 */
    rt_device_t hw_dev = RT_NULL;   /* 定时器设备句柄 */
    rt_hwtimer_mode_t mode;         /* 定时器模式 */
 
    /* 查找定时器设备 */
    hw_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        LOG_E("hwtimer sample run failed! can't find %s device!", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }
 
    /* 以读写方式打开设备 */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        LOG_E("open %s device failed!", HWTIMER_DEV_NAME);
        return ret;
    }
 
    /* 设置超时回调函数 */
    rt_device_set_rx_indicate(hw_dev, timeout_cb);
 
    /* 设置模式为周期性定时器 */
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        LOG_E("set mode failed! ret is :%d", ret);
        return ret;
    }
 
    /* 设置定时器超时值为5s并启动定时器 */
    timeout_s.sec = 0;      /* 秒 */
    timeout_s.usec = PERIOD_TIME*1000*1000;     /* 微秒 */
 
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        LOG_E("set timeout value failed");
        return RT_ERROR;
    }
 
    /* 延时ms */
    rt_thread_mdelay(500);
 
    /* 读取定时器当前值 */
    rt_device_read(hw_dev, 0, &timeout_s, sizeof(timeout_s));
    LOG_D("Read: Sec = %d, Usec = %d\n", timeout_s.sec, timeout_s.usec);
    
    return ret;
}
void  tail_rot_speed_init(){
    mb_speed_pub = rt_mb_create("speed_pub",100,RT_IPC_FLAG_FIFO);
  if (mb_speed_pub != RT_NULL) {
   rt_kprintf("邮箱Pub创建成功!\n\n");
  }
    i2c_bus = (struct mpu6xxx_device *) mpu6xxx_init(MPU6050_I2C_BUS_NAME, MPU6050_ADDR); //初始化MPU6050，测量单位为角速度，加速度    while(count++)
    while (mpu_dmp_init())
    {
        rt_thread_mdelay(100);
        rt_kprintf("\r\nMPU6050 DMP init Error\r\n");
    }
    rt_kprintf("\r\nMPU6050 DMP init OK\r\n");


    pulse_encoder_init();
    hwtimer_init();  
    msg_speed->v1=0;
    msg_speed->v2=0;
    msg_speed->v3=0;
    msg_speed->v4=0;  
    while (1)
    {
        MPU6050_DataGet();
      if( rt_mb_recv(mb_speed_sub, (rt_ubase_t*)&msg_speed,0)== RT_EOK)
      {
          


      }
      
    }
    
}
