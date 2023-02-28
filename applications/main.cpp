#include <board.h>
#include <main.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include "motors.h"
#include "andysensor.h"
#include "pid.h"
#include <ros/node_handle.h>
#include "ros/time.h"

float s1=0,s2=0,s3=0,s4=0;
float s1_last=0,s2_last=0,s3_last=0,s4_last=0;
float position_x=0,position_y=0,position_w=0;
float linear_x,linear_y,linear_w;
static ros::NodeHandle  nh;
MotorControl mtr(1, 2, 3, 4);   //Motor

rt_mailbox_t mb_speed_sub=RT_NULL;
extern rt_mailbox_t mb_speed_pub;
struct andysensor *speed_rec_pub=(struct andysensor*)rt_malloc(sizeof(struct andysensor)); 
 struct msg_speed*  msg_speed_andy = (struct msg_speed*)rt_malloc(sizeof( struct msg_speed));
bool msgRecieved = false;
float velX = 0;
geometry_msgs::Quaternion q;

static void speed_to_Offset(andysensor *speed_rec_pub)
{
  float s1_delta=0,s2_delta=0,s3_delta=0,s4_delta=0;    //andy thing thing 
  float v1=0,v2=0,v3=0,v4=0;
  float K4_1 = 1.0/(4.0*WHEEL_K);
  float position_x_delta,position_y_delta,position_w_delta;

  s1_last=s1;
  s2_last=s2;
  s3_last=s3;
  s4_last=s4;

  s1=speed_rec_pub->max_distance1;
  s2=speed_rec_pub->max_distance2;
  s3=speed_rec_pub->max_distance3;
  s4=speed_rec_pub->max_distance4;

  s1_delta=s1-s1_last; //每个轮子转速的增量
  s2_delta=s2-s2_last;
  s3_delta=s3-s3_last;
  s4_delta=s4-s4_last;

  
//逆运动学模型转化为X、Y方向的位移以及航向角的变化
  position_x_delta= 0.25*s1_delta+ 0.25*s2_delta+ 0.25*s3_delta+ 0.25*s4_delta;
  position_y_delta = -0.25*s1_delta+ 0.25*s2_delta- 0.25*s3_delta+ 0.25*s4_delta;
  position_w_delta = -K4_1*s1_delta-K4_1*s2_delta+K4_1*s3_delta+ K4_1*s4_delta;    //w 单位为弧度
  // rt_kprintf("增量位移%.2f\n",s1_delta);
 
  //以上电时候的坐标作为里程计的全局坐标 
  position_x=position_x+cos(position_w)*position_x_delta-sin(position_w)*position_y_delta;
  position_y=position_y+sin(position_w)*position_x_delta+cos(position_w)*position_y_delta;
  position_w=position_w+position_w_delta;
// rt_kprintf("x方向位移:%.2f \n , y方向位移:%.2f \n , z方向位移:%.2f \n",position_x,position_y,position_w); 

  if(position_w>2*WHEEL_PI)
  {
     position_w=position_w-2*WHEEL_PI;	
  }
  else if(position_w<-2*WHEEL_PI)
  {
     position_w=position_w+2*WHEEL_PI;
  }
  else;                //zzzz thing thing 

  v1 =   (speed_rec_pub->speed1)/60.0*WHEEL_R *WHEEL_PI*2;
  v2 =   (speed_rec_pub->speed2)/60.0*WHEEL_R *WHEEL_PI*2;
  v3 =   (speed_rec_pub->speed3)/60.0*WHEEL_R *WHEEL_PI*2;
  v4 =   (speed_rec_pub->speed4)/60.0*WHEEL_R *WHEEL_PI*2; 


  linear_x = (float)0.25*v1+ 0.25*v2+ 0.25*v3+ 0.25*v4;
  linear_y =(float) -0.25*v1+ 0.25*v2- 0.25*v3+ 0.25*v4;
  linear_w = (float)-K4_1*v1-K4_1*v2+K4_1*v3+ K4_1*v4;
  //  rt_kprintf("传递过来速度:%.2f \n , 处理之后速度:%.2f \n , 传递给ros速度:%.2f \n",speed_rec_pub->speed1,v1,linear_x); 

}
static void velCB( const geometry_msgs::Twist& twist_msg)  // 接收到命令时的回调函数
{
    float speed_x,speed_y,speed_w;

 
  speed_x = twist_msg.linear.x;
  speed_y = twist_msg.linear.y;
  speed_w = twist_msg.angular.z;
  
  msg_speed_andy->v1 =speed_x-speed_y-WHEEL_K*speed_w;          //转化为每个轮子的线速度
  msg_speed_andy->v2 =speed_x+speed_y-WHEEL_K*speed_w;      
  msg_speed_andy->v3 =(speed_x-speed_y+WHEEL_K*speed_w);   
  msg_speed_andy->v4 =(speed_x+speed_y+WHEEL_K*speed_w);   

  msg_speed_andy->v1 =msg_speed_andy->v1/(2.0*WHEEL_R*WHEEL_PI);    //转换为轮子的速度(弧度/s)
  msg_speed_andy->v2 =msg_speed_andy->v2/(2.0*WHEEL_R*WHEEL_PI);
  msg_speed_andy->v3 =msg_speed_andy->v3/(2.0*WHEEL_R*WHEEL_PI);
  msg_speed_andy->v4 =msg_speed_andy->v4/(2.0*WHEEL_R*WHEEL_PI);
   
  
  msg_speed_andy->v1 =msg_speed_andy->v1*60;    //转换为速度　单位　RPM （转/分钟）
  msg_speed_andy->v2 =msg_speed_andy->v2*60;
  msg_speed_andy->v3 =msg_speed_andy->v3*60;
  msg_speed_andy->v4 =msg_speed_andy->v4*60;
  


  rt_mb_send(mb_speed_sub, (rt_uint32_t)msg_speed_andy);
  

  velX = twist_msg.linear.x;
  msgRecieved = true;
}

//Subscriber
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCB );

//Publisher
std_msgs::Float64 velX_tmp;
std_msgs::Float64 turnBias_tmp; 
nav_msgs::Odometry  odom;
// sensor_msgs::Imu   imu_date;

ros::Publisher xv("vel_x", &velX_tmp);
ros::Publisher od("odom",&odom);
// ros::Publisher im("imu_data",&imu_date);

static void rosserial_thread_entry(void *parameter)
{
    //Init motors, specif>y the respective motor pins
    mtr.initMotors();

    //Init node>
    nh.initNode();

    // 订阅了一个话题 /cmd_vel 接收控制指令
    nh.subscribe(sub);

    // 发布了一个话题 /vel_x 告诉 ROS 小车速度
    nh.advertise(xv);

    // 发布了一个话题 /turn_bias 告诉 ROS 小车的旋转角速度

    nh.advertise(od);

    // nh.advertise(im);

    mtr.stopMotors();

    while (1)
    {
      if (msgRecieved)
      {
        velX *= mtr.maxSpd;
        // mtr.moveBot(velX, turnBias);
        msgRecieved = false;
      }

      velX_tmp.data = velX;

     
      //odom信息修改

if (rt_mb_recv(mb_speed_pub, (rt_ubase_t*)&speed_rec_pub,0) == RT_EOK)
    {   

          //  //线速度
          // imu_date.linear_acceleration.x = speed_rec_pub->linear_x; 
          // imu_date.linear_acceleration.y = speed_rec_pub->linear_y;
          // imu_date.linear_acceleration.z = speed_rec_pub->angular_z;
	        //    //角速度
          //   imu_date.angular_velocity.x = speed_rec_pub->linear_x; 
          //   imu_date.angular_velocity.y = speed_rec_pub->angular_y; 
          //   imu_date.angular_velocity.z = speed_rec_pub->angular_z;
      q=tf::createQuaternionFromYaw(speed_rec_pub->roll,speed_rec_pub->pitch,speed_rec_pub->yaw);
      
            speed_to_Offset(speed_rec_pub);
       ros::Time current_time;
        odom.header.frame_id="odom";
        odom.child_frame_id="base_footprint";
        odom.pose.pose.position.x = position_x;    //x位移 
        odom.pose.pose.position.y = position_y;     //y位移
        odom.pose.pose.position.z = 0.0; 
        odom.pose.pose.orientation=q;
        odom.header.stamp=nh.now();;


        odom.twist.twist.linear.x = linear_x;   //
        odom.twist.twist.linear.y = linear_y;
        odom.twist.twist.angular.z = linear_w;
        // rt_free(speed_rec_pub);
				// imu_date.header.stamp=nh.now();
        // imu_date.header.frame_id="base_imu_link";
        // imu_date.orientation.x=q.x;
        // imu_date.orientation.y=q.y;
        // imu_date.orientation.z=q.z;
        // imu_date.orientation.w=q.w;
        
        
    }
      
      // 更新话题内容
      xv.publish( &velX_tmp );
      od.publish(&odom);
      // im.publish(&imu_date);
      nh.spinOnce();
    }
}


static void enconer_thread_entry(void *parameter)
{
  tail_rot_speed_init();
}

int main(void)
{
mb_speed_sub = rt_mb_create("speed_sub",100,RT_IPC_FLAG_FIFO);
  if (mb_speed_sub != RT_NULL) {
   rt_kprintf("邮箱sub创建成功！\n\n");
  }
  PID_init(&(pid_andy.whlee1),1500,0,0);
  PID_init(&(pid_andy.whlee2),1,0,0);
  PID_init(&(pid_andy.whlee3),1,0,0);
  PID_init(&(pid_andy.whlee4),1,0,0);
	set_p_i_d(&(pid_andy.whlee1),1678,300,300);
  set_p_i_d(&(pid_andy.whlee2),1678,300,300);
  set_p_i_d(&(pid_andy.whlee3),1678,300,300);
  set_p_i_d(&(pid_andy.whlee4),1678,300,300);


rt_thread_t thread_enconer=rt_thread_create("zz",enconer_thread_entry,RT_NULL,1024,4,10);
    if(thread_enconer != RT_NULL)
    {
      rt_thread_startup(thread_enconer);
      rt_kprintf("enconer_thread_entry come\n");
    }
    else
    {
        rt_kprintf(" Failed to create thread enconer_thread_entry\n");
    }
   

    // 启动一个线程用来和 ROS 通信
    rt_thread_t thread = rt_thread_create("rosserial",  rosserial_thread_entry, RT_NULL, 2048, 4, 20);
    if(thread != RT_NULL)
    {
      rt_thread_startup(thread);
      rt_kprintf("[rosserial] New thread rosserial\n");
    }
    else
    {
        rt_kprintf("[rosserial] Failed to create thread rosserial\n");
    }
    return RT_EOK;
}
