#ifndef _andysensor_H_
#define _andysensor_H_


struct andysensor
{

    float speed1;
    float speed2;
    float speed3; 
    float speed4;
    float angular_x;
    float angular_y;
    float angular_z;
    float linear_x;
    float linear_y;
    float linear_z;
    float   pitch;
    float   roll;
    float   yaw;  
    float  max_distance1=0;
    float  max_distance2=0;
    float  max_distance3=0;
    float  max_distance4=0;
};
typedef union
{
  float f;
  int i;
}type_cast_t;



extern "C"
{
#include "sensor_inven_mpu6xxx.h"
#include "mpu6xxx.h"
#include "inv_mpu.h"
#include "curve_protocol.h"
void  tail_rot_speed_init();
void mpu6050_init_andy();
void MPU6050_DataGet(void);


}
#endif