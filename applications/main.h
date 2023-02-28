#ifndef _main_H_
#define _main_H_
#include <rtthread.h>
#include <rtdevice.h>

#define WHEEL_K        0.16        //则表示以车为中心轮子X，Y坐标的绝对值之和
#define WHEEL_R       0.0375
#define WHEEL_PI       3.1415926
#define WHEEL_RATIO    30



extern rt_mailbox_t mb_speed_sub;

struct msg_speed
{
    float v1;
    float v2;
    float v3;
    float v4;
};
#endif