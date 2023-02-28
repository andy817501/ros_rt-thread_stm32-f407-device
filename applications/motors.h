#include <rtthread.h>

class MotorControl {
  public:
    //Var
    rt_uint32_t  maxSpd;
    float moveFactor;
    float turnFactor;

    uint8_t whlee1_on_off;
    uint8_t whlee2_on_off;
    uint8_t whlee3_on_off;
    uint8_t whlee4_on_off;
    MotorControl(int fl_for, int fl_back,
                 int fr_for, int fr_back);
    void initMotors();
    void rotateBot(int dir, float spd);
    void moveBot(float spd, float bias);
    void stopMotors();
    void move_andy(float speed,int channel);
    

  private:
    struct rt_device_pwm *pwm_dev;
    struct rt_device_pwm *pwm_dev1;
    //The pins
    int fl_for;
    int fl_back;
    int fr_for;
    int fr_back;
    int bl_for;
    int bl_back;
    int br_for;
    int br_back;
};
//     void (MotorControl::*APP_Operation[])(void) = { /* Sequence must be consistent with EnumState */

//     MotorControl::stopMotors

// };
