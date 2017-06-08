#include "MotorDC.h"

MotorDC::MotorDC(PinName pin_pwm,PinName pin_dir0, PinName pin_dir1):

    pwmout(pin_pwm),dir_motor0(pin_dir0,0),dir_motor1(pin_dir1,0){
    pwmout = 0;
    pwmout.period(0.00005f);

}

void MotorDC::setSpeed(float fspeed){

  speed.value = fspeed;
  pwmout = fabs(fspeed);
  dir_motor0 = (speed.dir & MASK_SIGN) >> 31;
  dir_motor1 = !dir_motor0;

}

float MotorDC::operator =(float fspeed){

  this->setSpeed(fspeed);

  return fspeed;
}
