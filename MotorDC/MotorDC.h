#include "mbed.h"

#define MASK_SIGN 0x80000000

union speed_data{
  float value;
  int32_t dir;
};

/** Example MotorDC class.
 * @code
 * #include "mbed.h"
 * #include "MotorDC.h"
 *
 * MotorDC motor(PTC10,PTB3,PTB2);
 *
 * int main(){
 *
 *  motor = 0.5;  // motor.setSpeed(0.5);
 *  wait(2.0);
 *  motor = -0.5;
 *  wait(2.0);
 *  motor = 0;
 *
 *  while(true);
 *
 *
 * }
 * @endcode
 */

class MotorDC{
  public:
    /** Create and initializate MotorDC instance
    * @param pin_pwm A PwmOut pin, driving the H-bridge enable line to control the speed
    * @param pin_dir0 A DigitalOut, set direction motor pin0
    * @param pin_dir1 A DigitalOut, set direction motor pin1
    */
    MotorDC(PinName pin_pwm, PinName pin_dir0, PinName pin_dir1);
    /**
     * Set the speed of the Motor
     * @param fspeed speed between -1.0 and 1.0
     */
    void setSpeed(float fspeed);
    float operator =(float fspeed);

  private:

    speed_data speed;
    PwmOut pwmout;
    DigitalOut dir_motor0;
    DigitalOut dir_motor1;
};
