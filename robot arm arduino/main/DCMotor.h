
#define MOTOR_H

#include "Arduino.h"

class DCMotor{
  private:
    int Dir_pin = 3;
    int _Dir_pin = 2;
    int Pwm = 8 ;
    
  public:
    float phi;
    DCMotor();
    void initialDCMotor();
    void driveDCMotor(float duty);
};
