#ifndef PID_H
#define PID_H
#include "Arduino.h"

class PID{
  private:
    double kp;
    double ki;
    double kd;
    double integral;
    double lastValue;
    long error;
    double delta;
    
    double endtime;
    
  public:
    PID(double kp,double ki,double kd,double Input); 
    void setPoint(double Target);
    void setTime(double endtime);
    long getError();
    double getTime();
    double target;
    void Update(double Input);
    long output();   
    
};


#endif
