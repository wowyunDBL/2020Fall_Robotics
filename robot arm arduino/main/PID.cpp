#include "Arduino.h"
#include "PID.h"

PID ::PID(double kp,double ki,double kd,double Input)
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->lastValue = Input;
  this->integral = 0;
  this->kp = kp;
}
void PID::setPoint(double Target)
{
  this->target = Target;
}
void PID::setTime(double endtime)
{
  this-> endtime = endtime;
}
double PID::getTime()
{
  return endtime;
}
long PID::getError()
{
  return error;
}
void PID::Update(double Input)
{
  this->error = target - Input;
  this->integral = error + integral;
  this->delta = Input - lastValue;
  this->lastValue = Input;
}
long PID::output()
{
  long result = kp * error + ki * integral + kd * delta;
  return result;
}
