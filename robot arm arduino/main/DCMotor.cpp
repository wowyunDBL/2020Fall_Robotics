#include "Arduino.h"
#include "DCMotor.h"
#define dutyTolerance 15.0
DCMotor::DCMotor(){
}

void DCMotor::initialDCMotor(){
  pinMode(Dir_pin,OUTPUT);
  pinMode(_Dir_pin,OUTPUT);
  pinMode(Pwm,OUTPUT);  
  digitalWrite(Dir_pin, HIGH);
  digitalWrite(_Dir_pin, LOW);
  analogWrite(Pwm,0);;
}
void DCMotor::driveDCMotor(float duty){
//  Serial.println("Motor Driving...");
  if (abs(duty) < dutyTolerance){
    duty = 0; 
  }
  
  if (duty > 0)
  {
    digitalWrite(Dir_pin, HIGH);
    digitalWrite(_Dir_pin, LOW);
    Serial.println("FW");
  }
  else if (duty < 0)
  {
    digitalWrite(Dir_pin, LOW);
    digitalWrite(_Dir_pin, HIGH);
    Serial.println("BW");
  }
  else
  {
    digitalWrite(Dir_pin, HIGH);
    digitalWrite(_Dir_pin, HIGH);
  }
  if (duty > 100)
  {
    duty = 100;
  }
  analogWrite(Pwm,duty*255.0/100.0);
}
