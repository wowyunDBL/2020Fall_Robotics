
#include "PID.h"
#include "DCMotor.h"
#include <Wire.h>
#include <Servo.h>


//PID parameter
  //===============================================
  #define kp 1.5 //2.5
  #define ki 0.0000//0.00
  #define kd 0.030 //2.5
  #define tolerance 5
  int const Trigger = 24;
  bool const trig = 1;
  bool const axis1_boost = 1;
  int const boost_iter = 350;
  int const boost_tolerance = 15;
  #define motor2_Pin 10
  #define motor3_Pin 11
  //===============================================


  
//build Motors
  //=============================================
  DCMotor motor1 = DCMotor();
  bool boostFlag = 0;
  Servo motor2;
  Servo motor3;
  //=============================================

//build PID
  //=============================================
  PID pid1(kp, ki, kd, motor1.phi);
  //=============================================

//other
  //============================================
  long counter_L = 0;
  //============================================

long long int iter = 0;
long long int tik = millis();
int tiktok;
double last_phi = 0;
int received_iter = 0;

void setup() {
  Serial.begin(57600);
  Serial.flush();
  SetEncoder();
  pinMode(Trigger, INPUT);
  while ((digitalRead(Trigger)) && trig){}
  
  
  motor1.initialDCMotor();
  motor2.attach(motor2_Pin);
  motor3.attach(motor3_Pin);
  received_iter = 0;
  
  // // Set up the interrupt pin, its set as active high, push-pull
  // pinMode(intPin, INPUT);
  // digitalWrite(intPin, LOW);
  
}



void loop() {
  iter++;

  
  if (Serial.available()){
    getDesiredMotorAngle(&pid1, &motor2, &motor3);
    //Serial.println("read robot: " + String(robot1._position[0])+" "+String(robot1._position[1])+" 0 "+String(robot1.yaw));   
  }
  
  if (iter%10==0){
    getEncoder(&counter_L,&motor1);  
  }
  
  pid1.Update(motor1.phi);
  float duty = pid1.output();
//  Serial.print("Counter: ");
//  Serial.print(counter_L);
  Serial.print("phi: ");
  Serial.print(motor1.phi);
  Serial.print(", Duty: ");
  Serial.print(duty);
  Serial.print(", error: ");
  Serial.println(pid1.getError());
//  Serial.print(", target: ");
//  Serial.println(pid1.target);
  


  if (abs(pid1.getError()) < tolerance)
  {
    duty = 0;
    Serial.println("In Tolerance..");
  }  
   
  if (digitalRead(Trigger)  && trig){
    Serial.println("Trig Stop..");
    duty=0;
  }
  if (((received_iter > boost_iter || boostFlag) && duty != 0 && abs(pid1.getError()) > boost_tolerance) && axis1_boost){
    if (duty<0){
      duty = -100;
    }else{
      duty = 100;
    }
    
    Serial.println("axis 1 boost!!");
    boostFlag = 1;
    received_iter = 0;
  }else{
    boostFlag = 0;
  }
  received_iter += 1;
  
//  Serial.println(duty);
  motor1.driveDCMotor(duty);
  
}

void getDesiredMotorAngle(PID *pid1, Servo *motor2, Servo *motor3){
  String str = Serial.readString();
  Serial.flush();
  
  // str format = "XXXX,XXX,XXX"
  int ind[3] = {4,8,12};
  
  String motor1_deg_str, motor2_deg_str, motor3_deg_str;
  double motor1_deg, motor2_deg, motor3_deg;
  
  motor1_deg_str = str.substring(0,ind[0]);
  motor2_deg_str = str.substring(ind[0]+1,ind[1]);
  motor3_deg_str = str.substring(ind[1]+1,ind[2]);

  motor1_deg = motor1_deg_str.toDouble();
  motor2_deg = motor2_deg_str.toDouble();
  motor3_deg = motor3_deg_str.toDouble();

  Serial.println("direct read: "+String(motor1_deg)+","+String(motor2_deg)+","+String(motor3_deg));
  if (received_iter > 10 && pid1->getError() > boost_tolerance){
      // pass
  }else{
    received_iter = 0;
  }
  
  boostFlag = 0;
  
  pid1->target = motor1_deg;
  motor2->write(motor2_deg);
  motor3->write(motor3_deg);
  
  
}
