#include <math.h>
#include "DualVNH5019MotorShield.h"

// analog inputs
byte xpot = A2;
byte ypot = A3;

// remap joystick with idle position
// as origin 
float Jx;
float Jy;

// dimensions of ride-on
const float wheel_radius = 1.0;
const float width = 1.0;

// unincycle model & diff. drive model
float omega;
float velocity;
const float omega_const = 4;
const float vel_const = 1;
float vL;
float vR;

// position and heading variables
float theta_deg;
float theta_rad;
double position_scalar;

// state variables
float currVelocity;
float targetVelocity;
float targetSpeed1;  // Variable to store desired speed of motor 1 (left wheel)
float targetSpeed2;  // Variable to store desired speed of motor 2 (right wheel)
float error1;  // Variable used to store the difference between the current speed and the target speed of Motor 1
float error2;  // Variable used to store the difference between the current speed and the target speed of Motor 2
const float minSpeedDiff = 15;  // Minimum difference in target and current speed required for the wheels to change speed


DualVNH5019MotorShield m;

void setup() {
 m.init();

 m.setM1Speed(0);
 m.setM2Speed(0);
  
 Serial.begin(9600);
}

void loop() {
readJoystick();
genModel();
getSpeed();
useController();
//changeSpeed();

}

void readJoystick(){
 Jx = analogRead(A2);//(xpot);
 Jy = analogRead(A3);//(ypot);
Serial.print("Jx_in = ");
Serial.println(Jx);
Serial.print("Jy_in = ");
Serial.println(Jy);


 delay(10);
 Jx = map(Jx, 0, 1023, -400, 400);
 Jy = map(Jy, 0, 1023, -400, 400);
 
Serial.print("Jx = ");
Serial.println(Jx);
Serial.print("Jy = ");
Serial.println(Jy);
}

void genModel(){
  
 position_scalar = ((Jx * Jx) + (Jy * Jy)); //(float)sqrt(sq(Jx) + sq((float)Jy));
 position_scalar = pow(position_scalar, 0.5); 
 
 Serial.print("?? ? ");
 Serial.println(sq(Jx));
 Serial.print("position scalar = ");
 Serial.println(position_scalar);
 
 if(position_scalar < 25)
  theta_rad = 0;
 else
  theta_rad = atan2((double)Jy,(double)Jx);
 
 theta_deg = theta_rad * 4068 / 71;
 if((theta_deg > 0 && abs(theta_deg - 90) < 15))
  {
    theta_rad = PI / 2;
    theta_deg = 90;
  }
  else if((theta_deg < 0 && abs(theta_deg + 90) < 15))
  {
    theta_rad = (-1) * PI / 2;
    theta_deg = (-1) * 90;
  }
  else
    {
      theta_rad = theta_rad;
      theta_deg = theta_deg;
    }
    
 velocity = position_scalar * cos(theta_rad);
 
  if(position_scalar < 22)
    velocity = 0;
  else if(position_scalar >= 400)
    position_scalar = 400;
  
 velocity = position_scalar * cos(theta_rad);

 Serial.print("Angle is = ");
 Serial.println(theta_deg);
   
  if((velocity < 0) && (theta_deg < 0))
    theta_deg = theta_deg + 180;  
  else if((velocity < 0) && (theta_deg > 0))
    theta_deg = theta_deg - 180;
  
 if((theta_deg == 0) || (abs(theta_deg) < 15))
  omega = 0;
 else{
  omega = theta_deg * omega_const; 
 }

  Serial.print("Remap angle is = ");
  Serial.println(theta_deg);
  Serial.print("position scalar = ");
  Serial.println(position_scalar);
  Serial.print("velocity = ");
  Serial.println(velocity);
  
}

void getSpeed(){
  if(velocity < 0)
    velocity = map(velocity, -400, 400, -300, 300);  
  else
   velocity = velocity;     
/* 
  if((velocity < 0) && (theta_deg < 0))
    theta_deg = theta_deg + 180;  
  else if((velocity < 0) && (theta_deg > 0))
    theta_deg = theta_deg - 180;
*/
  vL = ((2 * velocity) - (omega * width)) / (2 * wheel_radius);
  vR = ((2 * velocity) + (omega * width)) / (2 * wheel_radius);

  Serial.print("Remap velocity = ");
  Serial.println(velocity);
  Serial.print("vL = ");
  Serial.println(vL);
  Serial.print("vR = ");
  Serial.println(vR);

  //Serial.print("Remap angle = ");
  //Serial.println(theta_deg);
}

void useController()
{
  error1 = vL - targetSpeed1;
  error2 = vR - targetSpeed2;
  
  if(error1 > minSpeedDiff)
  {
    vL = vL - minSpeedDiff;
    m.setM1Speed(vL);
  }
  else if(error1 < (-1 * minSpeedDiff))
  {
    vL = vL + minSpeedDiff;
    m.setM1Speed(vL);
  }
  else if(vL != targetSpeed1)
  {
    vL = vL - error1;
  }
  else
    vL = vL;

 
  if(error2 > minSpeedDiff)
  {
    vR = vR - minSpeedDiff;
    m.setM2Speed(vR);
  }
  else if(error2 < (-1 * minSpeedDiff))
  {
    vR = vR + minSpeedDiff;
    m.setM2Speed(vR);
  }
  else if(vR != targetSpeed2)
  {
    vR = vR - error2;
  }
  else
    vR = vR;
}

/*void changeSpeed(){
  m.setM1Speed(vL);
  m.setM2Speed(vR);
}
*/
