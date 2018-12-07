#include <DualVNH5019MotorShield.h>
#include <math.h>

// analog pins
  byte x_axis_potentiometer;
  byte y_axis_potentiometer;
  const byte pot_pin = A4;
  
// variables that store (x,y) coordinate of joystick location and potPin value
  float x_coordinate;
  float y_coordinate;
  int pot_value;
  float max_speed;
// dimensions of ride-on
// set to 1.0 but can be changed
// for different ride-on shapes/sizes
  const float wheel_radius = 1.0;
  const float width = 1.0;

// variables of the mathematical model used
  float turning_speed;
  float velocity;
  const float turning_rate = 4.0;
  const float velocity_rate = 0.45;
  
  float vL = 0;
  float vR = 0; 
  float theta_rad;
  float theta_deg;
  float radius;

// variables for tracking the current
// speed and the desired speed
  float target_L;
  float target_R;
  float error_L;
  float error_R;

// variables for creating 
// a controller that improves
// response speed
  const float max_speed_jump = 60;
  const float min_speed_diff = 135;
 
  DualVNH5019MotorShield m;  

void setup() {
  m.init();
  m.setM1Speed(0);
  m.setM2Speed(0);
  Serial.begin(9600);
}

void loop() {
  readSpeedGovernor();
  readJoystick();
  createMathModel();
  getSpeed();
  //useSpeedController1();
  useSpeedController2();
}

void readSpeedGovernor() {
  pot_value = analogRead(pot_pin);
  max_speed = map(pot_value, 0, 1023, 0, 400);
  Serial.println("----------------------------------------------------------");
  Serial.print("pot value");
  Serial.print("\t");
  Serial.print("max speed");
  Serial.println();
  Serial.print(pot_value);
  Serial.print("\t");
  Serial.print("\t");
  Serial.println(max_speed);
  Serial.println();
  Serial.println();
}

void readJoystick() {
  x_coordinate = analogRead(A2);
  y_coordinate = analogRead(A3);
  Serial.println("----------------------------------------------------------");
  Serial.print("(x,y) = ");
  Serial.print("\t");
  Serial.print("(");
  Serial.print(x_coordinate);
  Serial.print(",");
  Serial.print(y_coordinate);
  Serial.println(")");

  delay(10);
  x_coordinate = map(x_coordinate, 0, 1023, -max_speed, max_speed);
  y_coordinate = map(y_coordinate, 0, 1023, -max_speed, max_speed);
  Serial.print("(x_map,y_map) = ");
  Serial.print("(");
  Serial.print(x_coordinate);
  Serial.print(",");
  Serial.print(y_coordinate);
  Serial.print(")");
  Serial.println();
  Serial.println(); 
}

void createMathModel() {
  radius = sqrt(sq(x_coordinate) + sq(y_coordinate));    
    if(radius < 28) {
      theta_rad = 0;
    }
    else {
      theta_rad = atan2(y_coordinate,x_coordinate);
    }
    if(radius < 27) {
      radius = 0;
    }
    else if(radius >= 400) {
      radius = 400;
    }
    
  theta_deg = theta_rad * 4068/71;
  velocity = radius * cos(theta_rad);
  Serial.println("----------------------------------------------------------");
  Serial.print("angle = ");
  Serial.print("\t");
  Serial.println(theta_deg);
    if((theta_deg > 75) && (theta_deg < 105)) {
      theta_rad = PI/2;
      theta_deg = 90;
      velocity = 0;
    }
    else if((theta_deg > -105) && (theta_deg < -75)) {
      theta_rad = -PI/2;
      theta_deg = -90;
      velocity = 0;
    }
    if((velocity < 0) && (theta_deg < 0)) {
      theta_deg = theta_deg + 180;
    }
    else if((velocity < 0) && (theta_deg > 0)) {
      theta_deg = theta_deg - 180;    
    }
    if((theta_deg == 0) || (abs(theta_deg) < 15)) {
      turning_speed = 0;
    }
    else {
      turning_speed = theta_deg * turning_rate;
    }
  Serial.println("----------------------------------------------------------");
  Serial.print("remap angle");
  Serial.print("\t");
  Serial.print("radius");
  Serial.print("\t");
  Serial.print("velocity");
  Serial.println();
  Serial.print(theta_deg);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(radius);
  Serial.print("\t");
  Serial.print(velocity);
  Serial.println();
}

void getSpeed() {
      if(velocity < 0) {
      velocity = map(velocity, -max_speed, max_speed, -0.5*max_speed, 0.5*max_speed);
    }
    else {
      velocity = velocity;
    }
  target_L = ((2 * velocity) - (turning_speed * width)) / (2 * wheel_radius);
  target_R = ((2 * velocity) + (turning_speed * width)) / (2 * wheel_radius);
}

void useSpeedController1() {
    if(target_L > 400) {
      target_L = 400;
    }
    else if(target_L < -400) {
      target_L = -400;
    }
    if(target_R > 400) {
      target_R = 400;
    }
    else if(target_R < -400) {
      target_R = -400;
    }
  
  error_L = vL - target_L;
  error_R = vR - target_R;
    if(error_L > min_speed_diff) {
      vL = vL - min_speed_diff;
    }
    else if(error_L < (-1 * min_speed_diff)) {
      vL = vL + min_speed_diff;
    }
    else if(vL != target_L) {
      vL = vL - error_L;
    }
    else {
      vL = vL;
    } 
  m.setM1Speed(vR);
    if(error_R > min_speed_diff) {
      vR = vR - min_speed_diff;
    }
    else if(error_R < (-1 * min_speed_diff)) {
      vR = vR + min_speed_diff;
    }
    else if(vR != target_R) {
      vR = vR - error_R;
    }
    else {
      vR = vR;
    }
  m.setM2Speed(vL);
    if((abs(vL) < 15) && (abs(target_L) < 15)) {
      m.setM1Speed(0);
    }
    if((abs(vR) < 15) && (abs(target_R) < 15)) {
      m.setM2Speed(0); 
    } 
  Serial.println("----------------------------------------------------------");
  Serial.print("remap velocity");
  Serial.print("\t");
  Serial.print("vL");
  Serial.print("\t");
  Serial.print("vR");
  Serial.print("\t");
  Serial.print("target_L");
  Serial.print("\t");
  Serial.print("target_R");
  Serial.println();
  
  Serial.print(velocity);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(vL);
  Serial.print("\t");
  Serial.print(vR);
  Serial.print("\t");
  Serial.print(target_L);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(target_R);
  Serial.println();
  Serial.println();
}

void useSpeedController2() {
 
  error_L = vL - target_L;
  error_R = vR - target_R;
    
    if((error_L * velocity_rate) >= max_speed_jump) // Checks to see if the product of the acceleration rate and the error are larger than the max_speed_jump value; if so then the change in speed is capped by the value of max_speed_jump
      {
        vL = vL - max_speed_jump;
        //m.setM2Speed(vL);
      }
      else if((error_L * velocity_rate) <= (-1 * max_speed_jump))  // Checks to see if the product of the acceleration rate and the error are smaller than the negative max_speed_jump value; if so then the change in speed is capped by the value of max_speed_jump
      {
        vL = vL + max_speed_jump;
       // m.setM2Speed(vL);
      }
      else if(vL != target_L)  // Accelerate either in the positive or negative direction for motor 1 based on the sign of the error
      {
        vL = vL - (velocity_rate * error_L);
       // m.setM2Speed(vL);
      }
    else  // Do nothing if the current speed of motor 1 is the same as the target speed
      vL = vL;
  m.setM1Speed(vR);
    if((error_R * velocity_rate) >= max_speed_jump) // Checks to see how large of a speed change will occur for motor 2 and limits the max change to 90 per increment
     {
        vR = vR - max_speed_jump;
        //m.setM1Speed(vR);
     }
     else if((error_R * velocity_rate) <= (-1 * max_speed_jump))
     {
        vR = vR + max_speed_jump;
        //m.setM1Speed(vR);
     }
   
    else if(vR != target_R) // Accelerate either in the positive or negative direction for motor 2 based on the sign of the error
    {
      vR = vR - (velocity_rate * error_R);
     // m.setM1Speed(vR);
    }
    else
      vR = vR;  // Do nothing if the current speed of motor 2 is the same as the target speed      
  m.setM2Speed(vL);

    if((abs(vL) < 15) && (abs(target_L) < 15)) {
      m.setM1Speed(0);
    }
    if((abs(vR) < 15) && (abs(target_R) < 15)) {
      m.setM2Speed(0); 
}
}
