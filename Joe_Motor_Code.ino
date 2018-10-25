#include "DualVNH5019MotorShield.h"


//---for the 4 main switches of the joystick and the potentiometer---
const int forward = 11; 
const int reverse = 13;
const int left = 3;
const int right = 5;

//---for the potentiometer
const byte potPin = A2;         // set potentiometer pin to Analog 0
int potValue;                   // defining the usable potentiometer reading

//---for global variables, constants---
float maxSpeed; // Variable to store the value of the max speed
const float diagRate = 0.35; // Variable used to set the turning radius of diagonal turns
float error1;  // Variable used to store the difference between the current speed and the target speed of Motor 1
float error2;  // Variable used to store the difference between the current speed and the target speed of Motor 2

const float accelRate = .15;  // Rate for change in acceleration 
const float maxSpeedJump = 60;  // Largest increment that the current speed can change per unit time on either of the two motors

float currSpeed1 = 0; // Variable to store current speed of motor 1
float currSpeed2 = 0; // Variable to store current speed of motor 2
float targetSpeed1;  // Variable to store desired speed of motor 1
float targetSpeed2;  // Variable to store desired speed of motor 2

byte forwardState;   // Record state of forward switch
byte reverseState;  // Record state of reverse switch
byte leftState;    // Record state of left switch
byte rightState;  // Record state of right switch

DualVNH5019MotorShield m;

void setup() {
  m.init();

  pinMode(forward,INPUT_PULLUP);  // Pin connected to 5V through an internal pullup resistor(default reading is HIGH)
  pinMode(reverse,INPUT_PULLUP);  // Same thing as above: when switch is pressed, it connects to ground (and reads LOW)
  pinMode(left,INPUT_PULLUP); // Same thing as above
  pinMode(right,INPUT_PULLUP);  // Same thing as above

  m.setM1Speed(0);  // Set motor 1 to full coast(zero speed) when no switch is pressed
  m.setM2Speed(0);  // Set motor 2 to full coast(zero speed)

  Serial.begin(9600); // Open the serial port at 9600 bps
}

void loop() {

  readPotentiometer();
  checkSwitches();  // Checks which of the four switches was pressed on the joystick
  changeSpeed();  // Changes the speed of each motor by comparing the current and target speeds 
}

// Value read from potentiometer
// ranging from 0 to 1023
void readPotentiometer(){
  
  potValue = analogRead(potPin);
  maxSpeed = map(potValue, 0, 1023, 0, 400);

  Serial.print("Potentiomer reading is = ");
  Serial.println(potValue);
  Serial.print("Max speed is  = ");
  Serial.println(maxSpeed);
}

/* checks to see which switch is being pressed
*  stores the target speed of each motor
*  and prints the direction
 */
void checkSwitches(){
     
  forwardState = digitalRead(forward);  // HIGH is the same as off(depressed) and LOW is the same as on(pressed)
  reverseState = digitalRead(reverse);
  leftState = digitalRead(left);
  rightState = digitalRead(right);

   if(!digitalRead(left) && digitalRead(forward) && digitalRead(reverse) && digitalRead(right)) // Left turn
  {
    targetSpeed1 = -1 * maxSpeed;
    targetSpeed2 = maxSpeed;
    Serial.println("left");
  }
   else if(!digitalRead(left) && !digitalRead(forward) && digitalRead(reverse) && digitalRead(right))//(leftState == LOW && forwardState == LOW && reverseState == HIGH && rightState == HIGH) // Up-Left turn
    {
      targetSpeed1 = maxSpeed * diagRate;
      targetSpeed2 = maxSpeed;
      Serial.println("up_left");
    }
   else if(!digitalRead(left) && digitalRead(forward) && !digitalRead(reverse) && digitalRead(right))//(leftState == LOW && forwardState == HIGH && reverseState == LOW && rightState == HIGH) // Down-Left turn
    {
      targetSpeed1 = -1 * maxSpeed * diagRate;
      targetSpeed2 = -1 * maxSpeed;
      Serial.println("down_left");
    }
   else if(digitalRead(left) && digitalRead(forward) && digitalRead(reverse) && !digitalRead(right))//(leftState == HIGH && forwardState == HIGH && reverseState == HIGH && rightState == LOW)  // Right turn
    {
      targetSpeed1 = maxSpeed;
      targetSpeed2 = -1 * maxSpeed;
      Serial.println("right");
    } 
  else if(digitalRead(left) && !digitalRead(forward) && digitalRead(reverse) && !digitalRead(right))//(leftState == HIGH && forwardState == LOW && reverseState == HIGH && rightState == LOW)  // Up-Right turn
    {
      targetSpeed1 = maxSpeed;
      targetSpeed2 = maxSpeed * diagRate;
      Serial.println("up_right");
    }
  else if(digitalRead(left) && digitalRead(forward) && !digitalRead(reverse) && !digitalRead(right))//(leftState == HIGH && forwardState == HIGH && reverseState == LOW && rightState == LOW)  // Down-Right turn
    {
      targetSpeed1 = -1 * maxSpeed;
      targetSpeed2 = -1 * maxSpeed * diagRate;
      Serial.println("down_right");
    }
  else if(digitalRead(left) && !digitalRead(forward) && digitalRead(reverse) && digitalRead(right))//(leftState == HIGH && forwardState == LOW && reverseState == HIGH && rightState == HIGH) // Forwards(Up)
    {
      targetSpeed1 = maxSpeed;
      targetSpeed2 = maxSpeed;
      Serial.println("forward");
    }
    else if(digitalRead(left) && digitalRead(forward) && !digitalRead(reverse) && digitalRead(right))//(leftState == HIGH && forwardState == HIGH && reverseState == LOW && rightState == HIGH) // Reverse(Down)
    {
      targetSpeed1 = -1 * maxSpeed;
      targetSpeed2 = -1 * maxSpeed;
      Serial.println("reverse");
    }
    else  // Idle state and all other exceptions are taken care of by this
    {
      targetSpeed1 = 0;
      targetSpeed2 = 0;
     // m.setM1Speed(0); //m.setM1Brake(0);
     // m.setM2Speed(0); // m.setM2Brake(0);
      Serial.println("else");
    }

  Serial.print("Target Speed of M1 = ");
  Serial.println(targetSpeed1);
  Serial.print("Target Speed of M2 = ");
  Serial.println(targetSpeed2);
}

//---changes the speed of each motor based on the previous direction
void changeSpeed(){
  
     /* if(currSpeed1 < targetSpeed1)  // Accelerate in the positive direction if current speed of motor 1 is smaller than the target speed
        {
         currSpeed1 += accelRate;
         m.setM1Speed(currSpeed1);
        }
      else if(targetSpeed1 < currSpeed1)  // Accelerate in the negative direction if current speed of motor 1 is greater than the target speed
        {
         currSpeed1 -= accelRate;
         m.setM1Speed(currSpeed1);
        }
      else  // Do nothing if the current speed of motor 1 is the same as the target speed
        currSpeed1 = currSpeed1;     

      if(currSpeed2 < targetSpeed2)  // Accelerate in the positive direction if current speed of motor 2 is smaller than the target speed
        {
           currSpeed2 += accelRate;
           m.setM2Speed(currSpeed2);
        }
      else if(targetSpeed2 < currSpeed2)  // Accelerate in the negative direction if current speed of motor 2 is greater than the target speed
        {
          currSpeed2 -= accelRate;
          m.setM2Speed(currSpeed2);
        }
      else  // Do nothing if the current speed of motor 2 is the same as the target speed
        currSpeed2 = currSpeed2;

  Serial.print("Current Speed of M1 = ");
  Serial.println(currSpeed1);
  Serial.print("Current Speed of M2 = ");
  Serial.println(currSpeed2);
  */
  error1 = currSpeed1 - targetSpeed1;
  error2 = currSpeed2 - targetSpeed2;
    
  if((error1 * accelRate) >= maxSpeedJump) // Checks to see if the product of the acceleration rate and the error are larger than the maxSpeedJump value; if so then the change in speed is capped by the value of maxSpeedJump
    {
      currSpeed1 = currSpeed1 - maxSpeedJump;
      m.setM1Speed(currSpeed1);
    }
    else if((error1 * accelRate) <= (-1 * maxSpeedJump))  // Checks to see if the product of the acceleration rate and the error are smaller than the negative maxSpeedJump value; if so then the change in speed is capped by the value of maxSpeedJump
    {
      currSpeed1 = currSpeed1 + maxSpeedJump;
      m.setM1Speed(currSpeed1);
    }
    else if(currSpeed1 != targetSpeed1)  // Accelerate either in the positive or negative direction for motor 1 based on the sign of the error
    {
      currSpeed1 = currSpeed1 - (accelRate * error1);
      m.setM1Speed(currSpeed1);
    }
  else  // Do nothing if the current speed of motor 1 is the same as the target speed
    currSpeed1 = currSpeed1;

   if((error2 * accelRate) >= maxSpeedJump) // Checks to see how large of a speed change will occur for motor 2 and limits the max change to 90 per increment
   {
      currSpeed2 = currSpeed2 - maxSpeedJump;
      m.setM2Speed(currSpeed2);
   }
   else if((error2 * accelRate) <= (-1 * maxSpeedJump))
   {
      currSpeed2 = currSpeed2 + maxSpeedJump;
      m.setM2Speed(currSpeed2);
   }
 
  else if(currSpeed2 != targetSpeed2) // Accelerate either in the positive or negative direction for motor 2 based on the sign of the error
  {
    currSpeed2 = currSpeed2 - (accelRate * error2);
    m.setM2Speed(currSpeed2);
  }
  else
    currSpeed2 = currSpeed2;  // Do nothing if the current speed of motor 2 is the same as the target speed
            
}

  








