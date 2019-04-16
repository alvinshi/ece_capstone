#include "DualVNH5019MotorShield.h"
#include <Stepper.h>

DualVNH5019MotorShield md;
char input;
char BUFFER[5];

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int rolePerMinute = 17;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm

Stepper myStepper(stepsPerRevolution, 3, 11, 5, 13);

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

int intParse()
{    
  for(byte i = 0; i < 4; i++) {
    while (Serial.available() == 0); //Blocking read
    BUFFER[i] = Serial.read(); 
  } 
  BUFFER[4] = 0;
  return atoi(BUFFER);
}

void setup() 
{
  Serial.begin(9600);
  md.init(); // Initialize motor driver connection
  Serial.println("Dual VNH5019 Motor Shield Started");
  myStepper.setSpeed(rolePerMinute);
  Serial.println("Step Motor Started");
}

void loop() 
{
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'M') {
      int m1speed = intParse();
      Serial.print("Set M1 speed to: ");
      Serial.println(m1speed);
      int m2speed = intParse();
      Serial.print("Set M2 speed to: ");
      Serial.println(m2speed);
      md.setSpeeds(m1speed, m2speed);
      stopIfFault();
    }
    else if (input == 'D') {
      myStepper.step(stepsPerRevolution);
      delay(200);
      myStepper.step(stepsPerRevolution);
      delay(200);
      myStepper.step(stepsPerRevolution);
      delay(200);
      myStepper.step(stepsPerRevolution);
      delay(200);
      myStepper.step(stepsPerRevolution);
      delay(200);
    }
    else if (input == 'U') {
      myStepper.step(-stepsPerRevolution);
      delay(200);
      myStepper.step(-stepsPerRevolution);
      delay(200);
      myStepper.step(-stepsPerRevolution);
      delay(200);
      myStepper.step(-stepsPerRevolution);
      delay(200);
      myStepper.step(-stepsPerRevolution);
      delay(200);
    }
  }
}
