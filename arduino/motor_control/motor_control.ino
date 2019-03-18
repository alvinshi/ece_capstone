#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;
char input;
char BUFFER[5];

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
  Serial.println("Dual VNH5019 Motor Shield Started");
  md.init(); // Initialize motor driver connection
}

void loop() 
{
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'R') {
      int m1speed = intParse();
      Serial.print("Set M1 speed to: ");
      Serial.println(m1speed);
      md.setM1Speed(m1speed);
      stopIfFault();
    }
    else if (input == 'L') {
      int m2speed = intParse();
      Serial.print("Set M2 speed to: ");
      Serial.println(m2speed);
      md.setM2Speed(m2speed);
      stopIfFault();
    }
  }
}
