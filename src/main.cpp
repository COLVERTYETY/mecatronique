#include <Arduino.h>
#include <functions.cpp>
#include <header.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(baudrate);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0)  {
    int incomingData= Serial.parseInt();
    tspeed = incomingData;//set the target speed
    tduration = 20;//set the target duration
    E=0;
    olde=0;
  }
}