#include <Arduino.h>
#ifndef HEADER_H_
    #include "header.h"
#endif


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
    olde=0;//this is a useless comment wow
  }
}