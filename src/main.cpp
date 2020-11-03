#include <Arduino.h>
#include "header.h"
#include "functions.h"
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(baudrate);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(pin_dira,OUTPUT);
  pinMode(pin_dirb,OUTPUT);
  pinMode(pin_pwm,OUTPUT);
  pinMode(pin_encoder,INPUT);
  initializetimer();
  attachInterrupt(digitalPinToInterrupt(pin_encoder),encoderinterrupts,RISING);
}
ISR(TIMER1_COMPA_vect){
   cli();
    if(tduration > 0)
    {
        tduration--;
        // adjust error
        e = (tspeed) - (tick); //should be devided by 0.1s for time and mult by R for distance
        E = E + e;
        Ival = ki*E;
        Ival = (Ival < maxI) ? Ival : maxI; //Ival saturation to prevent windup, not really windup though
        //calculate pid values
        val = kp * e + Ival;        //calculate pid
        mapped = (val < 255) ? val : 255;
        mapped = (val < 0) ? 0 : val;
        analogWrite(pin_pwm, mapped);                //write the pid to the pwm
        //setup for next cycle
    }
    else
    {
        //TIMSK1 &= ~(1 << OCIE1A); //disable timer compare interupt
    }
    oldtick = tick;
    tick = 0;
    mainloop=true;
    sei();
}
void loop()
{
  // put your main code here, to run repeatedly:
  if(mainloop){
    Serial.println(oldtick);
    mainloop = false;
    led_status = !led_status;
    digitalWrite(LED_BUILTIN,led_status);
  }
}