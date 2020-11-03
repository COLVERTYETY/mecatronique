#include <Arduino.h>
#include "header.h"

void encoderinterrupt()
{
    tick++; //increase tick count
}
void initializetimer()
{
    // TIMER 1 for interrupt frequency 50 Hz:
    cli();      // stop interrupts
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0;  // initialize counter value to 0
    // set compare match register for 50 Hz increments
    OCR1A = 24999; // = 16000000 / (64 * 50) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12, CS11 and CS10 bits for 64 prescaler
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei(); // allow interrupts
}
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
  attachInterrupt(digitalPinToInterrupt(pin_encoder),encoderinterrupt,RISING);
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


