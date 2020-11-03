
#ifndef HEADER_H_
#define HEADER_H_

#include <stdint.h>
//targets
volatile int tspeed=20;//target speed
volatile int tduration=1000;//target duration
volatile int oldtick;
volatile bool mainloop;
//ticks
volatile int tick;//tick count   volatile for easy interrupt acces
int N;// number of ticks per rotation

//errors as speeds
int E;//integrated error or cumulated error
int e;//error at this measurement

//pid
#define maxI 512
int Ival = 0;
int val;//output of pid calculations for further treatment.
int mapped=0;//0-255 pwm val of pid
int kp=3;//proportional wheight
int ki=3;//integral wheight

//physical measurements
int L;//distance between the wheels
int R;//radius of the wheels

//timers
//? http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html

//pins 
const uint8_t pin_encoder=3;//encoder pin
const uint8_t pin_dira=7;//dira on hbridge
const uint8_t pin_dirb=8;//dirb
volatile const uint8_t pin_pwm=11;//pwm pin for motor 1
//! 9 and 10 not available for pwm while using timer 1 are available: 5 6 11 3

//communication
#define baudrate 115200
bool led_status=true;


#endif