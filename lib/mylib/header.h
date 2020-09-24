
#include "stdint.h"

#ifndef HEADER_H_
#define HEADER_H_

//targets
volatile int tspeed; //target speed
//! currently tspeed is in radian per cycles
volatile int tduration; //target duration

//ticks
volatile int tick; //tick count   volatile for easy interrupt acces
int N;             // number of ticks per rotation

//errors as speeds
int de;       //delta error or derived error
int E;        //integrated error or cumulated error
int e;        //error at this measurement
int olde = 0; //old e necessary for calculating de

//pid
int val;              //output of pid calculations for further treatment.
int mapped;           //mapped value of the pid result
int maxpidout = 4000; //max value of pid for mapping
int kp = 1;           //proportional wheight
int ki = 0;           //integral wheight
int kd = 0;           //derivativ wheight

//physical measurements
int L; //distance between the wheels
int R; //radius of the wheels

//timers
//? http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html

//pins
const uint8_t pencoder = 3;      //encoder pin
const uint8_t penable = 7;       //enable pin
volatile const uint8_t pm1 = 11; //pwm pin for motor 1
//! 9 and 10 not available for pwm while using timer 1 are available: 5 6 11 3

//communication
int baudrate = 9600;

#endif