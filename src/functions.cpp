// #include <header.h>
// #include <Arduino.h>
#ifndef HEADER_H_
    #include <header.h>
#endif
void encoderinterrupt(){
    tick++;//increase tick count
}

void initializetimer(){
    // TIMER 1 for interrupt frequency 10 Hz:
    cli(); // stop interrupts
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1  = 0; // initialize counter value to 0
    // set compare match register for 10 Hz increments
    OCR1A = 24999; // = 16000000 / (64 * 10) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12, CS11 and CS10 bits for 64 prescaler
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei(); // allow interrupts
}

void timerinterrupt(){
    if(tduration>0){
        tduration--;
        // adjust error
        e = (tspeed)-(tick);//should be devided by 0.1s for time and mult by R for distance
        de = e - olde;
        E = E + e;
        //calculate pid values
        val = kp*e+ki*E+kd*de;//calculate pid
        mapped = map(val,0,maxpidout,0,255);//mapping from pidvals to pwm vals
        analogWrite(pm1,mapped);//write the pid to the pwm
        //setup for next cycle
        tick = 0;
        olde =e;
    } else{
        TIMSK1 &= ~(1 << OCIE1A);//disable timer compare interupt
    }
}