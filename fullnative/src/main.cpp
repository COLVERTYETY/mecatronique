#include <Arduino.h> 
#include "header.h"

void rencodeur_callback() //RIGHT
{
  if(digitalRead(pin_rencodeur2)){
    rnbTic ++;
  }else
  {
    rnbTic --;
  }
    
}

void lencodeur_callback() //LEFT
{
  if(digitalRead(pin_lencodeur2)){
    lnbTic ++;
  }else
  {
    lnbTic --;
  }
    
}

ISR(TIMER1_COMPA_vect){
  //interrupt commands for TIMER 1 here, runs asynchonrously
  cli(); // interdire les interruptions
  rtickcopy = rnbTic;//RIGHT
  rnbTic = 0;
  ltickcopy = rnbTic;//LEFT
  lnbTic = 0;
  sei();//turn th einterrupt sback on

  re = rspeed - rtickcopy;//RIGHT 
  rde = re - rolde; 
  rE += re;  
  le = lspeed - ltickcopy; //LEFT  
  lde = le - lolde;
  lE += le;

  if (rE>512){rE = 512;}   //RIGHT
  if (rE<-512){rE = -512;}
  if (lE>512){lE = 512;}   //LEFT
  if (lE<-512){lE = -512;}

  rres_PID = re*kp + rde*kd + rE*ki; // RIGHT
  if (rres_PID>255){rres_PID = 255;}
  else if (rres_PID<0){rres_PID = 0;} 
  lres_PID = le*kp + lde*kd + lE*ki; // LEFT
  if (lres_PID>255){lres_PID = 255;}
  else if (lres_PID<0){lres_PID = 0;}

  analogWrite (pin_renable, rres_PID); // RIGHT
  analogWrite (pin_lenable, lres_PID); // LEFT
  rolde = re;//RIGHT  
  lolde = le;//LEFT
  Serial.print(ltickcopy);// on print 
  Serial.print(" ");// on print 
  Serial.println(rtickcopy);// on print 
}

void setup()  
{
  //serial
  Serial.begin(450000);

  //led
  pinMode(LED_BUILTIN, OUTPUT); 

  //ultrasound
  pinMode(pin_trigger, OUTPUT);
  digitalWrite(pin_trigger, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(pin_echo, INPUT);

  // put your setup code here, to run once: 
  pinMode(pin_renable, OUTPUT);  //RIGHT
  pinMode(pin_rdir1, OUTPUT);  
  pinMode(pin_rdir2, OUTPUT);  
  pinMode(pin_rencodeur, INPUT);  
  pinMode(pin_lenable, OUTPUT);  //LEFT
  pinMode(pin_ldir1, OUTPUT);  
  pinMode(pin_ldir2, OUTPUT);  
  pinMode(pin_lencodeur, INPUT);

  rdir = (rspeed>0);//RIGHT
  ldir = (lspeed>0);//LEFT
  digitalWrite(pin_renable, LOW); //RIGHT
  digitalWrite(pin_rdir1, rdir);  
  digitalWrite(pin_rdir2, !rdir); 
  digitalWrite(pin_lenable, LOW); //LEFT
  digitalWrite(pin_ldir1, ldir);  
  digitalWrite(pin_ldir2, !ldir); 



  // TIMER 1 pour 100 hz
  cli(); // on eteind les interruptions
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // on initialise un "buffer" pour le timer pour avoir la fréquence désiré
  OCR1A = 19999; // = 16000000 / (8 * 100) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 8 prescaler pour controler la vitesse du timer
  TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  // on allume le timer 1
  TIMSK1 |= (1 << OCIE1A);

  while (startdelai>0){ ///just a timer before start
    Serial.println(startdelai);
    digitalWrite(LED_BUILTIN, led_status);
    led_status = !led_status;
    delay(startdelai);
    startdelai -= 100;
  }
  Serial.println(" its go time !!");

  attachInterrupt(digitalPinToInterrupt(pin_rencodeur), rencodeur_callback, RISING); //RIGHT
  attachInterrupt(digitalPinToInterrupt(pin_lencodeur), lencodeur_callback, RISING); //LEFT
  sei(); // on rallume les interruptions
} 

void loop()  
{ 
  // measure dstance
  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigger, LOW);
  measure = pulseIn(pin_echo, HIGH, MEASURE_TIMEOUT);
  distance_mm = measure * SOUND_SPEED;
  
  //should we slow down ?
  if(distance_mm<slowdowndist){//yes
    cruisespeed -= dspeed;
  }else{//faster !!
    cruisespeed += dspeed;
  }
  // limit speed
  if(cruisespeed<minspeed){cruisespeed = minspeed;}
  else if(cruisespeed>maxspeed){cruisespeed = maxspeed;}

  //should we turn 
  if(distance_mm<turndist){//yes
    obsangle += 2*dangle;
  }else if(obsangle>0){// no correct
    obsangle -= 2*dangle;
  }else if(obsangle<0){//straight line
    obsangle = 0;
  }

  // point forward steering
  targetangle = wrap(objangle + obsangle); // calculate deviation angle
  alpha = wrap(targetangle - realangle); // calculate error
  rspeed = (int) (cruisespeed*(cos(alpha) + (K*sin(alpha))));
  lspeed = (int) (cruisespeed*(cos(alpha) - (K*sin(alpha))));

  // led 
  digitalWrite(LED_BUILTIN, led_status);//
  led_status = ! led_status;  
  // delay(100);  // this is really optional
} 

float wrap(float inangle){
  return atan2(sin(inangle),cos(inangle));
}