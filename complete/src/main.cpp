#include <Arduino.h> 
#include "header.h"

float wrap(float inangle){
  return atan2(sin(inangle),cos(inangle));
}

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
  ltickcopy = lnbTic;//LEFT
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
  lres_PID = le*kp + lde*kd + lE*ki; // LEFT
  //according to speed change dir so pid res always positiv
    if( rdir != (rres_PID>0)){
      rdir = (rres_PID>0);//RIGHT
      lres_PID = abs(lres_PID);
      digitalWrite(pin_rdir1, rdir);  //RIGHT
      digitalWrite(pin_rdir2, !rdir);
    }

    if( ldir != (lres_PID>0)){
      ldir = (lres_PID>0);//LEFT
      lres_PID = abs(lres_PID);
      digitalWrite(pin_ldir1, ldir);  //LEFT
      digitalWrite(pin_ldir2, !ldir);
    }
  if (rres_PID>255){rres_PID = 255;}//saturate pid results
  else if (rres_PID<0){rres_PID = 0;} 
  if (lres_PID>255){lres_PID = 255;}
  else if (lres_PID<0){lres_PID = 0;}

  analogWrite (pin_renable, rres_PID); // RIGHT
  analogWrite (pin_lenable, lres_PID); // LEFT
  rolde = re;//RIGHT  
  lolde = le;//LEFT

  //calculate angle or displacement
  rd = (rtickcopy / ratioencoder) * wheelperimeter;// mm/cycle
  ld = (ltickcopy / ratioencoder) * wheelperimeter;
  vcenter = (rd + ld)/2.00;
  realangle += (rd - ld)/entraxe; //calculate new angle
  X+=vcenter*cos(realangle);//update pos
  Y+=vcenter*sin(realangle);// simple trigonometrics
}

void setup()  
{
  //serial
  Serial.begin(115200);

  //led
  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(led_R, OUTPUT);
  pinMode(led_J, OUTPUT);
  pinMode(led_V, OUTPUT);
  digitalWrite(led_R, HIGH);
  digitalWrite(led_J, HIGH);
  digitalWrite(led_V, HIGH);
  //ultrasound
  for(int i =0;i<sensorarraylength;i++){
    sensorarray[i].init();
  }
  // init pins yes
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

  while (startdelai>50){ ///just a timer before start
    Serial.println(startdelai);
    digitalWrite(LED_BUILTIN, led_status);
    led_status = !led_status;
    rstatus = ! rstatus;
    digitalWrite( led_R, rstatus);
    delay((int)startdelai);
    startdelai *= 0.9;
  }
  rstatus = true;
  digitalWrite(led_R, rstatus);
  Serial.println(" its go time !!");

  // TIMER 1 pour 80 hz
  cli(); // on eteind les interruptions
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // on initialise un "buffer" pour le timer pour avoir la fréquence désiré
  OCR1A = 24999; // = 16000000 / (8 * 100) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 8 prescaler pour controler la vitesse du timer
  TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  // on allume le timer 1
  TIMSK1 |= (1 << OCIE1A);
  sei();

  attachInterrupt(digitalPinToInterrupt(pin_rencodeur), rencodeur_callback, RISING); //RIGHT
  attachInterrupt(digitalPinToInterrupt(pin_lencodeur), lencodeur_callback, RISING); //LEFT

} 

void loop()  
{
  //are we there yet?
  distsquared = (X2-X)*(X2-X) + (Y2-Y)*(Y2-Y);
  if(distsquared<=epsilon){//yes we have arrived
    vstatus = false;
    digitalWrite(led_V, vstatus);
    digitalWrite(pin_lenable, LOW);
    digitalWrite(pin_renable, LOW);
    cruisespeed = 0.00;
  }else{// still have to go on...
    // measure dstance
    Xsensor=0;
    Ysensor=0;
    float temp;
    for(int i =0;i<sensorarraylength;i++){
      temp = sensorarray[i].measure();
      delay(100);
      if(temp>=0.01){
        temp = (kw/(temp*temp));//so that we don t do this twice
        Xsensor+=temp*cos(sensorarray[i].angle);
        Ysensor+=temp*sin(sensorarray[i].angle);
      }
    }
    Serial.println("");
    distance_mm = Xsensor*Xsensor + Ysensor*Ysensor; //! is squared
    //should we slow down ?
    digitalWrite(led_J,distance_mm == 0);//show that an obstacle has been detected
    if(distance_mm<slowdowndist && distance_mm != 0.00){//yes
    cruisespeed -= dspeed; //deceleration
    }else{//faster !!
      cruisespeed += dspeed;//acceleration
    }
    
    // limit speed
    if(cruisespeed<minspeed){cruisespeed = minspeed;}
    else if(cruisespeed>maxspeed){cruisespeed = maxspeed;}

    obsangle = wrap(atan2(Ysensor, Xsensor)+realangle + PI);// + realangle to bring it to absolut coords and pi to +180°
    //calculate objangle
    objangle = atan2(Y2-Y,X2-X);
    
    // point forward steering
    targetangle = wrap(objangle + obsangle); // calculate deviation angle
    alpha = wrap(targetangle - realangle); // calculate error
    rspeed = (int) (cruisespeed*(cos(alpha) + (K*sin(alpha)))); //caclultae wheel speeds
    lspeed = (int) (cruisespeed*(cos(alpha) - (K*sin(alpha)))); //calculate wheel speeds

    // led 
    digitalWrite(LED_BUILTIN, led_status);// proof of normal operation
    led_status = ! led_status;  
    // delay(100);  // this is really optional
    }
  
} 

