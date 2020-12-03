#include <Arduino.h> 
#include "header.h"

void encodeur_callback() 
{
  if(digitalRead(pin_encodeur2)){
    nbTic ++;
  }else
  {
    nbTic --;
  }
    
}

ISR(TIMER1_COMPA_vect){
  //interrupt commands for TIMER 1 here, runs asynchonrously
  cli(); // interdire les interruptions
  tickcopy = nbTic;//copy of tick so that we can turn back on the interrupts
  nbTic = 0;// on remet la valeur nbTic à 0
  sei();//turn th einterrupt sback on

  e = speed - tickcopy; // erreur mesurée  
  de = e - olde; // delta erreur mesuré 
  E += e; // somme des erreurs mesurées 

  if (E>10){E = 512;} //antiwindup
  if (E<-10){E = -512;} //antiwindup

  res_PID = e*kp + de*kd + E*ki; // calcul du PID (de 0 à 255) 
  if (res_PID>255){res_PID = 255;} //saturation
  if (res_PID<0){res_PID = 0;} //saturation

  analogWrite (pin_enable, res_PID); // on écrit sur pin_enable la valeur de res_pid 
  olde = e;  
  Serial.println(tickcopy);// on print la valeur des RPM obtenus 
}

void setup()  
{ 
  // put your setup code here, to run once: 
  pinMode(LED_BUILTIN, OUTPUT);  
  pinMode(pin_enable, OUTPUT);  
  pinMode(pin_dir1, OUTPUT);  
  pinMode(pin_dir2, OUTPUT);  
  pinMode(pin_encodeur, INPUT);  
  
  attachInterrupt (digitalPinToInterrupt(pin_encodeur), encodeur_callback, RISING); //A chaque tic on appelle encodeur_interrupt qui se chargera d'incrémenter de 1 nbTic 
  
  dir = (speed>0);
  digitalWrite(pin_enable, LOW); //coupe l alimentation du moteur avant que le sens soit déclaré
  digitalWrite(pin_dir1, dir);  //on declare le sen de rotation
  digitalWrite(pin_dir2, !dir); //on declare le sen de rotation

  Serial.begin(450000); // on init la comm par serial

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
  sei(); // on rallume les interruptions
} 

void loop()  
{ 
  // put your main code here, to run repeatedly: 
  digitalWrite(LED_BUILTIN, led_status);//on fait clignoter la led intégré de facon bloquant
  led_status = !led_status;  
  delay(1000);//ceci est bloquant
} 