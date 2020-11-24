#include <Arduino.h> 

#define kp 0.3 // poids proportionnel 
#define ki 0.3 // poinds intégral 
#define kd 0.0 // poids dérivatif 
#define targerRPM 80

volatile int e; // erreur proportionnelle 
volatile int de; // delta erreur (erreur dérivative) 
volatile int E = 0; // erreur intégrale 
volatile int nbTic = 0; // nb de tics par cycle 
volatile int olde = 0;  

#define pin_enable 11 // a besoin du PWM 
#define pin_dir1 12 // direction 1 
#define pin_dir2 10 // direction 2 
#define pin_encodeur 2 // a besoin du pin interrupt 
#define N 224.4 // nb de tic par rotation du moteur avec reducteur
#define tps 10 // délai défini à 10ms 

const float tpsEnMinute = 60000.00/tps;  
const float ratio = tpsEnMinute/N;
float vitesse_rpm; // nombre de rotations par minute du moteur 
bool dir = true; //permet de changer la direction 
float res_PID; // résultat de sortie du PID 
bool led_status = true; //permet de controler la led integrer
int tickcopy;//copy of tick

void encodeur_callback() 
{ 
  nbTic ++;  
}
ISR(TIMER1_COMPA_vect){
  //interrupt commands for TIMER 1 here, runs asynchonrously
  cli(); // interdire les interruptions
  tickcopy = nbTic;//copy of tick so that we can turn back on the interrupts
  nbTic = 0;// on remet la valeur nbTic à 0
  sei();//turn th einterrupt sback on

  vitesse_rpm = tickcopy*ratio; // rotations en RPM 
  e = targerRPM - vitesse_rpm; // erreur mesurée  
  de = e - olde; // delta erreur mesuré 
  E += e; // somme des erreurs mesurées 

  if (E>10){E = 10;} //antiwindup
  if (E<-10){E = -10;} //antiwindup

  res_PID = e*kp + de*kd + E*ki; // calcul du PID (de 0 à 5) 
  res_PID *= 51; // PID (de 0 à 255 (valeurs du "monde" du PWM))

  if (res_PID>255){res_PID = 255;} //saturation
  if (res_PID<0){res_PID = 0;} //saturation

  analogWrite (pin_enable, (int)res_PID); // on écrit sur pin_enable la valeur de res_pid 
  olde = e;  
  Serial.println(vitesse_rpm);// on print la valeur des RPM obtenus 
} 
void setup()  
{ 
  // put your setup code here, to run once: 
  pinMode(LED_BUILTIN, OUTPUT);  
  pinMode (pin_enable, OUTPUT);  
  pinMode (pin_dir1, OUTPUT);  
  pinMode (pin_enable, OUTPUT);  
  pinMode (pin_encodeur, INPUT);  
  attachInterrupt (digitalPinToInterrupt(pin_encodeur), encodeur_callback, RISING); //A chaque tic on appelle encodeur_interrupt qui se chargera d'incrémenter de 1 nbTic 
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