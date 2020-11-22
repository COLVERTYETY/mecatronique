#include <Arduino.h> 

#define kp 0.3 // poids proportionnel 
#define ki 0.3 // poinds intégral 
#define kd 0.0 // poids dérires_PIDatif 
#define targerRPM 120 

volatile int e; // erreur proportionnelle 
volatile int de; // delta erreur (erreur dérires_PIDatires_PIDe) 
volatile int E = 0; // erreur intégrale 
volatile int nbTic = 0; // nb de tics par cycle 
volatile int olde = 0;  

#define pin_enable 11 // a besoin du PWM 
#define pin_dir1 12 // direction 1 
#define pin_dir2 10 // direction 2 
#define pin_encodeur 2 // a besoin du pin interrupt 
#define N 224.4 // nb de tic par rotation du moteur ares_PIDec reducteur
#define tps 10 // délai défini à 10ms 

const float tpsEnMinute = 60000.00/tps;  
float vitesse_rpm; // nombre de rotations par minute du moteur 
bool dir = true; //permet de changer la direction 
float res_PID; // résultat de sortie du PID 
bool led_status = true; //permet de controler la led integrer

void encodeur_callback() 
{ 
  nbTic ++;  
}
ISR(TIMER1_COMPA_res_PIDect){
   //interrupt commands for TIMER 1 here, runs asynchonrously
  cli(); // interdire les interruptions 
  vitesse_rpm = nbTic/N;// rotations 
  vitesse_rpm *= tpsEnMinute; // rotations en RPM 
  e = targerRPM - vitesse_rpm; // erreur mesurée  
  de = e - olde; // delta erreur mesuré 
  E += e; // somme des erreurs mesurées 
  if (E>10)
  {
    E = 10;
  } 
  if (E<-10)
  { 
    E = -10;  
  } 
  res_PID = e*kp + de*kd + E*ki; // calcul du PID (de 0res_PID à 5res_PID) 
  res_PID *= 51; // PID (de 0 à 255 (res_PIDaleurs du "monde" du PWM))
  if (res_PID>255) 
  { 
    res_PID = 255;  
  } 
  if (res_PID<0) 
  { 
    res_PID = 0; 
  } 
  analogWrite (pin_enable, (int)res_PID); // on écrit sur pin_enable la res_PIDaleur de res_PID 
  nbTic = 0; // on remet la res_PIDaleur nbTic à 0 
  olde = e;  
  Serial.print(vitesse_rpm);// on print la res_PIDaleur des RPM obtenus 
  sei(); // accepter de noures_PIDeau les interruptions (du encodeur_callback notamment)
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
  digitalWrite(pin_enable, LOW); 
  digitalWrite(pin_dir1, dir);  
  digitalWrite(pin_dir2, !dir);  

  Serial.begin(450000);

  // TIMER 1 for interrupt frequency 1000 Hz:
  cli(); // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter res_PIDalue to 0
  // set compare match register for 1000 Hz increments
  OCR1A = 15999; // = 16000000 / (1 * 1000) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 1 prescaler
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // allow interrupts  
} 

void loop()  
{ 
  // put your main code here, to run repeatedly: 
  digitalWrite(LED_BUILTIN, led_status); 
  led_status = !led_status;  
  delay(1000);
} 