#include <Arduino.h> 

#define kp 0.3 // poids proportionnel 
#define ki 0.3 // poinds intégral 
#define kd 0.0 // poids dérivatif 
#define targerRPM 120 

volatile int e; // erreur proportionnelle 
volatile int de; // delta erreur (erreur dérivative) 
volatile int E = 0; // erreur intégrale 
volatile int nbTic = 0; // nb de tics par cycle 
volatile int olde = 0;  

#define pin_enable 11 // a besoin du PWM 
#define pin_dir1 12 // direction 1 
#define pin_dir2 10 // direction 2 
#define pin_encodeur 2 // a besoin du pin interrupt 
#define N 11.0 // nb de tic par rotation du moteur 
#define tps 10 // délai défini à 10ms 
#define rapportDeReduction 0.05 // rapport de réduction entre la vitesse calculée du moteur et sa vitesse réelle 

const float tpsEnMinute = tps/60000.00;  
float R; // nombre de rotations par minute du moteur 
bool dir = true; //permet de changer la direction 
float V; // résultat de sortie du PID 
bool led_status = true; //permet de controler la led integrer

void encodeur_callback() 
{ 
  nbTic ++;  
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
} 

void loop()  
{ 
  // put your main code here, to run repeatedly: 
  digitalWrite(LED_BUILTIN, led_status); 
  led_status = !led_status;  
  delay(tps); 
  cli(); // interdire les interruptions 
  R = nbTic/N;// rotations 
  R *= rapportDeReduction; // obtenir le réel nombre de rotations 
  R /= tpsEnMinute; // rotations en RPM 
  e = targerRPM - R; // erreur mesurée  
  de = e - olde; // delta erreur mesuré 
  E += e; // somme des erreurs mesurées 
  if (E>512)
  {
    E = 512;
  } 
  if (E<-512)
  { 
    E = -512;  
  } 
  V = e*kp + de*kd + E*ki; // calcul du PID (de 0V à 5V) 
  V *= 51; // PID (de 0 à 255 (valeurs du "monde" du PWM))
  if (V>255) 
  { 
    V = 255;  
  } 
  if (V<0) 
  { 
    V = 0; 
  } 
  analogWrite (pin_enable, (int)V); // on écrit sur pin_enable la valeur de V 
  nbTic = 0; // on remet la valeur nbTic à 0 
  olde = e;  
  Serial.print(R);// on print la valeur des RPM obtenus 
  Serial.println(" : RPM");  
  sei(); // accepter de nouveau les interruptions (du encodeur_callback notamment) 
} 