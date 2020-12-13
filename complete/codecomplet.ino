#include <Arduino.h> 

class ultrasond{
  public:
    const float distance_max = 1100.00;
    const float SOUND_SPEED = (0.343)/2.0;
    const long  MEASURE_TIMEOUT = 2*distance_max/0.343;
    const int offset = 0;//un déclage pour prendre en compte la distance du capteur par raport au centre du robot
    int echo_pin; //pin echo
    int trigger_pin;// pin trigger
    float angle;
    ultrasond(int echo, int triggger, float aangle){
    echo_pin = echo;
    trigger_pin = triggger;
    angle = aangle;
  }
  public: 
    void init(){//initialisations des pins 
      pinMode(echo_pin,INPUT);
      pinMode(trigger_pin,OUTPUT);
      digitalWrite(trigger_pin, LOW);
    }
    float measure(){//leacture de la distance
      digitalWrite(trigger_pin,HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger_pin,LOW);
      long measure = pulseIn(echo_pin, HIGH,MEASURE_TIMEOUT);//casse par les timers interrupts
      return (measure * SOUND_SPEED)+offset;
    }
};

#define kp 30 // poids proportionnel 
#define ki 30 // poinds intégral 
#define kd 0 // poids dérivatif 


#define epsilon 1000 // min distance to goal before stop //! is squared for no sqrt
long distsquared = 999.00;

int slowdowndist = 35000; //mm //! should be square of actual value for computational reasons
int turndist =     30000;//? to remove ?

volatile long X = 0.00;//coords of robots
volatile long Y = 0.00;// init at 0,0 

long X2 = 2000.00;//coords of objectif
long Y2 = 2000.00;//should be a straight line

float ratioencoder = 220.00;//number of motor tick for 1 wheel turn
float rd = 0.0;//actual distance in mm 
float ld = 0.0;//actuall distance in mm
float vcenter = 0.00;

float cruisespeed = 0.0; // speed of center of momentum
const int maxspeed = 9;
const int minspeed = -9;
const float dspeed = 0.05; // variation of speed
const float dangle = PI/16.0; // variationof angle
volatile int rspeed = 4; //! important to determinate max speed
volatile int lspeed = 4; //! important to determinate max speed
float targetangle = 0; // tempangle used for math
float objangle = 0; // the desired angle
volatile float realangle = 0; // the real angle of the bot
float obsangle = 0; // angle buffer for avoidance


///         RIGHT
int re; // erreur proportionnelle 
int rde; // delta erreur (erreur dérivative) 
int rE = 0; // erreur intégrale 
int rnbTic = 0; // nb de tics par cycle 
int rolde = 0;  
volatile int rtickcopy;//copy of tick

///         LEFT
int le; // erreur proportionnelle 
int lde; // delta erreur (erreur dérivative) 
int lE = 0; // erreur intégrale 
int lnbTic = 0; // nb de tics par cycle 
int lolde = 0;  
volatile int ltickcopy;//copy of tick

///         RIGHT
#define pin_renable 11 // a besoin du PWM 
#define pin_rdir1 12 // direction 1 
#define pin_rdir2 10 // direction 2 
#define pin_rencodeur 2 // a besoin du pin interrupt 
#define pin_rencodeur2 4//sens de rotation du moteur


///         LEFT
#define pin_lenable 6 // a besoin du PWM 
#define pin_ldir1 8 // direction 1 
#define pin_ldir2 7 // direction 2 
#define pin_lencodeur 3 // a besoin du pin interrupt 
#define pin_lencodeur2 5//sens de rotation du moteur

///          RIGHT
volatile bool rdir = true; //permet de changer la direction 
int rres_PID; // résultat de sortie du PID 

///          LEFT
volatile bool ldir = true; //permet de changer la direction 
int lres_PID; // résultat de sortie du PID 

float startdelai = 1000.0;
bool led_status = true; //permet de controler la led integrer

///                 LEDs pour visualisation de l activité du robot
#define led_R A1
#define led_J A2
#define led_V A3
bool rstatus = true; // active low
bool jstatus = true; // active low
bool vstatus = true; // active low

///               ULTRASOUND

ultrasond left(13,A5,0.26);//! pins must be figured out
ultrasond right(13,A4,-0.26);//? can use the same echo ?
const int sensorarraylength=2;
ultrasond sensorarray[sensorarraylength]{left, right};// tableau de capteurs
float distance_mm;
float Xsensor;//vecteur obstacle
float Ysensor;
const float kw = 10.0;//poids de réduction de la force de repulsion

//              POINTFORWARD STEERING
float wrap(float inangle);//declare the function for easy find
float alpha;//angle error
const float entraxe = 219.0;//mm
const float wheelperimeter = 75.0*PI;//mm PI*diameter
const float l =entraxe;//mm //! TURNING GAIN 
const float K = entraxe/(2.0*l); //! K app [1/2:1]
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

