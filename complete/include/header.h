#include "ultrasound.cpp"
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