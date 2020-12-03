#define kp 3 // poids proportionnel 
#define ki 3 // poinds intégral 
#define kd 0 // poids dérivatif 


int slowdowndist = 400; //mm
int turndist = 300;//

float cruisespeed = 0.0; // speed of center of momentum
const int maxspeed = 5;
const int minspeed = 5;
const float dspeed = 0.3; // variation of speed
const float dangle = 0.05; // variationof angle
volatile int rspeed = 4; //! important to determinate max speed
volatile int lspeed = 4; //! important to determinate max speed
float targetangle = 0; // tempangle used for math
float objangle = 0; // the desired angle
float realangle = 0; // the real angle of the bot
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
#define pin_lenable 11 // a besoin du PWM 
#define pin_ldir1 12 // direction 1 
#define pin_ldir2 10 // direction 2 
#define pin_lencodeur 2 // a besoin du pin interrupt 
#define pin_lencodeur2 4//sens de rotation du moteur

///          RIGHT
bool rdir = true; //permet de changer la direction 
int rres_PID; // résultat de sortie du PID 

///          LEFT
bool ldir = true; //permet de changer la direction 
int lres_PID; // résultat de sortie du PID 

int startdelai = 1000;
bool led_status = true; //permet de controler la led integrer



///               ULTRASOUND

#define pin_trigger 4
#define pin_echo 5

const float SOUND_SPEED = (340.0 / 1000.0)/2.0;
const unsigned long MEASURE_TIMEOUT = 2.0/SOUND_SPEED; // T = D/V

long measure;//distance mesuré par ultrasound
float distance_mm;//la dist ne mm

//              POINTFORWARD STEERING
float wrap(float inangle);//declare the function for easy find
float alpha;//angle error
float entraxe = 250.0;//mm
float l =entraxe;//mm //! TURNING GAIN 
float K = entraxe/(2.0*l); //! K app [1/2:1]