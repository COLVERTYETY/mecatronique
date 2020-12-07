#define kp 30 // poids proportionnel 
#define ki 30 // poinds intégral 
#define kd 0 // poids dérivatif 

volatile int speed = 5; //! important to determinate max speed

int e; // erreur proportionnelle 
int de; // delta erreur (erreur dérivative) 
int E = 0; // erreur intégrale 
int nbTic = 0; // nb de tics par cycle 
int olde = 0;  
volatile int tickcopy;//copy of tick

#define pin_enable 6 // a besoin du PWM 
#define pin_dir1 8 // direction 1 
#define pin_dir2 7 // direction 2 
#define pin_encodeur 3 // a besoin du pin interrupt 
#define pin_encodeur2 5//sens de rotation du moteur

bool dir = true; //permet de changer la direction 
int res_PID; // résultat de sortie du PID 
bool led_status = true; //permet de controler la led integrer

