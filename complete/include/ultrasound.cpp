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