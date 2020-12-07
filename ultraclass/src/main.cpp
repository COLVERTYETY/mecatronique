
#include <Arduino.h>

#define pin_trigger 4
#define pin_echo 5


const float distance_max = 1100.00;
const float SOUND_SPEED = (0.343)/2.0;
const long  MEASURE_TIMEOUT = 2*distance_max/0.343;
bool ledstatus = false;

class ultrasond{
  public:
    const float distance_max = 1100.00;
    const float SOUND_SPEED = (0.343)/2.0;
    const long  MEASURE_TIMEOUT = 2*distance_max/0.343;
    int echo_pin;
    int trigger_pin;
    int angle;
    ultrasond(int echo, int triggger){
    echo_pin = echo;
    trigger_pin = triggger;
  }
  public: 
    void init(){
      pinMode(echo_pin,INPUT);
      pinMode(trigger_pin,OUTPUT);
      digitalWrite(trigger_pin, LOW);
    }
    float measure(){
      digitalWrite(trigger_pin,HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger_pin,LOW);
      long measure = pulseIn(pin_echo, HIGH,MEASURE_TIMEOUT);
      return measure * SOUND_SPEED;
    }
};

ultrasond k(5, 4);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  k.init();
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println(k.measure());
  digitalWrite(LED_BUILTIN, ledstatus);
  ledstatus =  ! ledstatus;
  delay(200);
}

