#include <Arduino.h>

#define pin_trigger 4
#define pin_echo 5


const float distance_max = 1100.00;
const float SOUND_SPEED = (0.343)/2.0;
const long  MEASURE_TIMEOUT = 2*distance_max/0.343;
bool ledstatus = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin_trigger, OUTPUT);
  digitalWrite(pin_trigger, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(pin_echo, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigger, LOW);
  long measure = pulseIn(pin_echo, HIGH,MEASURE_TIMEOUT);
  float distance_mm = measure * SOUND_SPEED;
  Serial.println(distance_mm);
  digitalWrite(LED_BUILTIN, ledstatus);
  ledstatus =  ! ledstatus;
  delay(200);
}