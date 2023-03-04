#include <Arduino.h>
int LED_Start = 2;
int LED_End = 7;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for(int i = LED_Start; i <= LED_End; ++i){
    pinMode(i,OUTPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = LED_Start; i <= LED_End; ++i){
    digitalWrite(i, HIGH);
  }
  delay(400);
  for(int i = LED_Start; i <= LED_End; ++i){
    digitalWrite(i,LOW);
  }
  delay(400);
}