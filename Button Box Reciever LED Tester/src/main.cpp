#include <Arduino.h>

uint8_t LED_Start = 3;
uint8_t LED_End = 24;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (size_t i = LED_Start; i <= LED_End; i++)
  {
    pinMode(i, OUTPUT);
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  for (size_t i = LED_Start; i <= LED_End; i++)
  {
    digitalWrite(i, HIGH);
  }
  delay(400);
  for (size_t i = LED_Start; i <= LED_End; i++)
  {
    digitalWrite(i, LOW);
  }
  delay(400);
}