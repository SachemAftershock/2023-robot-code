#include <Arduino.h>
#include <Wire.h>
#include "Timer.h"

Timer t;
char ch;

enum systemModeEnum {Off, ProcessTheMessages };
systemModeEnum currentSystemMode = ProcessTheMessages;
systemModeEnum lastSystemMode = Off;
int commandReceivedLedOnboardArduino = 13;

bool blinkOn = false;

void toggleCommandArrivedLed(int LightDuration = 50)
{
  digitalWrite(commandReceivedLedOnboardArduino, HIGH);
  delay(LightDuration);
  digitalWrite(commandReceivedLedOnboardArduino, LOW);
  delay(LightDuration);
}

void processCommand(char theCh) {
  Serial.print("Recieved Command ");
  Serial.print(theCh);
  Serial.print("\n"); 
  switch (theCh)
  {
    case 'o':
      currentSystemMode = Off;
      Serial.write("Commanded OFF. \n");
      break;
   
    case 'p':
      currentSystemMode = ProcessTheMessages;
      Serial.write("Commanded PROCESS. \n");
      break;

    default:
      Serial.write("Commanded not recognized. \n");
      break;
  }

}

void processCurrentMode() {
  switch (currentSystemMode) {

    case Off:
      if (lastSystemMode != Off) {

        lastSystemMode = Off;
      }
      break;
      
    case ProcessTheMessages:
      if (lastSystemMode != ProcessTheMessages) {

        lastSystemMode = ProcessTheMessages;
      }
      break;
            
    default:
      Serial.print("Unrecognized Mode");
      break;
       
  }
}

void receiveEvent(int howMany)
{
  Serial.write("receiveEvent Invoked. \n");
  while (Wire.available()) //when you recieve a byte (through i2c)
  {
    toggleCommandArrivedLed();

    unsigned char state = Wire.read();
    Serial.write("Received Char: <");
    Serial.write(state);
    Serial.write("> \n");
    //not sure if it should be a normal or unsigned char

    processCommand(state);
  }
}

void setup() {
  Wire.begin(1); //communicate on this address
  Wire.onReceive(receiveEvent);//when communication is successful,
  Serial.begin(9600);          //go to the function
  t.every(200, processCurrentMode);
  pinMode(commandReceivedLedOnboardArduino, OUTPUT);
  toggleCommandArrivedLed();
  Serial.write("Arduino-Slave-I2C-ButtonBoxSerialReceiver Initialization Complete. \n");
}

void loop() {
   t.update();
  if (Serial.available()) {
    ch = Serial.read();
    processCommand(ch);
  }

}
