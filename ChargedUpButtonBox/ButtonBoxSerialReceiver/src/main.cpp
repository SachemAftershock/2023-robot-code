// #include <Arduino.h>
// #include <Wire.h>
// #include "Timer.h"

// Timer t;
// char ch;

// enum systemModeEnum {eReady, eBusy };
// systemModeEnum currentSystemMode = eReady;
// systemModeEnum lastSystemMode = eBusy;
// int commandReceivedLedOnboardArduino = 13;

// enum buttonBoxLedsEnum { 
//   eLedRoboRioBusy = 0,

//   eLed_Slot1_Cone = 1,
//   eLed_Slot2_Cube = 2,
//   eLed_Slot3_Cone = 3,

//   eLed_Slot4_Cone = 4,
//   eLed_Slot5_Cube = 5,
//   eLed_Slot6_Cone = 6,

//   eLed_Slot7_Cone = 7,
//   eLed_Slot8_Cube = 8,
//   eLed_Slot9_Cone = 9,

//   eLed_HumanStationLeft = 10,
//   eLed_HumanStationRight = 11,

//   eLed_ConeMode = 12,
//   eLed_CubeMode = 13,

//   eLed_ElevatorHigh = 14,
//   eLed_ElevatorMedium = 15,
//   eLed_ElevatorLow = 16,
//   eLed_ElevatorFloor = 17,
//   eLed_ElevatorPark = 18,

//   eLed_Ingest = 19,
//   eLed_Eject = 20
// };

// bool blinkOn = false;

// class LED {
//   uint8_t m_Id;

//   public:
//     LED(uint8_t ledId) {
//       m_Id = ledId;
//       pinMode(m_Id, OUTPUT);
//     }

//     void Enable() {
//       digitalWrite(m_Id, HIGH);
//     }

//     void Disable() {
//       digitalWrite(m_Id, LOW);
//     }
// };




// void toggleCommandArrivedLed(int LightDuration = 50)
// {
//   digitalWrite(commandReceivedLedOnboardArduino, HIGH);
//   delay(LightDuration);
//   digitalWrite(commandReceivedLedOnboardArduino, LOW);
//   delay(LightDuration);
// }

// void processCommand(char theCh) {
//   Serial.print("Recieved Command ");
//   Serial.print(theCh);
//   Serial.print("\n"); 
//   switch (theCh)
//   {
//     case 'r':
//       currentSystemMode = eReady;
//       Serial.write("Commanded RoboRIO is ready for next command. \n");
//       break;
   
//     case 'b':
//       currentSystemMode = eBusy;
//       Serial.write("Commanded RoboRIO is busy. \n");
//       break;

//     default:
//       Serial.write("Commanded not recognized. \n");
//       break;
//   }

// }

// void processCurrentMode() {
//   switch (currentSystemMode) {

//     case eReady:
//       if (lastSystemMode != eReady) {
//         digitalWrite(eLedRoboRioBusy,LOW);
//         lastSystemMode = eReady;
//       }
//       break;
      
//     case eBusy:
//       if (lastSystemMode != eBusy) {
//         digitalWrite(eLedRoboRioBusy,HIGH);
//         lastSystemMode = eBusy;
//       }
//       break;
            
//     default:
//       Serial.print("Unrecognized Mode");
//       break;
       
//   }
// }

// void receiveEvent(int howMany)
// {
//   Serial.write("receiveEvent Invoked. \n");
//   while (Wire.available()) //when you recieve a byte (through i2c)
//   {
//     toggleCommandArrivedLed();

//     unsigned char state = Wire.read();
//     Serial.write("Received Char: <");
//     Serial.write(state);
//     Serial.write("> \n");
//     //not sure if it should be a normal or unsigned char

//     processCommand(state);
//   }
// }

// void setup() {
//   Wire.begin(1); //communicate on this address
//   Wire.onReceive(receiveEvent);//when communication is successful,
//   Serial.begin(9600);          //go to the function
//   t.every(200, processCurrentMode);
//   pinMode(commandReceivedLedOnboardArduino, OUTPUT);
//   toggleCommandArrivedLed();
//   pinMode(eLedRoboRioBusy, OUTPUT);
//   Serial.write("Arduino-Slave-I2C-ButtonBoxSerialReceiver Initialization Complete. \n");
// }

// void loop() {
//    t.update();
//   if (Serial.available()) {
//     ch = Serial.read();
//     processCommand(ch);
//   }

// }
