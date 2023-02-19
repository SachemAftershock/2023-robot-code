#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

String errorMessage= "No Error";

enum ledEnum { eIngest, eEject};

struct TokenBlock {
  String Token1;
  String Token2;
  String Token3;
  String Token4;
};

// quick and dirty tokenizer
TokenBlock Tokenize(String inputString) {
   TokenBlock Tokens;
   inputString.trim();

  int nextSpace = inputString.indexOf(" ");
  Tokens.Token1 = inputString.substring(0,nextSpace);

  inputString = inputString.substring(nextSpace);
  inputString.trim();
  nextSpace = inputString.indexOf(" ");
  Tokens.Token2 = inputString.substring(0,nextSpace);

  inputString = inputString.substring(nextSpace);
  inputString.trim();
  nextSpace = inputString.indexOf(" ");
  Tokens.Token3 = inputString.substring(0,nextSpace);

  inputString = inputString.substring(nextSpace);
  inputString.trim();
  nextSpace = inputString.indexOf(" ");
  Tokens.Token4 = inputString.substring(0,nextSpace);  
  
  return Tokens;
}

void processInputLine(String theInput) {
  String kCommandLED = "LED";
  String kCommandMSG = "MSG";

  Serial.println(theInput);
  TokenBlock tokens = Tokenize (theInput);
  Serial.println(":" + tokens.Token1 + ":" + tokens.Token2 + ":" + tokens.Token3 + ":" + tokens.Token4 + ":");
  if (tokens.Token1.equalsIgnoreCase(kCommandLED)){ 
      Serial.println("Detected LED command.");
      if (tokens.Token2.equalsIgnoreCase("ALL")){
        //TODO make all the LEDs go to setting in Token3.
      } else {
        //TODO process for only a single LED specified in Token3
        int ledAddr = tokens.Token3.toInt();
      }
  } else if (tokens.Token1.equalsIgnoreCase(kCommandMSG)){
      Serial.println("Detected MSG command.");
      //TODO do work here using the tokens 2 on.
  } else {
      Serial.println("Ignoring unrecognized command.");
  }
}

void setup()
{
  Serial.begin(9600);
  //Serial.println(Serial.getTimeout());   // Default is 1 second for readStringUntil.

  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.setCursor(1, 0); // set the cursor to column 1, line 0
  lcd.print("AFTERSHOCK 263");  // Print a message to the LCD
  lcd.setCursor(2, 1); // set the cursor to column 2, line 1
  lcd.print("Charged Up !");  // Print a message to the LCD.
  delay(3000);
  lcd.clear();
}

void loop()
{
  String command = Serial.readStringUntil('\n');
  if (command.length() > 0) {
    processInputLine(command);
  }

  lcd.setCursor(0, 0); // set the cursor to column 0jjhkjh, line 0
  lcd.print("Status Busy/Idle");  // Print a message to the LCD  //TODO
  lcd.setCursor(0, 1); // set the cursor to column 0, line 1
  lcd.print("ER: ");
  lcd.println(errorMessage.substring(0,12));
}