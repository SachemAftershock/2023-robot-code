#include <Arduino.h>
#include <string.h>
#include <state.h>
#include <Led.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

void setDisplay(String message)
{

  lcd.clear();

  if (message.length() > 32)
  {
    lcd.setCursor(0, 0);
    lcd.print("Message too long");
  }
  else if (message.length() > 16)
  {
    lcd.setCursor(0, 0);
    lcd.print(message.substring(0, 16));
    lcd.setCursor(0, 1);
    lcd.print(message.substring(16));
  }
  else
  {
    lcd.setCursor(0, 0);
    lcd.print(message);
  }
}

void processCommand(Command command)
{
  if (command.commandType == NO_TYPE)
    return;

  switch (command.commandType)
  {
  case COMMAND_ONE_LED:
    uint8_t ledId = command.ledId;

    if (command.status == LED_ON)
      ledGroup.enableLed(ledId);
    else if (command.status == LED_OFF)
      ledGroup.disableLed(ledId);
    else if (command.status == LED_BLINKING)
      ledGroup.setLedStatus(ledId, LED_BLINKING);
    break;
  case COMMAND_ALL_LEDS:
    if (command.status == LED_ON)
      ledGroup.enableAll();
    else if (command.status == LED_OFF)
      ledGroup.disableAll();
    else if (command.status == LED_BLINKING)
      ledGroup.blinkAll();
    break;
  case MESSAGE_COMMAND:
    setDisplay(command.message);
    break;
  default:
    break;
  }
}

Command cmd;

LED leds[] = {
    LED(0),
    LED(1),
};

LedGroup ledGroup(leds, sizeof(leds) / sizeof(leds[0]));

void setup()
{
  Serial.begin(9600);

  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.setCursor(1, 0);         // set the cursor to column 1, line 0
  lcd.print("AFTERSHOCK 263"); // Print a message to the LCD
  lcd.setCursor(2, 1);         // set the cursor to column 2, line 1
  lcd.print("Charged Up !");   // Print a message to the LCD.
}

void loop()
{
  String command = Serial.readStringUntil('\n');
  cmd = parseCommand(command);
  ledGroup.blinkLeds();
}