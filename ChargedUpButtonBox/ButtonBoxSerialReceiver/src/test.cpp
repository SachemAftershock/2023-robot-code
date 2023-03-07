#include <Arduino.h>
#include <string.h>
#include <state.h>
#include <Led.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

LED DriveToCone1 = LED(5);
LED DriveToCube2 = LED(21);
LED DriveToCone3 = LED(7);
LED DriveToCone4 = LED(4);
LED DriveToCube5 = LED(17);
LED DriveToCone6 = LED(15);
LED DriveToCone7 = LED(6);
LED DriveToCube8 = LED(18);
LED DriveToCone9 = LED(3);
LED HumanLeft = LED(14);
LED HumanRight = LED(23);
LED Cancel = LED(16);

LED DriveToLeds[] = {
    DriveToCone1,
    DriveToCube2,
    DriveToCone3,
    DriveToCone4,
    DriveToCube5,
    DriveToCone7,
    DriveToCube8,
    DriveToCone9,
    HumanLeft,
    HumanRight,
    Cancel,
};

LedGroup driveToLedGroup(DriveToLeds, sizeof(DriveToLeds) / sizeof(DriveToLeds[0]));

LedGroup ledGroups[] = {
    driveToLedGroup,
};
size_t ledGroupsSize = sizeof(ledGroups) / sizeof(ledGroups[0]);

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

void enableLed(uint8_t ledId)
{
  for (size_t i = 0; i < ledGroupsSize; i++)
  {
    if (ledGroups[i].hasLed(ledId))
    {
      ledGroups[i].enableLed(ledId);
      return;
    }
  }
}

void disableLed(uint8_t ledId)
{
  for (size_t i = 0; i < ledGroupsSize; i++)
  {
    if (ledGroups[i].hasLed(ledId))
    {
      ledGroups[i].disableLed(ledId);
      return;
    }
  }
}

void blinkLed(uint8_t ledId)
{
  for (size_t i = 0; i < ledGroupsSize; i++)
  {
    if (ledGroups[i].hasLed(ledId))
    {
      ledGroups[i].setLedStatus(ledId, LED_BLINKING);
      return;
    }
  }
}

void enableAllLeds()
{
  for (size_t i = 0; i < ledGroupsSize; i++)
  {
    ledGroups[i].enableAll();
  }
}

void disableAllLeds()
{
  for (size_t i = 0; i < ledGroupsSize; i++)
  {
    ledGroups[i].disableAll();
  }
}

void blinkAllLeds()
{
  for (size_t i = 0; i < ledGroupsSize; i++)
  {
    ledGroups[i].blinkAll();
  }
}

void processCommand(Command command)
{

  setDisplay(String(command.commandType));

  if (command.commandType == NO_TYPE)
    return;

  setDisplay("Switching");

  switch (command.commandType)
  {
  case COMMAND_ONE_LED:
    setDisplay("Commanding LED");
    if (command.status == LED_ON)
      enableLed(command.ledId);
    else if (command.status == LED_OFF)
      disableLed(command.ledId);
    else if (command.status == LED_BLINKING)
      blinkLed(command.ledId);
    break;
  case COMMAND_ALL_LEDS:
    if (command.status == LED_ON)
      enableAllLeds();
    else if (command.status == LED_OFF)
      disableAllLeds();
    else if (command.status == LED_BLINKING)
      blinkAllLeds();
    break;
  case MESSAGE_COMMAND:
    setDisplay(command.message);
    break;
  default:
    break;
  }
}

Command cmd;

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

  if (!command.equals(""))
  {
    cmd = parseCommand(command);
    // setDisplay(String(cmd.status));
    processCommand(cmd);
  }

  for (size_t i = 0; i < ledGroupsSize; i++)
  {
    ledGroups[i].blinkLeds();
  }
}