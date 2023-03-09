#include <Arduino.h>
#include <string.h>
#include <state.h>
#include <Led.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

LED DriveToCone1 = LED(5);
LED DriveToCube2 = LED(23);
LED DriveToCone3 = LED(7);
LED DriveToCone4 = LED(4);
LED DriveToCube5 = LED(17);
LED DriveToCone6 = LED(15);
LED DriveToCone7 = LED(6);
LED DriveToCube8 = LED(18);
LED DriveToCone9 = LED(3);
LED HumanLeft = LED(14);
LED HumanRight = LED(25);
LED Cancel = LED(16);

LED DriveToLeds[] = {
    DriveToCone1,
    DriveToCube2,
    DriveToCone3,
    DriveToCone4,
    DriveToCube5,
    DriveToCone6,
    DriveToCone7,
    DriveToCube8,
    DriveToCone9,
    HumanLeft,
    HumanRight,
    Cancel,
};
LedGroup driveToLedGroup(DriveToLeds, sizeof(DriveToLeds) / sizeof(DriveToLeds[0]));

LED elevatorStow(11);
LED elevatorHuman(10);
LED elevatorLow(22);
LED elevatorMid(24);
LED elevatorHigh(12);

LED elevatorLeds[] = {
    elevatorStow,
    elevatorHuman,
    elevatorLow,
    elevatorMid,
    elevatorHigh,
};
LedGroup elevatorLedGroup(elevatorLeds, sizeof(elevatorLeds) / sizeof(elevatorLeds[0]));

LED injest(19);
LED eject(13);

LED intakeLeds[] = {
    injest,
    eject,
};
LedGroup intakeLedGroup(intakeLeds, sizeof(intakeLeds) / sizeof(intakeLeds[0]));

LED cubeToggle(2);
LED coneToggle(9);

LED toggleLeds[] = {
    cubeToggle,
    coneToggle,
};
LedGroup toggleLedGroup(toggleLeds, sizeof(toggleLeds) / sizeof(toggleLeds[0]));

LED joystickEnabledLed(8);

LED joystickLeds[] = {
    joystickEnabledLed,
};
LedGroup joystickLedGroup(joystickLeds, sizeof(joystickLeds) / sizeof(joystickLeds[0]));

LedGroup ledGroups[] = {
    driveToLedGroup,
    elevatorLedGroup,
    intakeLedGroup,
    toggleLedGroup,
    joystickLedGroup,
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
    message.replace("_", " ");

    lcd.setCursor(0, 0);
    lcd.print(message.substring(0, 16));
    lcd.setCursor(0, 1);
    lcd.print(message.substring(16));
  }
  else
  {
    message.replace("_", " ");

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
      ledGroups[i].blinkLed(ledId);
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

  if (command.commandType == NO_TYPE)
    return;

  switch (command.commandType)
  {
  case COMMAND_ONE_LED:
    if (command.status == LED_ON)
    {
      enableLed(command.ledId);
    }
    else if (command.status == LED_OFF)
      disableLed(command.ledId);
    else if (command.status == LED_BLINKING)
    {
      blinkLed(command.ledId);
    }
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
  Serial.setTimeout(50);

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

  static char buffer[80];
  static size_t currentIndex = 0;

  if (Serial.available())
  {
    char input = Serial.read();
    buffer[currentIndex++] = input;
  }

  if (currentIndex > 0)
  {
    if (buffer[currentIndex - 1] == '\0' || buffer[currentIndex - 1] == '\n')
    {
      if (buffer[currentIndex - 1] == '\n')
      {
        buffer[currentIndex - 2] = '\0';
      }
      String command = buffer;
      cmd = parseCommand(command);
      lcd.clear();
      lcd.print(cmd.commandType);
      lcd.setCursor(0, 1);
      lcd.print(cmd.message);
      processCommand(cmd);
      memset(buffer, 0, sizeof(buffer));
      currentIndex = 0;
    }
  }

  driveToLedGroup.blinkLeds();
}