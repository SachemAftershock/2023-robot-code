#include <Arduino.h>
#include <string.h>
#include <state.h>
#include <Led.h>

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
}

void loop()
{
  String command = Serial.readStringUntil('\n');
  cmd = parseCommand(command);
  ledGroup.blinkLeds();
}