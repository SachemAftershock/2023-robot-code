#ifndef LED_H_
#define LED_H_

#include <Arduino.h>

constexpr size_t kLedBlinkDelay = 50;

enum CommandType
{
  NO_TYPE,
  COMMAND_ONE_LED,
  COMMAND_ALL_LEDS,
  MESSAGE_COMMAND,
};

enum LedStatus
{
  LED_OFF,
  LED_ON,
  LED_BLINKING,
};

struct Command
{
  CommandType commandType;
  uint8_t ledId;
  LedStatus status;
  String message;
};

class LED
{
  uint8_t m_Id;
  LedStatus m_DesiredState;
  LedStatus m_CurrentState;
  size_t m_PrevTime;

public:
  LED(uint8_t ledId, LedStatus status = LED_OFF)
  {
    m_Id = ledId;
    m_DesiredState = status;
    m_CurrentState = status;
    pinMode(m_Id, OUTPUT);
  }

  void enable()
  {
    digitalWrite(m_Id, HIGH);
    m_CurrentState = LED_ON;
  }

  void disable()
  {
    digitalWrite(m_Id, LOW);
    m_CurrentState = LED_OFF;
  }

  uint8_t getId()
  {
    return m_Id;
  }

  void setDesiredState(LedStatus status)
  {
    m_DesiredState = status;
  }

  LedStatus getDesiredState()
  {
    return m_DesiredState;
  }

  LedStatus getCurrentState()
  {
    return m_CurrentState;
  }

  void setTime(size_t time)
  {
    m_PrevTime = time;
  }

  size_t getPrevTime()
  {
    return m_PrevTime;
  }
};

class LedGroup
{
  LED *m_Leds;
  size_t m_Count;

public:
  LedGroup(LED *leds, size_t size)
  {
    m_Leds = leds;
    m_Count = size;
  }

  void enableAll()
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      (m_Leds + i)->enable();
      (m_Leds + i)->setDesiredState(LED_ON);
    }
  }

  void disableAll()
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      (m_Leds + i)->disable();
      (m_Leds + i)->setDesiredState(LED_OFF);
    }
  }

  void blinkAll()
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      (m_Leds + i)->setDesiredState(LED_BLINKING);
    }
  }

  void enableLed(uint8_t ledId)
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      if ((m_Leds + i)->getId() == ledId)
      {
        (m_Leds + i)->enable();
        (m_Leds + i)->setDesiredState(LED_ON);
      }
    }
    disableOtherLeds(ledId);
  }

  void disableLed(uint8_t ledId)
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      if ((m_Leds + i)->getId() == ledId)
      {
        (m_Leds + i)->disable();
        (m_Leds + i)->setDesiredState(LED_OFF);
      }
    }
  }

  void blinkLed(uint8_t ledId)
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      if ((m_Leds + i)->getId() == ledId)
      {
        (m_Leds + i)->setDesiredState(LED_BLINKING);
      }
    }
  }

  void blinkLeds()
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      // LED led = *(m_Leds + i);

      if ((m_Leds + i)->getDesiredState() != LED_BLINKING)
        continue;
      if ((millis() - (m_Leds + i)->getPrevTime()) < kLedBlinkDelay)
        continue;

      if ((m_Leds + i)->getCurrentState() == LED_ON)
        (m_Leds + i)->disable();
      else
        (m_Leds + i)->enable();

      (m_Leds + i)->setTime(millis());
    }
  }

  boolean hasLed(uint8_t ledId)
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      if ((m_Leds + i)->getId() == ledId)
      {
        return true;
      }
    }
    return false;
  }

private:
  void disableOtherLeds(uint8_t ledId)
  {
    for (size_t i = 0; i < m_Count; i++)
    {
      if ((m_Leds + i)->getId() != ledId)
      {
        (m_Leds + i)->disable();
      }
    }
  }
};

int8_t stoi(char *string)
{
  char c = *string;
  size_t length = 0;
  while (c != '\0')
  {
    length++;
    c = *(string + length);
  }

  if (length > 2)
    return -1;

  c = *string;
  int8_t sum = 0;

  for (size_t i = 0; i < length; i++)
  {
    c = *(string + i);
    if (c < '0' || c > '9')
    {
      return -1;
    }
    sum += pow(10, length - 1 - i) * (c - '0');
  }

  return sum;
}

/**
 * Parses a command based on format and commands available in readme file
 *
 * @param command the command to process
 *
 * @return the command in a Command struct
 */
Command parseCommand(String command)
{

  char *str = strupr((char *)command.c_str());
  char *split;
  Command cmd;

  split = strtok(str, " ");

  if (split == NULL)
    return cmd;

  if (strcmp(split, "LED") == 0)
  {
    split = strtok(NULL, " ");

    if (split == NULL)
      return cmd;

    if (strcmp(split, "SET") == 0)
    {
      cmd.commandType = COMMAND_ONE_LED;

      split = strtok(NULL, " ");
      if (split == NULL)
      {
        cmd.commandType = NO_TYPE;
        return cmd;
      }

      int8_t id = stoi(split);
      if (id < 0)
      {
        cmd.commandType = NO_TYPE;
        return cmd;
      }

      cmd.ledId = (uint8_t)id;
    }
    else if (strcmp(split, "SETALL") == 0)
    {
      cmd.commandType = COMMAND_ALL_LEDS;
    }
    else
    {
      return cmd;
    }

    split = strtok(NULL, " ");
    if (split == NULL)
    {
      cmd.commandType = NO_TYPE;
      return cmd;
    }

    if (strcmp(split, "ON") == 0)
    {
      cmd.status = LED_ON;
    }
    else if (strcmp(split, "OFF") == 0)
    {
      cmd.status = LED_OFF;
    }
    else if (strcmp(split, "BLINK") == 0)
    {
      cmd.status = LED_BLINKING;
    }
    else
    {
      cmd.commandType = NO_TYPE;
    }
  }
  else if (strcmp(split, "MSG") == 0)
  {
    cmd.commandType = MESSAGE_COMMAND;

    split = strtok(NULL, " ");
    if (split == NULL)
    {
      cmd.commandType = NO_TYPE;
      return cmd;
    }

    cmd.message = split;
  }
  else
  {
    cmd.commandType = NO_TYPE;
    cmd.message = "Invalid command";
  }

  return cmd;
}

#endif /* LED_H_ */