#include <Arduino.h>

class LED
{
  uint8_t m_Id;

public:
  LED(uint8_t ledId)
  {
    m_Id = ledId;
    pinMode(m_Id, OUTPUT);
  }

  void enable()
  {
    digitalWrite(m_Id, HIGH);
  }

  void disable()
  {
    digitalWrite(m_Id, LOW);
  }
};

/**
 * Splits a string into substrings using a given separator
 *
 * @param string the string to split
 * @param separator the character to split the string at
 *
 * @return an array of substrings
 *
 */
String *split(String string, char separator)
{
  if (separator == '\0')
  {
    String result[1] = {string};
    return result;
  }

  size_t strLength = string.length();

  if (string.length() == 0)
  {
    String result[0];
    return result;
  }

  size_t arrLength = 0;

  for (size_t i = 1; i < strLength - 1; i++)
  {
    if (string[i] != separator)
      continue;

    char prev = string[i - 1];
    char next = string[i + 1];

    if (prev != separator && next != separator)
      arrLength++;
  }

  String result[arrLength];

  size_t lower = 0;
  size_t pos = 0;
  for (size_t i = 1; i < strLength; i++)
  {

    if (string[i - 1] == separator)
    {
      lower++;
      continue;
    }

    if (string[i] != separator)
      continue;

    result[pos++] = string.substring(lower, i - lower);
  }

  return result;
}

/**
 * Process a command based on format and commands available in readme file
 *
 * @param command the command to process
 */
void processCommand(String command)
{

  // first we need to split the command up into its part
  // since it will be separated by spaces, we need to split it by spaces
}

void setup()
{
}

void loop()
{
}