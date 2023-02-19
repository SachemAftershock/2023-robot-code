// #include <Arduino.h>
// #include <string.h>
// #include <state.h>

// enum CommandType
// {
//   NO_TYPE,
//   COMMAND_ONE_LED,
//   COMMAND_ALL_LEDS,
//   MESSAGE_COMMAND,
// };

// enum LedStatus
// {
//   LED_OFF,
//   LED_ON,
//   LED_BLINKING,
// };

// struct Command
// {
//   CommandType commandType;
//   uint8_t ledId;
//   LedStatus status;
//   String message;
// };

// class LED
// {
//   uint8_t m_Id;

// public:
//   LED(uint8_t ledId)
//   {
//     m_Id = ledId;
//     pinMode(m_Id, OUTPUT);
//   }

//   void enable()
//   {
//     digitalWrite(m_Id, HIGH);
//   }

//   void disable()
//   {
//     digitalWrite(m_Id, LOW);
//   }

//   uint8_t getId()
//   {
//     return m_Id;
//   }
// };

// class LedGroup
// {
//   LED *m_Leds;
//   size_t m_Count;

// public:
//   LedGroup(LED *leds, size_t size)
//   {
//     m_Leds = leds;
//     m_Count = size;
//   }

//   void enableAll()
//   {
//     for (size_t i = 0; i < m_Count; i++)
//     {
//       (m_Leds + i)->enable();
//     }
//   }

//   void disableAll()
//   {
//     for (size_t i = 0; i < m_Count; i++)
//     {
//       (m_Leds + i)->disable();
//     }
//   }

//   void enableLed(uint8_t ledId)
//   {
//     for (size_t i = 0; i < m_Count; i++)
//     {
//       if ((m_Leds + i)->getId() == ledId)
//       {
//         (m_Leds + i)->enable();
//       }
//     }
//   }

//   void disableLed(uint8_t ledId)
//   {
//     for (size_t i = 0; i < m_Count; i++)
//     {
//       if ((m_Leds + i)->getId() == ledId)
//       {
//         (m_Leds + i)->disable();
//       }
//     }
//   }

//   LED *
//   findLed(uint8_t ledId)
//   {
//     for (size_t i = 0; i < m_Count; i++)
//     {
//       if ((m_Leds + i)->getId() == ledId)
//         return (m_Leds + i);
//     }
//     return NULL;
//   }
// };

// int8_t stoi(char *string)
// {
//   char c = *string;
//   size_t length = 0;
//   while (c != '\0')
//   {
//     length++;
//     c = *(string + length);
//   }

//   if (length > 2)
//     return -1;

//   c = *string;
//   int8_t sum = 0;

//   for (size_t i = 0; i < length; i++)
//   {
//     if (c < '0' || c > '9')
//     {
//       return -1;
//     }
//     sum += pow(10, length - 1) * (c - '0');
//   }

//   return sum;
// }

// void processCommand(Command command)
// {
// }

// /**
//  * Parses a command based on format and commands available in readme file
//  *
//  * @param command the command to process
//  *
//  * @return the command in a Command struct
//  */
// Command parseCommand(String command)
// {
//   char *str = strupr((char *)command.c_str());
//   char *split;
//   Command cmd;

//   split = strtok(str, " ");

//   if (split == NULL)
//     return cmd;

//   if (strcmp(split, "LED") == 0)
//   {
//     split = strtok(NULL, " ");

//     if (split == NULL)
//       return cmd;

//     if (strcmp(split, "SET") == 0)
//     {
//       cmd.commandType = COMMAND_ONE_LED;

//       split = strtok(NULL, " ");
//       if (split == NULL)
//       {
//         cmd.commandType = NO_TYPE;
//         return cmd;
//       }

//       int8_t id = stoi(split);
//       if (id < 0)
//       {
//         cmd.commandType = NO_TYPE;
//         return cmd;
//       }

//       cmd.ledId = (uint8_t)id;
//     }
//     else if (strcmp(split, "SETALL") == 0)
//     {
//       cmd.commandType = COMMAND_ALL_LEDS;
//     }
//     else
//     {
//       return cmd;
//     }

//     split = strtok(NULL, " ");
//     if (split == NULL)
//     {
//       cmd.commandType = NO_TYPE;
//       return cmd;
//     }

//     if (strcmp(split, "ON") == 0)
//     {
//       cmd.status = LED_ON;
//     }
//     else if (strcmp(split, "OFF") == 0)
//     {
//       cmd.status = LED_OFF;
//     }
//     else if (strcmp(split, "BLINK") == 0)
//     {
//       cmd.status = LED_BLINKING;
//     }
//     else
//     {
//       cmd.commandType = NO_TYPE;
//     }
//   }
//   else if (strcmp(split, "MSG"))
//   {
//     cmd.commandType = MESSAGE_COMMAND;

//     split = strtok(NULL, " ");
//     if (split == NULL)
//     {
//       cmd.commandType = NO_TYPE;
//       return cmd;
//     }

//     cmd.message = split;
//   }

//   return cmd;
// }

// Command cmd;

// void setup()
// {
//   Serial.begin(9600);
// }

// void loop()
// {
//   String command = Serial.readStringUntil('\n');
//   cmd = parseCommand(command);
//   Serial.write(cmd.commandType + '0');
//   Serial.write("\n");
//   Serial.write(cmd.ledId + '0');
//   Serial.write("\n");
//   Serial.write(cmd.status + '0');
//   Serial.write("\n");
// }