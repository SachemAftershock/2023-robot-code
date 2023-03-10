// #include <Arduino.h>
// #include <Wire.h>

// uint8_t LED_Start = 2;
// uint8_t LED_End = 26;

// void setup()
// {
//     // put your setup code here, to run once:
//     Serial.begin(9600);
//     for (size_t i = LED_Start; i <= LED_End; i++)
//     {
//         if (i == 20 || i == 21)
//             continue;
//         pinMode(i, OUTPUT);
//     }
// }

// void loop()
// {
//     // put your main code here, to run repeatedly:
//     for (size_t i = LED_Start; i <= LED_End; i++)
//     {
//         if (i == 20 || i == 21)
//             continue;
//         digitalWrite(i, HIGH);
//         delay(500);
//     }
//     delay(400);
//     for (size_t i = LED_Start; i <= LED_End; i++)
//     {
//         digitalWrite(i, LOW);
//     }
//     delay(400);
// }