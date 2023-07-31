/**
 * @brief ESP32-SBC-FabGL UART demo example
 * Tested with:
 * Board revision   : Rev.A
 * Arduino IDE      : 1.8.9
 * Espressif package: 2.0.9
 * 
 * Example description:
 * Example will check the version of the CH32V003 firmware expander
 * UART feature is not available before 1.0v
 * Recommended baudrate up to 115200. On higher values longer messages will cause malfunctions
 * 
 * If initialized and configured properly
 * the ESP32 will send a message to the expander every 2 seconds
 * while the expander will send to the ESP32 every character
 * received on its own RX line (carriage return or line feed will
 * implicitly send the other character to make sure you send a new line)
 * and it will be transmitted on the ESP32's TX line.
 * 
*/

#include "CH32V003.h"
#include <string.h>

#define MAX_LENGTH  250
unsigned char SendMessage[MAX_LENGTH];
unsigned char ReceiveMessage[MAX_LENGTH];

CH32V003 Expander;

void error(char message[])
{
  Serial.println(message);
  while (1);
}

void setup()
{
  Serial.begin(115200);

  if (Expander.begin ())
  {
    uint16_t ver = Expander.version();
    Serial.printf("CH32V003 firmware version: %d.%d" EOL, ver >> 8, ver & 0xFF);
    if (ver < 0x0100)
      error("Your CH32V003 firmware version does not support features needed by this example");
  }
  else
    error("CH32V003 not found");

  Expander.configureUART (115200, UART_StopBits_1, UART_Parity_No);

  pinMode (IO_EXP_IRQ, INPUT);
}

void loop()
{
  static long int Time=0, PrevTime=0, Counter=0;
  Time = millis();
  if (Time - PrevTime > 2000)
  {
    sprintf ((char*)SendMessage, "Hello CH32V003! (from ESP32)   %d" EOL, Counter++);
    Expander.writeUART (SendMessage, strlen((char*)SendMessage));
    PrevTime = Time;
  }

  if (digitalRead(IO_EXP_IRQ))
  {
    uint8_t received = Expander.readUART(ReceiveMessage, sizeof(ReceiveMessage));
    for (int i=0; i<received; i++)
      if ((char)ReceiveMessage[i] == '\r' || (char)ReceiveMessage[i] == '\n')
        Serial.println();
      else   
        Serial.printf ("%c", ReceiveMessage[i]);
  }
}
