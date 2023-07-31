/**
 * @brief ESP32-SBC-FabGL UART echo example
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
 * If initialized and configured properly upon receiving a character
 * on the expander's RX it will return echo (next ASCII character) on the TX line
 * To achieve that the IO_EXP_IRQ will be set (indicating an interrupt on the expander)
 * the ESP32 will receive the character, process it (adding 1 to it) and then send it back
 * which will then be transmitted on the expander's TX line
 * 
*/

#include "CH32V003.h"

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
  static uint8_t Char=0;
  if (digitalRead(IO_EXP_IRQ))
  {
    Expander.readUART (&Char, 1);
    Char = Char + 1;  // returns the next ASCII character
    Expander.writeUART (&Char, 1);
  }
}
