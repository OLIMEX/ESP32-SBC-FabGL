/**
 * @brief ESP32-SBC-FabGL interrupt example
 * Tested with:
 * Board revision   : Rev.A
 * Arduino IDE      : 1.8.9
 * Espressif package: 2.0.9
 * 
 * Example description:
 * Example will check the version of the CH32V003 firmware expander
 * Interrupt feature is not available before 1.0v
 * 
 * If initialized and configured properly
 * when an interrupt occure on the expander it will
 * send signal to the ESP32 via IO_EXP_IRQ line
 * and then the ESP32 will check the status of the
 * pins that could be initialized as interrupt pins.
 * Such pins are UEXT 7-10 and could be configured:
 * on rising edge : FRONT_RISING
 * on falling edge: FRONT_FALLING
 * on either edge : FRONT_CHANGE
 * 
*/
#include "CH32V003.h"

CH32V003 Expander;

void error(char message[])
{
  Serial.println(message);
  while (1);
}

void PrintOutput (char Label[], uint8_t reg, uint8_t flags)
{
  Serial.printf ("%s ", Label);

  for (int8_t i=7; i>=0; i--)
    if (flags & (1 << i))
      Serial.printf("%d", (reg & flags) != 0);
    else
      Serial.printf("-");

  Serial.println();
}

bool ScanPinValue (uint8_t port)
{
  uint8_t flags = Expander.getPortIntFlags(port);
  if (flags)
  {
    Serial.printf("Port%c\n\r", port|0x40);
    Serial.printf ("Flags = 0x%02X\n\r", flags);
    if (flags >= 0xFA)
    {
      Serial.printf("PRESS A KEY\r\n");
      while (!Serial.available ());
      Serial.read();  // read the character
      return false;
    }
    uint8_t capture = Expander.getPortIntCaptured(port);
    PrintOutput ("Capture", capture, flags);
  }
  return true;
}

void InitExpander ()
{
  // UEXT 7
  Expander.configureGPIO  (UEXT_GPIO_7, DIRECTION_IN,  0x00); // PA2 input, pull down
  Expander.enableInterrupt(UEXT_GPIO_7, FRONT_CHANGE);

  // UEXT 8
  Expander.configureGPIO  (UEXT_GPIO_8, DIRECTION_IN,  0x00); // PA1 input, pull down
  Expander.enableInterrupt(UEXT_GPIO_8, FRONT_CHANGE);
  
  // UEXT 9
  Expander.configureGPIO  (UEXT_GPIO_9, DIRECTION_IN,  0x00); // PD4 input, pull down
  Expander.enableInterrupt(UEXT_GPIO_9, FRONT_CHANGE);

  // UEXT 10
  Expander.configureGPIO  (UEXT_GPIO_10, DIRECTION_IN,  0x00); // PD3 input, pull down
  Expander.enableInterrupt(UEXT_GPIO_10, FRONT_CHANGE);
}

void setup()
{
  Serial.begin(115200);

  pinMode(IO_EXP_IRQ, INPUT);
  
  if (Expander.begin ())
  {
    uint16_t ver = Expander.version();
    Serial.printf("CH32V003 firmware version: %d.%d" EOL, ver >> 8, ver & 0xFF);
    if (ver < 0x0100)
      error("Your CH32V003 firmware version does not support features needed by this example");
  }
  else
    error("CH32V003 not found");

  InitExpander ();
}

void loop ()
{
  if (digitalRead(IO_EXP_IRQ))
  {
    if (!ScanPinValue (GPIO_PORTA)) return;
    if (!ScanPinValue (GPIO_PORTD)) return;
    Serial.println ("------------------------------------------------------------------");
  }
  
  if (Serial.available ())  // reinit the extender on character
  {
    Serial.read();  // read the character
    InitExpander ();
  }
}
