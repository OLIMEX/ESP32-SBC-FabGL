/**
 * @brief ESP32-SBC-FabGL battery sens example
 * Tested with:
 * Board revision   : Rev.A
 * Arduino IDE      : 1.8.9
 * Espressif package: 2.0.9
 * 
 * Example description:
 * Example will check the version of the CH32V003 firmware expander
 * Battery sens feature is not available before 1.0v
 * 
 * If initialized properly every second should print the value
 * of the analog input (on the expander) connected to the
 * battery sens pin - PD2 (analog channel 3)
 * 
*/

#include "CH32V003.h"

CH32V003 Expander;

uint16_t bat_sense;
uint8_t  bat_percent;

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
}

void loop ()
{
  bat_sense = Expander.batterySense();
  bat_percent = Expander.batteryPercent(bat_sense);
  Serial.printf ("Battery measurement: %d mV (%d%%)" EOL, bat_sense, bat_percent);
  delay (1000);
}