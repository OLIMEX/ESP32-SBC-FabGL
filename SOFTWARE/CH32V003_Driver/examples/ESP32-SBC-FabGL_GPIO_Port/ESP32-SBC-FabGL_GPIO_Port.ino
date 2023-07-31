/**
 * @brief ESP32-SBC-FabGL GPIO port example
 * Tested with:
 * Board revision   : Rev.A
 * Arduino IDE      : 1.8.9
 * Espressif package: 2.0.9
 * 
 * Example description:
 * Example will print the version of the CH32V003 firmware expander
 * 
 * The example demonstrates the feature of reading and writing value
 * of the GPIO port (meaning you can manipulate multiple pins at once).
 * To do so attach something on the pins (for example a LED or a button/jumper wire).
 * The example scans the value of an input and set/clear the output accordingly
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
  }
  else
    error("CH32V003 not found");

  Expander.configurePort (
    GPIO_PORTD,                 // PORTD is selected
    GPIO_Pin_3 | GPIO_Pin_4,    // Configured pins: only 3 and 4 will be configured
    (~GPIO_Pin_3) | GPIO_Pin_4, // Pin directions: PD3 Output (0), PD4 Input (1)
    ~GPIO_Pin_4                 // Pull up/down: GPIO4 pulled down (0)
  );
}

void loop ()
{
  if (Expander.getPort (GPIO_PORTD, GPIO_Pin_4))  // scanning the state of PD4
    Expander.setPort (GPIO_PORTD, GPIO_Pin_3, GPIO_Pin_3);  // PD3 = 1
  else
    Expander.setPort (GPIO_PORTD, GPIO_Pin_3, ~GPIO_Pin_3); // PD3 = 0
}
