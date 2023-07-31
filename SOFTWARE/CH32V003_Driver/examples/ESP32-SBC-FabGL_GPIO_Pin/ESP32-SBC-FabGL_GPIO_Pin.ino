/**
 * @brief ESP32-SBC-FabGL GPIO pin example
 * Tested with:
 * Board revision   : Rev.A
 * Arduino IDE      : 1.8.9
 * Espressif package: 2.0.9
 * 
 * Example description:
 * Example will print the version of the CH32V003 firmware expander
 * 
 * The example demonstrates the feature of reading and writing value
 * of the GPIOs (primarily on the UEXT). To do so attach something
 * on the pins (for example a LED or a button/jumper wire).
 * The example scans the value of an input and set/clear the output accordingly
 * 
 * To select the pin you want to read/write you can use either
 * "GPIO_PORTx, GPIO_x" macros or
 * "UEXTx" instead in case you are using UEXT pins
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

  Expander.configureGPIO (UEXT_GPIO_9, DIRECTION_IN,  0x00); // PD4 input, pull down
  Expander.configureGPIO (UEXT_GPIO_10, DIRECTION_OUT, 0x00); // PD3 output
  // alternatively you can use this initialization code instead
  //Expander.configureGPIO (GPIO_PORTD, GPIO_4, DIRECTION_IN,  0x00); // PD4 input, pull down
  //Expander.configureGPIO (GPIO_PORTD, GPIO_3, DIRECTION_OUT, 0x00); // PD3 output
  
}

void loop ()
{
  Expander.setGPIO (UEXT_GPIO_10, Expander.getGPIO (UEXT_GPIO_9));
}