/**
 * @brief ESP32-SBC-FabGL I2C Nunchuk example
 * Tested with:
 * Board revision   : Rev.A
 * Arduino IDE      : 1.8.9
 * Espressif package: 2.0.9
 * 
 * Example description:
 * Example will print the version of the CH32V003 firmware expander
 * 
 * The example will scan the data from the Nunchuk and print them
 * every 100 milliseconds. The values are:
 * X and Y axis of the Joystick
 * X, Y and Z axis of the Accelerometer
 * Buttons C and Z
 * 
*/

#include "CH32V003.h"

#define NUNCHUCK_ADDRESS 0x52

CH32V003 Expander;

uint8_t BuffOut[2]={0xF0, 0x55}, BuffIn[6];

void error(char message[])
{
  Serial.println(message);
  while (1);
}

void setup()
{
  Serial.begin (115200);
  
  if (Expander.begin ())
  {
    uint16_t ver = Expander.version();
    Serial.printf("CH32V003 firmware version: %d.%d" EOL, ver >> 8, ver & 0xFF);
  }
  else
    error("CH32V003 not found");

  Expander.configureI2C (400000);

  // Initialize Nunchuk
  Expander.writeI2C (NUNCHUCK_ADDRESS, BuffOut, 2);
  delay (10);
  Expander.readI2C (NUNCHUCK_ADDRESS, BuffIn, 6);
  BuffOut [0] = 0;
}

void loop()
{
  Expander.writeI2C (NUNCHUCK_ADDRESS, BuffOut, 1);
  delay (10);
  Expander.readI2C (NUNCHUCK_ADDRESS, BuffIn, 6);
  Serial.printf ("Joystick     : X = %3d (0x%02X) Y = %3d (0x%02X)\n\r", BuffIn[0], BuffIn[0], BuffIn[1], BuffIn[1]);
  Serial.printf ("Accelerometer: X = %3d (0x%02X) Y = %3d (0x%02X) Z = %3d (0x%02X)\n\r", BuffIn[2], BuffIn[2], BuffIn[3], BuffIn[3], BuffIn[4], BuffIn[4]);
  Serial.printf ("Buttons      : C = %d; Z = %d\n\r", BuffIn[5] & 0x02?0:1, BuffIn[5] & 0x01?0:1);
  Serial.printf ("\n\r");
  delay (100);
}
