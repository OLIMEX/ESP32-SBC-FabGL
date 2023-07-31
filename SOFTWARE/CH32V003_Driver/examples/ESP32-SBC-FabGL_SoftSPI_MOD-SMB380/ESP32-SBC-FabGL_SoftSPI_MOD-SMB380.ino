/**
 * @brief ESP32-SBC-FabGL Software SPI MOD-SMB380 example
 * Tested with:
 * Board revision   : Rev.A
 * Arduino IDE      : 1.8.9
 * Espressif package: 2.0.9
 * 
 * Example description:
 * Example will print the version of the CH32V003 firmware expander
 * 
 * The example will scan the data from the SMB380
 * and print them every second. The values are:
 * Chip ID, al, ml
 * Accelerometer X, Y, Z
 * Temperature
 * 
*/

#include "CH32V003.h"

CH32V003 Expander;

uint16_t tx_buf[9], rx_buf[9], i;
int16_t Value;

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
  Expander.configureSPI (3, 50000);
}

void loop()
{
  for (i=0; i<9; i++)
    // first byte of the word to be sent in SPI should have a 1 in the MSB (meaning we are reading)
    // and the LSB consist of the index of the register to be read.
    // The second byte of the word is sent only for the SPI transfer
    tx_buf[i] = 0x8000 | (i<<8);  
  Expander.transferSPI16 (tx_buf, rx_buf, 9);

  Serial.printf ("Chip data:\n\r");
  Serial.printf ("  Chip ID: 0x%X\n\r", rx_buf[0] & 0x07);
  Serial.printf ("  al version: 0x%X\n\r", rx_buf[1]>>4);
  Serial.printf ("  ml version: 0x%X\n\r", rx_buf[1]&0xF);
  
  Serial.printf ("Accelerometer data:\n\r");
  // the high data byte should be first shifted 8 position to the left and then 6 to the right (/64) (instead of only 2 to the left) to achieve 2's complement value.
  if (rx_buf[2] & 0x01) // new data on X axis
  {
    Value = (int16_t)((rx_buf[3] << 8) | rx_buf[2]) / 64;
    Serial.printf ("  X axis = %4d" EOL, Value);
  }

  if (rx_buf[4] & 0x01) // new data on Y axis
  {
    Value = (int16_t)((rx_buf[5] << 8) | rx_buf[4]) / 64;
    Serial.printf ("  Y axis = %4d" EOL, Value);
  }

  if (rx_buf[6] & 0x01) // new data on Z axis
  {
    Value = (int16_t)((rx_buf[7] << 8) | rx_buf[6]) / 64;
    Serial.printf ("  Z axis = %4d" EOL, Value);
  }

  Serial.printf ("Temperature data:\n\r");
  Value = (rx_buf[8]/2) - 30;
  Serial.printf ("  T = %d" EOL, Value);
  Serial.println ("============================================================");
  
  delay(1000);
}
