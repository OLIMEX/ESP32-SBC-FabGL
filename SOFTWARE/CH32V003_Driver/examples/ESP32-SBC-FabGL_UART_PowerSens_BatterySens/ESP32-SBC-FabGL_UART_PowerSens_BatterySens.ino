/**
 * @brief ESP32-SBC-FabGL UART, power and battery sens example
 * Tested with:
 * Board revision   : Rev.A
 * Arduino IDE      : 1.8.9
 * Espressif package: 2.0.9
 * 
 * Example description:
 * Example will check the version of the CH32V003 firmware expander
 * UART, Power sens and battery sens features are not available before 1.0v
 * 
 * Upon changing of the value of the power sens (PD0) an interrupt
 * will be generated from the expander and the IO_EXP_IRQ will be risen
 * indicating for an interrupt to the ESP32. Then the ESP32 will check
 * the value of the power sens pin and print message on a terminal
 * attached to the UEXT (since the native UART will be unavailable).
 * The message is based on the status of the pin:
 * "On external power" if the USB is plugged
 * or
 * "On battery" if the USB is unplugged,
 * followed by message with the voltage of the battery every second
 * until the USB is plugged in again
 * 
*/

#include "CH32V003.h"
#include <string.h>

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

  Expander.configureUART (115200, UART_StopBits_1, UART_Parity_No);

  Serial.println ("Power Sense example");
  Serial.println ("Connect the battery, plug another UART device on the UEXT to monitor the result and then unplug external power");
  Expander.strWriteUART("Power Sense example\r\n");
  Expander.strWriteUART("Connect the battery and then unplug external power\r\n");

  pinMode (IO_EXP_IRQ, INPUT);
}

char OutputMessage[30];
uint8_t OnBattery=0;
uint32_t Time=0, PrevTime=0;
void loop()
{
  if (digitalRead(IO_EXP_IRQ))
    if (Expander.powerSense())
    {
      OnBattery = 0;
      Expander.strWriteUART("On external power\n\r");
    }
    else
    {
      OnBattery = 1;
      Expander.strWriteUART("On battery\n\r");
    }


  Time = millis ();
  if (OnBattery && (Time-PrevTime>1000))
  {
    bat_sense = Expander.batterySense();
    bat_percent = Expander.batteryPercent(bat_sense);
    PrevTime = Time;
    sprintf (OutputMessage, "Battery level: %d mV (%d%%)\n\r", bat_sense, bat_percent);
    Expander.strWriteUART(OutputMessage);
  }
}