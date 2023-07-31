/**
 * @brief ESP32-SBC-FabGL 2 I2C slaves example
 * Tested with:
 * Board revision   : Rev.A
 * Arduino IDE      : 1.8.9
 * Espressif package: 2.0.9
 * 
 * Example description:
 * Example will print the version of the CH32V003 firmware expander
 * 
 * A combined example of the I2C_MOD-RTC2 and I2C_Nunchuk examples
 * to demonstrate functinoality of more than one I2C module
 * with different I2C addresses.
 * Every 200 milliseconds it will print the values from the nunchuk:
 * joystick X/Y,
 * accelerometer X/Y/Z
 * button C/Z
 * and every second it will print the time from the MOD-RTC2.
 * 
*/

#include "CH32V003.h"
#include "MOD-RTC2.h"

#define MOD_RTC2_ADDRESS (0x68)
#define NUNCHUCK_ADDRESS (0x52)

const char DayOfTheWeek[][10] {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

CH32V003 Expander;

int Century=20, Year, Month, Date, Day, Hours, Minutes, Seconds, Prev_Sec=-1;
double Temp;
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


  // Setup date, time, and 2 alarms to
  // 1999/12/31 (FRIDAY) 23:59:50
  //               YY  MM  DD   DOTW
  MOD_RTC2_SetDate(99, 12, 31, FRIDAY);
  //               HH  MM  SS
  MOD_RTC2_SetTime(23, 59, 50);
  //                 HH  MM  SS     DD          Setting
  MOD_RTC2_SetAlarm1( 0,  0,  5, SATURDAY, ALARM_EVERY_DOTW);
  //                 HH  MM, DD,     Setting
  MOD_RTC2_SetAlarm2( 0,  0,  1, ALARM_EVERY_DATE);


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
  Serial.printf ("JX = %3d\tJY = %3d\t", BuffIn[0], BuffIn[1]);
  Serial.printf ("AX = %3d\tAY = %3d\tAZ = %3d\t", BuffIn[2], BuffIn[3], BuffIn[4]);
  Serial.printf ("C = %d; Z = %d\n\r", BuffIn[5] & 0x02?0:1, BuffIn[5] & 0x01?0:1);
  
  MOD_RTC2_UpdateData ();

  Year    = MOD_RTC2_GetYear ();
  Month   = MOD_RTC2_GetMonth ();
  Date    = MOD_RTC2_GetDate ();
  Day     = MOD_RTC2_GetDayOfTheWeek ();
  Hours   = MOD_RTC2_GetHours ();
  Minutes = MOD_RTC2_GetMinutes ();
  Seconds = MOD_RTC2_GetSeconds ();
  Temp    = MOD_RTC2_GetTemperature ();

  if (MOD_RTC2_CheckCentury())
  {
    Century++;
    printf ("New century!\r\n");
  }

  if (MOD_RTC2_CheckAlarm1())
    printf ("Alarm 1 triggered!\r\n");

  if (MOD_RTC2_CheckAlarm2())
    printf ("Alarm 2 triggered!\r\n");

  if (Seconds != Prev_Sec)
  {
    Serial.printf ("      %2d/%2d/%2d (%s) %02d:%02d:%02d;  Temperature = %3.2lf C\n\r", (Century-1)*100+Year, Month, Date, DaysOfTheWeek[Day], Hours, Minutes, Seconds, Temp); // if float is supported in printf
    //Serial.printf ("%2d/%2d/%2d (%s) %02d:%02d:%02d;  Temperature = %d C\r\n", (Century-1)*100+Year, Month, Date, DaysOfTheWeek[Day], Hours, Minutes, Seconds, (int)Temp); // if float is not supported in printf
    Prev_Sec = Seconds;
  }
  delay (200);
}
