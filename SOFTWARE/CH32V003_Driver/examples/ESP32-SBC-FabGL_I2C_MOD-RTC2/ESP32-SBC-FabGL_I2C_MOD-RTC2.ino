/**
 * @brief ESP32-SBC-FabGL I2C MOD-RTC2 example
 * Tested with:
 * Board revision   : Rev.A
 * Arduino IDE      : 1.8.9
 * Espressif package: 2.0.9
 * 
 * Example description:
 * Example will print the version of the CH32V003 firmware expander
 * 
 * The example demonstrates the I2C feature using MOD-RTC2.
 * The program will scan the data of the MOD-RTC2 but print
 * them only whenever there is a change of the time (every second)
 * 
 * In order to work connect a MOD-RTC2 module to the UEXT
 * of the ESP32-SBC-FabGL host board.
 * 
*/

#include "CH32V003.h"
#include "MOD-RTC2.h"

#define MOD_RTC2_ADDRESS MOD_RTC2

const char DayOfTheWeek[][10] {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

CH32V003 Expander;

int Century=20, Year, Month, Date, Day, Hours, Minutes, Seconds, Prev_Sec=-1;
double Temp;

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
}

void loop()
{
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
    Serial.printf ("%2d/%2d/%2d (%s) %02d:%02d:%02d;  Temperature = %3.2lf C\n\r", (Century-1)*100+Year, Month, Date, DaysOfTheWeek[Day], Hours, Minutes, Seconds, Temp); // if float is supported in printf
    //Serial.printf ("%2d/%2d/%2d (%s) %02d:%02d:%02d;  Temperature = %d C\r\n", (Century-1)*100+Year, Month, Date, DaysOfTheWeek[Day], Hours, Minutes, Seconds, (int)Temp); // if float is not supported in printf
    Prev_Sec = Seconds;
  }
  delay(50);
}
