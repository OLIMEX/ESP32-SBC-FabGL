#ifndef _MOD_RTC2_H_
#define _MOD_RTC2_H_
#include "CH32V003.h"

#define MOD_RTC2   (0x68)

#define I2C_SendDataToSlave(Address, Register, Data)  Expander.writeRegI2C(Address, Register, Data)
#define I2C_ReadDataFromSlave(Address, Register)      Expander.readRegI2C(Address, Register)


// registers map
typedef enum
{
    SECONDS         = 0x00,
    MINUTES         = 0x01,
    HOURS           = 0x02,
    DAY_OF_WEEK     = 0x03,
    DATE            = 0x04,
    MONTH           = 0x05,
    YEAR            = 0x06,

    ALARM1_SECONDS  = 0x07,
    ALARM1_MINUTES  = 0x08,
    ALARM1_HOURS    = 0x09,
    ALARM1_DAY_DATE = 0x0A,

    ALARM2_MINUTES  = 0x0B,
    ALARM2_HOURS    = 0x0C,
    ALARM2_DAY_DATE = 0x0D,

    CTRL_REGISTER   = 0x0E,
    STAT_REGISTER   = 0x0F,

    AGING_OFFSET    = 0x10,
    TEMPERATURE_H   = 0x11,
    TEMPERATURE_L   = 0x12,

    NUMBER_OF_REGS  = 0x13
}MOD_RTC2_Registers;

// masking for the BCD to int conversion
#define SECONDS_MASK    0x7F
#define MINUTES_MASK    0x7F
#define HOURS_MASK      0x3F
#define DOTW_MASK       0x07
#define DATE_MASK       0x3F
#define MONTH_MASK      0x1F
#define YEAR_MASK       0xFF

// alarm 1 and 2 settings
// for details refer to DS3231 datasheet, the table with Alarm mask bits
// since alarm 2 doesn't have a seconds settings it will be triggered on 00 seconds
#define ALARM_EVERY_SECOND 0x0F    // Alarm once per second
#define ALARM_EVERY_MINUTE 0x0E    // Alarm when seconds match
#define ALARM_EVERY_HOUR   0x0C    // Alarm when minutes and seconds match
#define ALARM_EVERY_DAY    0x08    // Alarm when hours, minutes and seconds match
#define ALARM_EVERY_DATE   0x00    // Alarm when date, hours, minutes and seconds match
#define ALARM_EVERY_DOTW   0x40    // Alarm when day of the week, hours, minutes and seconds match


typedef enum
{
    SUNDAY         = 0,
    MONDAY         = 1,
    TUESDAY        = 2,
    WEDNESDAY      = 3,
    THURSDAY       = 4,
    FRIDAY         = 5,
    SATURDAY       = 6,
    NUMBER_OF_DAYS = 7
}MOD_RTC2_DOTW;
extern const char DaysOfTheWeek[NUMBER_OF_DAYS][10];


unsigned char BCDtoInt(unsigned char BCD);
unsigned char InttoBCD(unsigned char Int);

void MOD_RTC2_SetTime (int Hours, int Minutes, int Seconds);
void MOD_RTC2_SetDate (int Year, int Month, int Date, int DayOfTheWeek);
void MOD_RTC2_SetAlarm1 (int Hours, int Minutes, int Seconds, int Day_Date, int Setting);
void MOD_RTC2_SetAlarm2 (int Hours, int Minutes, int Day_Date, int Setting);

void MOD_RTC2_UpdateData ();

int MOD_RTC2_CheckCentury ();
int MOD_RTC2_CheckAlarm1 ();
int MOD_RTC2_CheckAlarm2 ();

int MOD_RTC2_GetYear ();
int MOD_RTC2_GetMonth ();
int MOD_RTC2_GetDate ();
int MOD_RTC2_GetDayOfTheWeek ();
int MOD_RTC2_GetHours ();
int MOD_RTC2_GetMinutes ();
int MOD_RTC2_GetSeconds ();
double MOD_RTC2_GetTemperature ();

#endif
