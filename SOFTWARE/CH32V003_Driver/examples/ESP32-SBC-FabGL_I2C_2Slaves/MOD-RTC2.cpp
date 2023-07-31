#include "MOD-RTC2.h"

extern CH32V003 Expander;

const char DaysOfTheWeek[NUMBER_OF_DAYS][10] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
static unsigned char Read_Buff[NUMBER_OF_REGS];


// all of the numbers in the DS3231 registers are stored as binary coded decimals (BCD) so we need convertion between the two

unsigned char BCDtoInt(unsigned char BCD)
{
    unsigned char a, b;

    a = BCD & 0x0F;
    b = BCD >> 4;

    return b*10 + a;
}

unsigned char InttoBCD(unsigned char Int)
{
    unsigned char a, b;

    a = Int % 10;
    b = (Int / 10) % 10;

    return (b << 4) + a;
}

void MOD_RTC2_SetTime (int Hours, int Minutes, int Seconds)
{
    Hours   = InttoBCD(Hours);
    Minutes = InttoBCD(Minutes);
    Seconds = InttoBCD(Seconds);

    I2C_SendDataToSlave (MOD_RTC2, SECONDS , Seconds);    // seconds
    I2C_SendDataToSlave (MOD_RTC2, MINUTES , Minutes);    // minutes
    I2C_SendDataToSlave (MOD_RTC2, HOURS   , Hours);      // hours
}

void MOD_RTC2_SetDate (int Year, int Month, int Date, int DayOfTheWeek)
{
    Year         = InttoBCD (Year);
    Month        = InttoBCD (Month);
    Date         = InttoBCD (Date);
    DayOfTheWeek = InttoBCD (DayOfTheWeek);

    I2C_SendDataToSlave (MOD_RTC2, DAY_OF_WEEK, DayOfTheWeek);  // day of the week
    I2C_SendDataToSlave (MOD_RTC2, DATE       , Date);          // date
    I2C_SendDataToSlave (MOD_RTC2, MONTH      , Month);         // month
    I2C_SendDataToSlave (MOD_RTC2, YEAR       , Year);          // year (last 2 digits)
}

void MOD_RTC2_SetAlarm1 (int Hours, int Minutes, int Seconds, int Day_Date, int Setting)
{
    Day_Date = InttoBCD(Day_Date) | (((Setting>>3) & 0x01)<<7) | (Setting & 0x40); // A1M4 + DY/!DT
    Hours    = InttoBCD(Hours)     | (((Setting>>2) & 0x01)<<7); // A1M3
    Minutes  = InttoBCD(Minutes)   | (((Setting>>1) & 0x01)<<7); // A1M2
    Seconds  = InttoBCD(Seconds)   | (((Setting>>0) & 0x01)<<7); // A1M1

    I2C_SendDataToSlave (MOD_RTC2, ALARM1_SECONDS , Seconds);    // seconds
    I2C_SendDataToSlave (MOD_RTC2, ALARM1_MINUTES , Minutes);    // minutes
    I2C_SendDataToSlave (MOD_RTC2, ALARM1_HOURS   , Hours);      // hours
    I2C_SendDataToSlave (MOD_RTC2, ALARM1_DAY_DATE, Day_Date);   // day of the week/date of the month
}

void MOD_RTC2_SetAlarm2 (int Hours, int Minutes, int Day_Date, int Setting)
{
    Day_Date = InttoBCD(Day_Date) | (((Setting>>3) & 0x01)<<7) | (Setting & 0x40); // A2M4 + DY/!DT
    Hours    = InttoBCD(Hours)     | (((Setting>>2) & 0x01)<<7); // A2M3
    Minutes  = InttoBCD(Minutes)   | (((Setting>>1) & 0x01)<<7); // A2M2

    I2C_SendDataToSlave (MOD_RTC2, ALARM2_MINUTES , Minutes);    // minutes
    I2C_SendDataToSlave (MOD_RTC2, ALARM2_HOURS   , Hours);      // hours
    I2C_SendDataToSlave (MOD_RTC2, ALARM2_DAY_DATE, Day_Date);   // day of the week/date of the month
}

void MOD_RTC2_UpdateData ()
{
    I2C_SendDataToSlave (MOD_RTC2, CTRL_REGISTER, 0x3C);      // Control register -> Convert temperature enabled (this bit has to be raised constantly), 8.192 kHz

    for (int i=0; i<NUMBER_OF_REGS; i++)
        Read_Buff[i] = I2C_ReadDataFromSlave (MOD_RTC2, i);
}

int MOD_RTC2_CheckCentury ()
{
    // bit 7 of the month register (address 0x05) store new century occurrence
    if (Read_Buff[MONTH] & 0x80)
    {
        I2C_SendDataToSlave (MOD_RTC2, MONTH, Read_Buff[MONTH] & 0x7F);   // clearing the new century flag
        return 1;
    }
    return 0;
}

int MOD_RTC2_CheckAlarm1 ()
{
    if (Read_Buff[STAT_REGISTER] & 0x01)
    {
        I2C_SendDataToSlave (MOD_RTC2, STAT_REGISTER, Read_Buff[STAT_REGISTER] & (~0x01));   // clearing the Alarm1 flag
        return 1;
    }
    return 0;
}

int MOD_RTC2_CheckAlarm2 ()
{
    if (Read_Buff[STAT_REGISTER] & 0x02)
    {
        I2C_SendDataToSlave (MOD_RTC2, STAT_REGISTER, Read_Buff[STAT_REGISTER] & (~0x02));   // clearing the Alarm2 flag
        return 1;
    }
    return 0;
}

int MOD_RTC2_GetYear ()
{
    return BCDtoInt (Read_Buff[YEAR] & YEAR_MASK);
}

int MOD_RTC2_GetMonth ()
{
    return BCDtoInt (Read_Buff[MONTH] & MONTH_MASK);
}

int MOD_RTC2_GetDate ()
{
    return BCDtoInt (Read_Buff[DATE] & DATE_MASK);
}

int MOD_RTC2_GetDayOfTheWeek ()
{
    return BCDtoInt (Read_Buff[DAY_OF_WEEK] & DOTW_MASK);
}

int MOD_RTC2_GetHours ()
{
    return BCDtoInt (Read_Buff[HOURS] & HOURS_MASK);
}

int MOD_RTC2_GetMinutes ()
{
    return BCDtoInt (Read_Buff[MINUTES] & MINUTES_MASK);
}

int MOD_RTC2_GetSeconds ()
{
    return BCDtoInt (Read_Buff[SECONDS] & SECONDS_MASK);
}

double MOD_RTC2_GetTemperature ()
{
    return (double)(((signed int)((Read_Buff[TEMPERATURE_H]<<8) | Read_Buff[TEMPERATURE_L]))>>6)/4.0;;
}
