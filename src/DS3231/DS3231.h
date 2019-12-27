/*
 * DS3231.h
 * (c) Alex Gerchanovsky
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <time.h>

// Simple general-purpose date/time class (no TZ / DST / leap second handling!)
#ifndef I2C_ADDR_DS3231
#define I2C_ADDR_DS3231 0x68
#endif

// tm_sec    The number of seconds after the minute, normally in the range 0 to 59, but can be up to 60 to allow for leap seconds.
// tm_min    The number of minutes after the hour, in the range 0 to 59.
// tm_hour   The number of hours past midnight, in the range 0 to 23.
// tm_mday   The day of the month, in the range 1 to 31.
// tm_mon    The number of months since January, in the range 0 to 11.
// tm_year   The number of years since 1900.
// tm_wday   The number of days since Sunday, in the range 0 to 6.
// tm_yday   The number of days since January 1, in the range 0 to 365.
// tm_isdst  A flag that indicates whether daylight saving time is in effect at the time described.
//           The value is positive if daylight saving time is in effect, zero if it is not, and negative if the information is not available.

class DS3231 {
public:
    void begin() {
        // sets the mode to 12-hour (true) or 24-hour (false).
        // One thing that bothers me about how I've written this is that
        // if the read and right happen at the right hourly millisecnd,
        // the clock will be set back an hour. Not sure how to do it better,
        // though, and as long as one doesn't set the mode frequently it's
        // a very minimal risk.
        // It's zero risk if you call this BEFORE setting the hour, since
        // the setHour() function doesn't change this mode.

        // force 24h
        writeRegister(0x02, readRegister(0x02) & ~0b01000000);
        // Turn off alarms
        // turn off BBSQW flag
        //Serial.printf("DS3231 regs: 0E=%02X 0F=%02X\r\n", readRegister(0x0e), readRegister(0x0f));
//1C,08
/*
76543210
00011100b
EOSC=0  Enabled: Oscillator
BBSQW=0 Disabled:Battery-Backed Square-Wave
CONV=0  Disabled:Convert Temperature
RS2=1   Square wave set to 8.192kHz
RS1=1
INTCN=1 Enabled:Interrupt Control
A2IE=0  Disabled:Alarm 2 Interrupt
A1IE=0  Disabled:Alarm 1 Interrupt
00001000b
OSF=0   Disabled:Oscillator Stop Flag
EN32kHz=1 Enabled:Enable 32kHz Output
BSY=0   Disabled:Busy
A2F=0   Disabled:Alarm 2 Flag
A1F=0   Disabled:Alarm 1 Flag
*/
        writeRegister(0x0e, 0);
        writeRegister(0x0f, 0);
        if (readRegister(6)==0) //year 2000?
            set(__DATE__, __TIME__);// set to compile time
    /*
        writeRegister(0x0e, (readRegister(0x0e) & ~(1<<6   | //disable BBSQW (Battery-Backed Square-Wave Enable)
                                                    1<<2   | //disable INTCN (Interrupt Control)
                                                    1<<1   | //disable A2IE (Alarm 2 Interrupt Enable)
                                                    1<<0)) | //disable A2IE (Alarm 2 Interrupt Enable)
                                                    1<<7);   //disable battery EOSC (Enable Oscillator)

        // turn off 32kHz pin
        writeRegister(0x0f, readRegister(0x0f) & ~(1<<3));
    */
    }

    static bool get(struct tm &t) {
        Wire.beginTransmission(I2C_ADDR_DS3231);
        Wire.write(0);	// This is the first register address (Seconds)
        		// We'll read from here on for 7 bytes: secs reg, minutes reg, hours, days, months and years.
        Wire.endTransmission();
        Wire.requestFrom(I2C_ADDR_DS3231, 7);
        if (!Wire.available())
            return false;
        t.tm_sec    = bcd2dec(Wire.read() & 0x7F); //00
        t.tm_min    = bcd2dec(Wire.read());        //01
        t.tm_hour   = bcd2dec(Wire.read());        //02
        t.tm_wday   = Wire.read()-1;               //03
        t.tm_mday   = bcd2dec(Wire.read());        //04
        byte b = Wire.read();
        t.tm_mon    = bcd2dec(b & ~(1<<7))-1;//05 clear Century flag
        t.tm_year   = bcd2dec(Wire.read())+((b&(1<<7))?100:0);   //06 starting from 1900
        return true;
    }

    static void set(const struct tm &t) {
        bool century = (t.tm_year>=100);
        Wire.beginTransmission(I2C_ADDR_DS3231);
        Wire.write(0);	// This is the first register address (Seconds)
        		// We'll write from here 7 bytes: secs reg, minutes reg, hours, days, months and years.
        Wire.write(dec2bcd(t.tm_sec));             //00
        Wire.write(dec2bcd(t.tm_min));             //01
        Wire.write(dec2bcd(t.tm_hour));            //02
        Wire.write(dec2bcd(t.tm_wday+1));          //03
        Wire.write(dec2bcd(t.tm_mday));            //04
        Wire.write(dec2bcd(t.tm_mon+1)|(century?(1<<7):0)); //05
        Wire.write(dec2bcd(t.tm_year-(century?100:0)));//06 year stared from 1900
        Wire.endTransmission();
        // This function also resets the Oscillator Stop Flag, which is set whenever power is interrupted.
        // Clear OSF flag
        writeRegister(0x0f, readRegister(0x0f) & ~(1<<7));
    }

private:
    // Get all date/time at once to avoid rollover (e.g., minute/second don't match)
    static uint8_t dec2bcd(uint8_t val) { return ( ((val/10)<<4) + (val%10) ); } // Convert normal decimal numbers to binary coded decimal
    //static uint8_t dec2bcd(uint8_t val) { return val + 6 * (val / 10); }
    static uint8_t bcd2dec(uint8_t val) { return ( ((val>>4)*10) + (val&0xf) ); } // Convert binary coded decimal to normal decimal numbers
    //static uint8_t bcd2dec(uint8_t val) { return val - 6 * (val >> 4); }
    static byte readRegister(byte idx) {
        Wire.beginTransmission(I2C_ADDR_DS3231);
        Wire.write(idx);
        Wire.endTransmission();
        Wire.requestFrom(I2C_ADDR_DS3231, 1);
        return Wire.read();
    }
    static void writeRegister(byte idx, byte val) {
        Wire.beginTransmission(I2C_ADDR_DS3231);
        Wire.write(idx);
        Wire.write(val);
        Wire.endTransmission();
    }
    // Convert compile time to system time
    void set(const char *date, const char *time) {
        char s_month[5];
        int year;
        struct tm t;
        static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
        sscanf(date, "%s %d %d", s_month, &t.tm_mday, &year);
        sscanf(time, "%2d %*c %2d %*c %2d", &t.tm_hour, &t.tm_min, &t.tm_sec);
        // Find where is s_month in month_names. Deduce month value.
        t.tm_mon = (strstr(month_names, s_month) - month_names) / 3;
        t.tm_year = year - 1900;
        t.tm_wday = DayOfWeek(t.tm_year, t.tm_mon+1, t.tm_mday)-1;//The number of days since Sunday, in the range 0 to 6.
        IsDST(t);
        Serial.printf("'%s' '%s'\r\n", date, time);
        Serial.println(&t, NULL);//"%A, %B %d %Y %H:%M:%S");
        set(t);
        get(t);
        Serial.println(&t, NULL);//"%A, %B %d %Y %H:%M:%S");
    }
};
