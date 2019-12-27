#include "DST.h"

static const char* const wday2str[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
void printDateTime(const struct tm _dt) {
    Serial.printf("%4d/%02d/%02d %s %d:%02d:%02d DST=%d\r\n",
        _dt.tm_year+1900, _dt.tm_mon+1, _dt.tm_mday, wday2str[_dt.tm_wday],
        _dt.tm_hour, _dt.tm_min, _dt.tm_sec, _dt.tm_isdst);
}

/*
__inline bool LEAP_YEAR(uint16_t Y) {
    Y += 1970;
    return (Y>0) && !(Y%4) && ((Y%100) || !(Y%400));
}

// Converts a unix time stamp to a tm structure
static void ConvertUnixTimestamp(uint32_t _t, struct tm &_dt) {
    uint8_t _year, _month;
    unsigned long _days;

    _dt.tm_sec = _t % 60;
    _t /= 60; // now it is minutes
    _dt.tm_min = _t % 60;
    _t /= 60; // now it is hours
    _dt.tm_hour = _t % 24;
    _t /= 24; // now it is _days
    _dt.tm_wday = ((_t + 4) % 7) + 1;  // Sunday is day 1

    _year = 0;
    _days = 0;
    while ((unsigned)(_days += (LEAP_YEAR(_year) ? 366 : 365)) <= _t) {
        _year++;
    }
    _dt.tm_year = _year+(1970-1900); // year is offset from 1970

    int leap_year = LEAP_YEAR(_year)?1:0;
    _days -= (365 + leap_year);
    _t  -= _days; // now it is days in this year, starting at 0

    _days = 0;
    _month = 0;
    for (_month = 0; _month < 12; _month++) {
        static const uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };
        uint8_t _monthLength = pgm_read_byte(daysInMonth + _month);
        if (_month == 1)  // february
            _monthLength += leap_year;

        if (_t < _monthLength)
            break;
        _t -= _monthLength;
    }
    _dt.tm_mon = _month;      // jan is month 0
    _dt.tm_mday = _t + 1;     // day of month
}
*/
// time zone is the difference to UTC in hours
// if _isDayLightSaving is true, time will be adjusted accordingly
// Use returned time only after checking "ret.valid" flag
bool localTime(time_t _unixTime, struct tm *info) {

    //time(&now);
    //localtime_r(&_unixTime, info);
    gmtime_r(&_unixTime, info);
    if (info->tm_year < (2016 - 1900))
        return false;
/*
    //_dt.__TM_GMTOFF = (unsigned long)(_timeZone * 3600.0);
    _unixTime += gmtOffset_sec; // adjust timezone
    ConvertUnixTimestamp(_unixTime, _dt);
    if ((adjust == t_daylightsaving::ST && summerTime(*info)) ||         // European Summer time
        (adjust == t_daylightsaving::DST && daylightSavingTime(*info))) { // US daylight time
        _unixTime += 3600;
        ConvertUnixTimestamp(_unixTime, *info);
        info->tm_isdst = true;
    } else {
        info->tm_isdst = false;
    }
*/
    return true;
}

// Implementation due to Tomohiko Sakamoto
byte DayOfWeek(int y, byte m, byte d)
{   // y > 1752, 1 <= m <= 12
    static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    y -= m < 3;
    return ((y + y/4 - y/100 + y/400 + t[m-1] + d) % 7) + 1; // 01...07, 01 = Sunday
}

//
// Summertime calculates the daylight saving time for middle Europe. Input: Unixtime in UTC
//
bool summerTime(const struct tm &_dt) {
    if (_dt.tm_mon < (3-1) || _dt.tm_mon > (10-1)) return false; // keine Sommerzeit in Jan, Feb, Nov, Dez
    if (_dt.tm_mon > (3-1) && _dt.tm_mon < (10-1)) return true; // Sommerzeit in Apr, Mai, Jun, Jul, Aug, Sep
    return (_dt.tm_mon == (3-1)  && (_dt.tm_hour + 24 * _dt.tm_mday) >= (3 +  24 * (31 - (5 * (_dt.tm_year+1900) / 4 + 4) % 7))) ||
           (_dt.tm_mon == (10-1) && (_dt.tm_hour + 24 * _dt.tm_mday) <  (3 +  24 * (31 - (5 * (_dt.tm_year+1900) / 4 + 1) % 7)));
}

bool daylightSavingTime(const struct tm &_dt)
{
    // here the US code
    //return false;
    // see http://stackoverflow.com/questions/5590429/calculating-daylight-saving-time-from-only-date
    // since 2007 DST begins on second Sunday of March and ends on first Sunday of November.
    // Time change occurs at 2AM locally
    if (_dt.tm_mon < (3-1) || _dt.tm_mon > (11-1)) return false;  //January, february, and december are out.
    if (_dt.tm_mon > (3-1) && _dt.tm_mon < (11-1)) return true;   //April to October are in
    int previousSunday = _dt.tm_mday - (_dt.tm_wday - 1);  // dow Sunday input was 1,
    // need it to be Sunday = 0. If 1st of month = Sunday, previousSunday=1-0=1
    //int previousSunday = day - (dow-1);
    // -------------------- March ---------------------------------------
    //In march, we are DST if our previous Sunday was = to or after the 8th.
    if (_dt.tm_mon == (3-1) ) {  // in march, if previous Sunday is after the 8th, is DST
        // unless Sunday and hour < 2am
        if ( previousSunday >= 8 ) { // Sunday = 1
            // return true if day > 14 or (dow == 1 and hour >= 2)
            return ((_dt.tm_mday > 14) || ((_dt.tm_wday == 1 && _dt.tm_hour >= 2) || _dt.tm_wday > 1));
        } // end if ( previousSunday >= 8 && _dt.tm_wday > 0 )
        else
        {
            // previousSunday has to be < 8 to get here
            //return (previousSunday < 8 && (_dt.tm_wday - 1) = 0 && _dt.tm_hour >= 2)
            return false;
        } // end else
    } // end if (_dt.tm_mon == (3-1) )
    // ------------------------------- November -------------------------------
    //In november we must be before the first Sunday to be dst.
    //That means the previous Sunday must be before the 2nd.
    if (previousSunday < 1)
        // is not true for Sunday after 2am or any day after 1st Sunday any time
        return ((_dt.tm_wday == 1 && _dt.tm_hour < 2) || (_dt.tm_wday > 1));
    else
        // return false unless after first wk and dow = Sunday and hour < 2
        return (_dt.tm_mday < 8 && _dt.tm_wday == 1 && _dt.tm_hour < 2);
} // end boolean NTPtime::daylightSavingTime(unsigned long _timeStamp)

int16_t dst = -1;

//dayofWeek  Sunday is day 1
void IsDST(struct tm &t)
{
    // here the US code
    // see http://stackoverflow.com/questions/5590429/calculating-daylight-saving-time-from-only-date
    // since 2007 DST begins on second Sunday of March and ends on first Sunday of November.
    // Time change occurs at 2AM locally
    if (t.tm_mon < (3-1) || t.tm_mon > (11-1))
        dst = 0;  //January, february, and december are out.
    else if (t.tm_mon > (3-1) && t.tm_mon < (11-1))
        dst = 1;   //April to October are in
    else {
        int previousSunday = t.tm_mday - (t.tm_wday - 1);  // dow Sunday input was 1,
        if (t.tm_mon == (3-1) ) // in march, if previous Sunday is after the 8th, is DST
        	dst = (previousSunday >= 8) && ((t.tm_mday > 14) || ((t.tm_wday == 1 && t.tm_hour >= 2) || t.tm_wday > 1));
        else // November
        	dst = (previousSunday < 1)?((t.tm_wday == 1 && t.tm_hour < 2) || (t.tm_wday > 1)):
        	                            (t.tm_mday <8 && t.tm_wday == 1 && t.tm_hour < 2);
    }
    t.tm_isdst = dst;
}
/*
static bool IsDST(struct tm &t)
{
    //January, february, and december are out.
    if (t.tm_mon < (3-1) || t.tm_mon > (11-1))
        dst = 0;
    //April to October are in
    else if (t.tm_mon > (3-1) && t.tm_mon < (11-1))
        dst = 1;
    else {
        register int previousSunday = t.tm_mday - t.tm_wday;
        //In march, we are DST if our previous sunday was on or after the 8th.
        dst = (t.tm_mon == (3-1))?(previousSunday >= 8):(previousSunday <= 0);
    }
}
static void IsDST(struct tm &t) {
  dst = 0;
  if((t.tm_mon > (3-1) && t.tm_mon < (11-1)) ||
     (t.tm_mon == (11-1) && t.tm_mday < 8 && ((t.tm_mday < t.tm_wday) || (t.tm_wday == 1 && t.tm_hour < 1))) ||
     (t.tm_mon == (3-1) && t.tm_mday > 7 && t.tm_mday >= (t.tm_wday + 7) && !(t.tm_wday == 1 && t.tm_hour < 2)))
    dst = 1;
}

static void IsDST(int year, int t.tm_mon, int t.tm_mday, int t.tm_wday, int t.tm_hour)
{
   int y = tm_year + 1900 - 2000;                // Get year from RTC and subtract 2000
   // ********************* Calculate offset for Sunday *********************
   int x = (y + y/4 + 2) % 7;      // remainder will identify which day of month
                                                // is Sunday by subtracting x from the one
                                                // or two week window.  First two weeks for March
                                                // and first week for November

   // *********** Test DST: BEGINS on 2nd Sunday of March @ 2:00 AM *********
   if (t.tm_mon < (3-1) || t.tm_mon > (11-1))
       DST = 0;//January, february, and december are out.
   else if(t.tm_mon < (11-1) && t.tm_mon > (3-1))
       DST = 1;//January, february, and december are out.
   else if (t.tm_mon == (3-1))
       DST = ((t.tm_mday == (14 - x) && t.tm_hour >= 2) ||
              (t.tm_mday > (14 - x) || t.tm_mon > (3-1))); // Daylight Savings Time is TRUE (add one t.tm_hour)
   // ************* Test DST: ENDS on 1st Sunday of Nov @ 2:00 AM ************
   else if (t.tm_mon == (11-1))
       DST = !((t.tm_mday == (7 - x) && t.tm_hour >= 2) ||
               (t.tm_mday > (7 - x) || t.tm_mon > (11-1) || t.tm_mon < (3-1))); // daylight savings time is FALSE (Standard time)

//   if(DST == 1)                        // Test DST and add one hour if = 1 (TRUE)
//        t.tm_hour++;
}
*/
