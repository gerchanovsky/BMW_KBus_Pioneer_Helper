#pragma once

#include <Arduino.h>
#include <time.h>

extern int16_t dst;

extern void IsDST(struct tm &t);

// other functions
extern void printDateTime(const struct tm _dt);
extern bool localTime(time_t _unixTime, struct tm *info);
extern byte DayOfWeek(int y, byte m, byte d); // 01...07, 01 = Sunday
