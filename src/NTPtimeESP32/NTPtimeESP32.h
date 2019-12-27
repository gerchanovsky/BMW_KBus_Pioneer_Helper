/*

  NTPtime for ESP32
  This routine gets the unixtime from a NTP server and adjusts it to the time zone and the
  Middle European summer time if requested

  Author: Alex Gerchanovsky V2.0 2019-11-16
  Original Author: Andreas Spiess V1.0 2016-6-28

  Based on work from John Lassen: http://www.john-lassen.de/index.php/projects/esp-8266-arduino-ide-webconfig

*/
#pragma once

#include <Arduino.h>
#include <time.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define DEBUG_ON

/*
  int	tm_yday;
  int	tm_isdst;
#ifdef __TM_GMTOFF
  long	__TM_GMTOFF;
#endif
#ifdef __TM_ZONE
  const char *__TM_ZONE;
#endif
*/
#define SEND_INTERVAL_DEF 5       // 1 second
#define RECV_TIMEOUT_DEF  1       // 1 second
class NTPtime {
  public:
    NTPtime(const String &server = "ch.pool.ntp.org", uint32_t send_interval_sec = SEND_INTERVAL_DEF, uint32_t recv_timeout_sec = RECV_TIMEOUT_DEF);
    time_t getNTPtime();

  private:
    String NTPserver;
    unsigned long sentTime;
    unsigned long sendInterval;
    unsigned long recvTimeout;
    WiFiUDP UDPNTPClient;
};
