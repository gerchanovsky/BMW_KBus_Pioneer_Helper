/*
   This sketch shows an example of sending a reading to data.sparkfun.com once per day.

   It uses the Sparkfun testing stream so the only customizing required is the WiFi SSID and password.

   The Harringay Maker Space
   License: Apache License v2
*/
#include "src/NTPtimeESP32/NTPtimeESP32.h"


NTPtime NTPch("ch.pool.ntp.org",5);   // Choose server pool as required
const char *ssid      = "guest";               // Set you WiFi SSID
const char *password  = "rubezhnoye";               // Set you WiFi password

#define TIMEZONE  "PST8PDT,M3.2.0,M11.1.0" // "US/Pacific-New"
#define NTPSERVER "ch.pool.ntp.org"

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booted");
  Serial.println("Connecting to Wi-Fi");

  WiFi.mode(WIFI_STA);
  WiFi.begin (ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("WiFi connected");
    //Update RTC
    Serial.println("Downloading time from NTP server now... ");
    //configTzTime(TIMEZONE, NTPSERVER);
    configTime(-8*3600, 0/*3600*/, NTPSERVER);
}

void loop() {

  // first parameter: Time zone in floating point (for India); second parameter: 1 for European summer time; 2 for US daylight saving time (contributed by viewwer, not tested by me)
  
  time_t t = NTPch.getNTPtime();
  
  // check dateTime.valid before using the returned time
  // Use "setSendInterval" or "setRecvTimeout" if required
  if(t != 0){
    tm dateTime = NTPch.localTime(t, -8.0, NTPtime::t_daylightsaving::DST);
    NTPtime::printDateTime(dateTime);

    struct tm timeInfo;
    if (getLocalTime(&timeInfo, 5000)) {
      Serial.println(&timeInfo, "Done: %A, %B %d %Y %H:%M:%S");
    }
  }
}
