/*
NTP
This routine gets the unixtime from a NTP server and adjusts it to the time zone and the
Middle European summer time if requested

Author: Andreas Spiess V1.0 2016-5-28

Based on work from John Lassen: http://www.john-lassen.de/index.php/projects/esp-8266-arduino-ide-webconfig

*/

#include "NTPTimeESP32.h"

#define SEC_TO_MS         1000

// NTPserver is the name of the NTPserver
NTPtime::NTPtime(const String &server, uint32_t send_interval_s, uint32_t recv_timeout_s) :
    NTPserver(server),
    sentTime(0),
    sendInterval(send_interval_s * SEC_TO_MS),
    recvTimeout(recv_timeout_s * SEC_TO_MS)
{}

time_t NTPtime::getNTPtime() {
    static byte _packetBuffer[48];
    if (sentTime != 0) {
        int cb = UDPNTPClient.parsePacket();
        if (cb != 0) {
#ifdef DEBUG_ON
            Serial.print("NTP packet received, length=");
            Serial.println(cb);
#endif
            sentTime = 0;
            UDPNTPClient.read(_packetBuffer, sizeof(_packetBuffer)); // read the packet into the buffer
            uint32_t secsSince1900 = (((unsigned)_packetBuffer[40] << 24) | (_packetBuffer[41] << 16) | (_packetBuffer[42] << 8) | _packetBuffer[43]);
            return (time_t)(secsSince1900 - 2208988800UL);//seventyYears;
        }
        if ((millis() - sentTime) > recvTimeout) {
            sentTime = 0;
        }
    } else if ((sentTime == 0) || ((millis() - sentTime) >= sendInterval)) {
        UDPNTPClient.begin(1337); // Port for NTP receive

        IPAddress _timeServerIP;
        WiFi.hostByName(NTPserver.c_str(), _timeServerIP);
#ifdef DEBUG_ON
        Serial.print("Sending NTP packet to ");
        Serial.println(_timeServerIP);
#endif
        memset(_packetBuffer, 0, sizeof(_packetBuffer));
        _packetBuffer[ 0] = 0b11100011; // LI, Version, Mode
        _packetBuffer[ 1] = 0;          // Stratum, or type of clock
        _packetBuffer[ 2] = 6;          // Polling Interval
        _packetBuffer[ 3] = 0xEC;       // Peer Clock Precision
        _packetBuffer[12] = 49;
        _packetBuffer[13] = 0x4E;
        _packetBuffer[14] = 49;
        _packetBuffer[15] = 52;
//        UDPNTPClient.beginPacket(_NTPserver.c_str(), 123);
        UDPNTPClient.beginPacket(_timeServerIP, 123);
        UDPNTPClient.write(_packetBuffer, sizeof(_packetBuffer));
        UDPNTPClient.endPacket();
        sentTime = millis();
    }
    return 0;
}

/*
https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-time.c

extern "C" bool getLocalTime(struct struct tm * info, uint32_t ms = 5000);
extern "C" void configTime(long gmtOffset_sec, int daylightOffset_sec,
        const char* server1, const char* server2 = nullptr, const char* server3 = nullptr);
extern "C" void configTzTime(const char* tz,
        const char* server1, const char* server2 = nullptr, const char* server3 = nullptr);

Dear all,
I have an issue that drives me crazy, hope somebody can help me. I have a running project with an ESP32, where the ESP32 is in deep sleep most of the time and wakes up every minute or so to do something. Every 10th wakeup, or after around 10 minutes, it is connecting to WIFI and passing some data to a server.

Now, when connected to WIFI, I want to readjust my RTC by NTC. After 10 mintes my time drifted for some seconds, no big deal, as I have a WIFI connection I just can reajust. But, the time is never updated. It updates only after a power on reset, but never after a deep sleep. Any idea? Below the code which is updating the clock. It always says "Done", but actually the rtc is never adjused.

Thank you for your help.
Christian

#define TIMEZONE  "PST8PDT,M3.2.0,M11.1.0" // "US/Pacific-New"
#define NTPSERVER "ch.pool.ntp.org"
bool updateServer(void) {
  if (connectWifi()) {

    //Update RTC
    Serial.println("Downloading time from NTP server now... ");
    configTzTime(TIMEZONE, NTPSERVER);

    struct struct tm timeInfo;
    if (getLocalTime(&timeInfo, 10000)) {
      Serial.println(&timeInfo, "Done: %A, %B %d %Y %H:%M:%S");
    }
    else Serial.println("Error. Unable to download time from NTP server.");

    //Finish
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return true;

  }
  else Serial.println("Error, cannot connect to WIFI. Sorry.");

  return false;
}
*/
