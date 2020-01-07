//KMB-IKE gauge:
/*
C:\BMW\IBus\example_dump_1.bin
80 07 0C 08 04 92 15
C:\BMW\IBus\example_dump_1.bin-
80 07 0C 08 46 DD 18

80 07 0C 0A 00 00 81
example_dump_1.txt
80 07 0C 08 46 DD 18
80 04 A0 24
example_dump_1.txt-

80 07 0C 0A 0B 40 CA
80 07 0C 0A 01 40 C0
80 07 0C 08 46 DD 18
80 07 0C 08 04 92 15
80 04 A0 24

Gong
80 05 0C 11 98  DIA IKE Set IO status Data="11"    [ ]

80 04 A0 24     IKE DIA Diagnostic command acknowledged Data=""    []

80 04 9F 1B End test
80 04 A0 24 Status reset


Self test
0x80 0x04 0x30 0xB4   4 seconds
Stop test
0x80 0x04 0x9F 0x1B   reset analog
*/
#include <Arduino.h>
#include "credentials.h" // declare WIFI_SSID and WIFI_PASSWORD here

#define countof(a) (sizeof((a))/sizeof((a)[0]))
//ESP32
//     SCK  SDI  SDO  CS
//     SCLK MISO MOSI SS
//FSPI  6    7   8    11  //flash (can use the same data lines but different SS)
//HSPI 14   12   13   15
//VSPI 25   26   27   32
//     18?  19?  23?  5?
//  MCP4151-104 ESP32 GPIO
//pin 1         5  CS
//pin 2         18 SCK
//pin 3         23 SDI/SDO
//pin 4 GND
//pin 5 GND
//pin 6 Wiper
//pin 7 N/C
//pin 8 3.3v
#define PIN_WR_CS  	15
#define PIN_WR_SCK 	14
#define PIN_WR_SDO 	13

#define PIN_WR_SHIFT     5   //Wired remote shift

// discreet control wires
#define PIN_BACKCAM     18   // HIGH - +12V to turn on Backup camera
#define PIN_ILLUM       19   // HIGH - +12V to dim LCD backlight
#define PIN_MUTE        23   // LOW  - ground to attenuate sound
//#define PIN_IR        26   // ESP32 RMT IR Remote controller

//UART	RX IO	TX IO
//UART0	GPIO3	GPIO1
//UART1	GPIO9	GPIO10
//UART2	GPIO16	GPIO17
#define PIN_IBUS_ENABLE 33   // Melexix TH3122.3 Enable pin
#define PIN_IBUS_SENSTA 35   // Melexix TH3122.3 SEN/STA pin
#define PIN_IBUS_TX     17   // Melexix TH3122.3 TX pin Serial2
#define PIN_IBUS_RX     16   // Melexix TH3122.3 RX pin Serial2

#define PIN_OLED_SDA    21
#define PIN_OLED_SCL    22

#define PIN_BUTTON      11
#define PIN_BUZZER       4
#define PIN_LED          2 //LED_BUILTIN

#define PIN_JQ6500_RX   25
#define PIN_JQ6500_TX   27
#define PIN_JQ6500_BUSY 32

#define I2C_ADDR_DS3231  0x68
#define I2C_ADDR_SSD1306 0x3C

typedef enum : uint8_t {
  TAG_MANUAL,
  TAG_WIRED_RELEASE,
  TAG_WIFI_TIMEOUT,
  TAG_BEEP,
  TAG_MUTE_MP3,
  TAG_MP3_INIT,
  TAG_IKE_REVERSE,
  TAG_TURN_MIND,
  TAG_TURN_LCM,
  TAG_DIMMER_RLS,
  TAG_DIMMER_LCM,
  TAG_DBL_PRESS_RT,
  TAG_DBL_PRESS_TALK,
  TAG_LONG_PRESS_TALK,
  TAG_IKE_SWEEP,
  TAG_IKE_HELLO,
  TAG_CLOSE_HOOD
} TAG_ACTION;

#include "src/Scheduler/Scheduler.h"
static void output_pin(uint8_t pin, uint8_t val)
{
  digitalWrite(pin, val);
  pinMode(pin, OUTPUT);
}

#include "esp32-hal.h"

//////////////////////////////////////////////////////////////////////////////////////////////
// DEBUG SERIAL
// If using processor board with more than one HardwareSerial port, you don't need to use SoftwareSerial or AltSoftSerial for
// DEBUG, so just define which HardwareSerial port you plan to use.
//#define BT

#ifndef BT
#define debugSerial Serial
#define DEBUG_OK true
#else
#define debugSerial SerialBT
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
#define DEBUG_OK SerialBT.hasClient() //http://makerangst.com/ios-ble-part-2-esp32
#endif
namespace Debug {
static void setup()
{
  // Tell library which serial port is used for DEBUG. Comment out line if debug disabled in IbusGlobals.h
  debugSerial.begin(115200); // Set debug Baud rate. Keeps speed high if sending lots of debug messages over serial. 38400 is about as fast as SoftwareSerial can handle reliably.
#if defined(BT)
  Serial.println("Bluetooth");
  SerialBT.begin("IBus"); //Bluetooth device name
#endif
  debugSerial.println("*** BMW I-Bus infotainment assistant ***");
}
}//namespace Debug
////////////////////////////////////////////////////////////////////////////////////

static void beep(int duration = 50)
{
  output_pin(PIN_BUZZER, HIGH);
  Scheduler::add(TAG_BEEP, duration, [] (byte v) { output_pin(PIN_BUZZER, LOW); });
}

class pinClass {
    byte pin;
    int inverse;
  public:
    uint32_t on;
    pinClass(const int _pin, const int _inverse = 0) : pin(_pin), inverse(_inverse), on(0) {
      output_pin(pin, inverse ^ LOW);
    }
    void enable(const byte tag) {
      if (on==0) {
        digitalWrite(pin, inverse ^ HIGH);
        onEnable(tag);
      }
      on |= (1<<tag);
    }
    void disable(const byte tag) {
      register uint32_t on_old = on;
      on &= ~(1<<tag);
      if (on==0 && on_old!=0) {
        digitalWrite(pin, inverse ^ LOW);
        onDisable(tag);
      }
    }
    bool isEnabled(const byte tag) {
      return (on&(1<<tag))!=0;
    }
    virtual void onEnable(const byte tag) const {}
    virtual void onDisable(const byte tag) const {}
};

class audioMuteClass : public pinClass {
public:
  audioMuteClass() : pinClass(PIN_MUTE, 1) { }
  virtual void onEnable(const byte tag) const  {debugSerial.printf("\r\nMute   (tag=%d)\r\n", tag);}
  virtual void onDisable(const byte tag) const {debugSerial.printf("\r\nUnmute (tag=%d)\r\n", tag);}
};
audioMuteClass audioMute;

class backupCamClass : public pinClass {
public:
  backupCamClass() : pinClass(PIN_BACKCAM, 0) { }
  virtual void onEnable(const byte tag) const  {debugSerial.printf("\r\nBackup cam ON  (tag=%d)\r\n", tag);}
  virtual void onDisable(const byte tag) const {debugSerial.printf("\r\nBackup cam OFF (tag=%d)\r\n", tag);}
};
backupCamClass backupCam;

class dimmerClass : public pinClass {
public:
  dimmerClass() : pinClass(PIN_ILLUM, 0) { }
  virtual void onEnable(const byte tag) const  {debugSerial.printf("\r\nDim headunit backlight   (tag=%d)\r\n", tag);}//beep(200);
  virtual void onDisable(const byte tag) const {debugSerial.printf("\r\nUndim headunit backlight (tag=%d)\r\n", tag);}//beep(200);
};
dimmerClass dimmer;

//////////////////////////////////////////////////////////////////////////////////////////////
#include "src/JQ6500/JQ6500.h"
#include "src/IBus/RingBuffer.h"

namespace Speaker {
typedef enum : uint8_t {
  EMPTY,
  TWILIGHT = 1,//0
  DARKNESS,    //1
  RAIN,        //2
  TUNNEL,      //3
  GARAGE,      //4
  BRIGHT,      //5
  //
  CHIME,       //6
  TURN_MIND,   //7
  LETS_GO,     //8
  HELLO,       //9
  GOOD_BYE,    //10
  CLOSE_HOOD   //11
} AUDIO_TAG;

static JQ6500 mp3(Serial1);
static RingBuffer mp3_queue(5);
static volatile bool busy = false;
static AUDIO_TAG mp3_last = EMPTY;

static IRAM_ATTR void busyISR()
{
  busy = (LOW!=digitalRead(PIN_JQ6500_BUSY));
  if (busy)
    audioMute.enable(TAG_MUTE_MP3);
  else
    audioMute.disable(TAG_MUTE_MP3);
}

static inline void setup()
{
  mp3.begin(PIN_JQ6500_RX, PIN_JQ6500_TX);
  mp3.setVolume(30);
  mp3.setSource(JQ6500::SRC_FLASH);
  //mp3.setEqualizer(EQ_NORMAL);
  mp3.setLoopMode(JQ6500::LOOP_ONE_STOP);
  //static int mp3_count = 0;
  //mp3_count = mp3.countFiles();
  pinMode(PIN_JQ6500_BUSY, INPUT);
  busy = (LOW!=digitalRead(PIN_JQ6500_BUSY));
  attachInterrupt(digitalPinToInterrupt(PIN_JQ6500_BUSY), busyISR, CHANGE);
  //Serial.printf("Volume (0-30): %d\r\n", mp3.getVolume());
  //Serial.printf("Number of files: %d\r\n", mp3_count);
  //Serial.printf("Current file #: %d\r\n", mp3.currentFile());
  mp3_last = EMPTY;
}
static inline void loop()
{
  if (busy || mp3_queue.available()<=0)
    return;
  mp3_last = (AUDIO_TAG)mp3_queue.pop();
  debugSerial.printf("\r\nPlay audio #%d\r\n", mp3_last);
  mp3.play(mp3_last);
}
inline void play(const AUDIO_TAG file_idx)
{
  mp3_queue.push(file_idx);
}
inline void replay()
{
  if (mp3_last!=EMPTY) {
    debugSerial.printf("\r\nReplay audio #%d\r\n", mp3_last);
    play(mp3_last);
  }
}
} // namespace Speaker
//////////////////////////////////////////////////////////////////////////////////////////////
static void oled_show_rtc();
namespace BMW {
static void set_ike_time();
}
#include "src/DST/DST.h"
#include "src/DS3231/DS3231.h"
#include "src/NTPtimeESP32/NTPtimeESP32.h"
static DS3231 ds3231;
#include <WiFiType.h>
// NTP
// Send the ntp packet (see https://tf.nist.gov/tf-cgi/servers.cgi for other ip addresses).
#define TIME_ZONE_DESC  "PST8PDT,M3.2.0,M11.1.0"      // "US/Pacific-New"
#define NTP_SERVER "north-america.pool.ntp.org" //"ch.pool.ntp.org", "pool.ntp.org", "ch.pool.ntp.org"
#define TIME_ZONE -8 //SET FOR US PST

namespace Wifi {

static NTPtime NTPch(NTP_SERVER);   // Choose server pool as required
static bool time_is_set = false;
static inline void setup()
{
  ds3231.begin();
  oled_show_rtc();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  configTzTime(TIME_ZONE_DESC, NTP_SERVER);
  Scheduler::add(TAG_WIFI_TIMEOUT, 30000, [] (byte) {
    time_is_set = true;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  });

  //configTime(TIME_ZONE*3600, 0/*3600*/, NTPSERVER);
/*
  struct tm timeInfo;
  if (getLocalTime(&timeInfo, 5000)) {
    Serial.println(&timeInfo, "Done: %A, %B %d %Y %H:%M:%S");
  }
*/
/*
  while (WiFi.status() != WL_CONNECTED) {
    debugSerial.print(".");
    delay(500);
  }
*/
}
static inline void loop()
{
  if (time_is_set || (WiFi.status() != WL_CONNECTED))
    return;
  time_t t = NTPch.getNTPtime();
  if (t == 0)
    return;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  time_is_set = true;
  t += TIME_ZONE * 3600; // adjust timezone
  struct tm dt;
  localTime(t, &dt);
  ds3231.set(dt);
  IsDST(dt);
  dt.tm_isdst = dst;
  dt.tm_hour += dst;
  debugSerial.print("Set DS3231 RTC");
  BMW::set_ike_time();
  printDateTime(dt);
}
}//namespace Wifi {
//////////////////////////////////////////////////////////////////////////////////////////////
//https://github.com/ThingPulse/esp8266-oled-ssd1306
//#include <SSD1306.h>
//static SSD1306Wire display(0x3c, PIN_OLED_SDA, PIN_OLED_SCL, GEOMETRY_128_32);
//#include <SH1106.h>
//static SH1106Wire display(0x3c, PIN_OLED_SDA, PIN_OLED_SCL);

#include "src/Adafruit_GFX_Library/Adafruit_GFX.h"
#include "src/Adafruit_SSD1306_SH1106/Adafruit_SSD1306_SH1106.h"
namespace Display {

#define SCREEN_WIDTH  SSD1306_LCDWIDTH  // OLED display width, in pixels
#define SCREEN_HEIGHT SSD1306_LCDHEIGHT // OLED display height, in pixels
Adafruit_SSD1306 scr(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1/*OLED_RESET*/);

static inline void setup()
{
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!scr.begin(SSD1306_SWITCHCAPVCC, I2C_ADDR_SSD1306)) { // Address 0x3D for 128x64
    Serial.println("SSD1306 allocation failed");
  }
  scr.setRotation(2);
  scr.setTextWrap(false);
  scr.setTextSize(1);
  scr.clearDisplay();
  //scr.setFont(ArialMT_Plain_10);
  scr.setTextColor(WHITE, BLACK); // 'inverted' text
  scr.print("BMW E46 I-Bus monitor");
}

#define CHAR_W    6
#define CHAR_H    8
#define SCREEN_W  (SCREEN_WIDTH/CHAR_W)
#define SCREEN_H  (SCREEN_HEIGHT/CHAR_H)

static void drawText(int left, int top, int w, bool dbg, const char *format, ...)
{
  left *= CHAR_W;
  top *=  CHAR_H;
  w *= CHAR_W;
  //scr.writeFillRect(0,0,SCREEN_WIDTH,(CHAR_H+PAD)*2,BLACK);
  scr.fillRect(left, top, w, CHAR_H, BLACK),
  scr.setCursor(left, top);

  char loc_buf[100];
  int len;
  va_list arg;
  va_start(arg, format);
  len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
  va_end(arg);
  if (len < 0 && len >= sizeof(loc_buf)) {
    scr.print("**over**");
  } else {
    //scr.write((uint8_t*)&loc_buf, len);
    scr.print(loc_buf);
    if (dbg) {
      debugSerial.println();
      debugSerial.println(loc_buf);
    }
  }
  //scr.print(str);
  //scr.drawRect(0, 0, scr.getWidth()-1, scr.getHeight()-1);
  scr.display();
}

} //namespace Display

static const char txt_gears[] = "xPRND432";
static void oled_show_gear(int gear_num)
{
#define PAD   1
#define WIDTH ((CHAR_W+1)*7)
#define HEIGHT (CHAR_H+1)
#define LEFT (SCREEN_WIDTH-WIDTH)
#define TOP  (SCREEN_HEIGHT-HEIGHT)
  Display::scr.fillRect(LEFT, TOP, WIDTH, HEIGHT, BLACK);
  for (int i=1;i<sizeof(txt_gears);i++) {
    Display::scr.setCursor(LEFT+PAD+(i-1)*(CHAR_W+PAD),TOP+PAD);
    Display::scr.write(txt_gears[i]);
  }
  Display::scr.fillRect(LEFT+gear_num*(CHAR_W+1), TOP, (CHAR_W+1), HEIGHT, INVERSE),
  Display::scr.display();
#undef PAD
#undef WIDTH
#undef HEIGHT
#undef LEFT
#undef TOP
}

static void oled_show_rtc()
{
  struct tm dt;
  ds3231.get(dt);
  if (dst<0)
    IsDST(dt);
  //if (getLocalTime(&dt, 10000))
  //  Serial.println(&dt, "%A, %B %d %Y %H:%M:%S");
  //size_t strftime(char *__restrict _s, size_t _maxsize, const char *__restrict _fmt, const struct tm *__restrict _t);
  Display::drawText(0, 4, SCREEN_WIDTH, false, "%2d:%02d:%02d %2d/%02d/%4d",
    dt.tm_hour, dt.tm_min, dt.tm_sec,
    //(h12)?(PM?"PM":"AM"):"",
    dt.tm_mon+1,
    dt.tm_mday,
    dt.tm_year+1900);
}
#define oled_show_msg(...)       Display::drawText(0, 0, 21, true, __VA_ARGS__)
//#define oled_show_lcm_light(...) Display::drawText(0, 2, 8, true, __VA_ARGS__)
#define oled_show_rls(...)       Display::drawText(8, 2, SCREEN_W-8, false, __VA_ARGS__)
#define oled_show_time(str)      Display::drawText(0, 3, 8, false, str)
#define oled_show_date(str)      Display::drawText(8, 3, 10, false, str)
#define oled_show_speed(mph)     Display::drawText(0, SCREEN_H-1, 7, false, "%d MPH", mph)

static inline void initcheck_AC()
{
//  enable AC compressor using IHKA
//  if (ds3231.getTemperature()>25.0) {// (in C)
//    msg=83  Air conditioning compressor status
//  }
}

////////////////////////////////////////////////////////////////////////////////////
// IR remote
#ifdef PIN_IR
#include "src/PioneerIrCommands/PioneerIrCommands.h"
#endif
////////////////////////////////////////////////////////////////////////////////////
// 3.5mm Pioneer Wired Remote
static int current_speed_mph = -1;
//MFL remote codes
typedef enum : byte {
  MFL_PRESS,
  MFL_HOLD,
  MFL_RELEASE,
  MFL_UNKNOWN
} MFL_PRESS_TYPE;

static const char *press2str(register const byte t)
{
  return (t==MFL_PRESS)?"Press":
         (t==MFL_HOLD)?"Hold":
         (t==MFL_RELEASE)?"Release":"Unknown";
}

#include <SPI.h>

namespace PioneerRemote {

static SPIClass wr_dp_spi(HSPI);
static const SPISettings SPI_SETTING(10000000, MSBFIRST, SPI_MODE0);

typedef enum : byte {
  SRC = 0,    //shift=Tel.menu/shift+Hold=BT menu 2sec off
  MUTE,
  DISP,   //Camera
  NEXT,
  PREV,
  VOL_UP,
  VOL_DN,
  ENTER,  //Select
  BAND,
  PIO_SHIFT,// <= SHIFT
  TEL_MENU = PIO_SHIFT,//2sec Hold=BT menu
  TEL_ANSWER,
  TEL_HANGUP,
  FOLDER_UP,
  FOLDER_DN,
  UNK2,
  UNK3,
  UNK4,
  VOICE_CTRL,
} PIONEER_BTN;

static const char* const remote_str[] = {
  "Source",
  "Mute",
  "Display",   //Camera
  "Next",
  "Prev",
  "Volume(+)",
  "Volume(-)",
  "Enter/Select",
  "Band",
  //shift
  "Tel.Menu",
  "Tel.Answer",
  "Tel.Hangup",
  "Folder(+)",
  "Folder(-)",
  "UNK2",
  "UNK3",
  "UNK4",
  "VoiceCtrl"
};
static const byte btn_val[] = {
  3,   //1.22K
  9,   //4.18K  10
  15,  //6.05K  15
  21,  //8.27K  21
  30,  //11.64K 29
  42,  //16.12K 40
  63,  //23.17K 57
  90,  //35.45K 86
  159, //59.29K 143
};

#define DT_SHUTDOWN   0x40FD
#define DT_REACTIVATE 0x40FF

static void wr_dp_write(uint16_t value)
{
  digitalWrite(PIN_WR_CS, LOW);
  wr_dp_spi.beginTransaction(SPI_SETTING);
  wr_dp_spi.transfer16(value);
  if (value != DT_SHUTDOWN)
    wr_dp_spi.transfer16(DT_REACTIVATE);
  wr_dp_spi.endTransaction();
  digitalWrite(PIN_WR_CS, HIGH);
}

static void release_()
{
  Scheduler::del(TAG_WIRED_RELEASE);
  wr_dp_write(DT_SHUTDOWN);
  digitalWrite(PIN_WR_SHIFT, HIGH);
}
static void release()
{
  release_();
  oled_show_msg("release");
}

#define PRESS_TIME_KEEP    0
#define PRESS_TIME_SHORT  70
#define PRESS_TIME_LONG 2500
#define PRESS_TIME_SIRI 1000

#define RT_DBL_PRESS_TIME   600
#define TALK_DBL_PRESS_TIME 500

static void press(const PIONEER_BTN btn, unsigned int t = 0)
{
  bool shift = (btn>=PIO_SHIFT);
  oled_show_msg("press:%s (%dms)", remote_str[btn], t);
  digitalWrite(PIN_WR_SHIFT, shift?LOW:HIGH);
  wr_dp_write((uint16_t)btn_val[btn % PIO_SHIFT]);
  if (t>0)
    Scheduler::add(TAG_WIRED_RELEASE, t, [] (byte) {release();});
  else
    Scheduler::del(TAG_WIRED_RELEASE);
}

static void pressRT(const byte data1)
{
  debugSerial.printf("R/T pressed (data=0x%02X)", data1);
  bool off = (data1==0);
  if (Scheduler::del(TAG_DBL_PRESS_RT)) {
    press(DISP, PRESS_TIME_SHORT);
  } else {
    Scheduler::add(TAG_DBL_PRESS_RT, RT_DBL_PRESS_TIME, [] (byte) {
      press(TEL_MENU, PRESS_TIME_SHORT);
    });
  }
}

static void pressTalk(const MFL_PRESS_TYPE state)
{
  static bool hold_talk = false;
  debugSerial.printf("Talk button %s", press2str(state));
  switch (state) {
  case MFL_PRESS:
    hold_talk = false;
    //
    Scheduler::add(TAG_LONG_PRESS_TALK, 500, [] (byte) {
      hold_talk = true;
      Scheduler::del(TAG_DBL_PRESS_TALK);
      press(VOICE_CTRL);
    });
    //
    break;
  case MFL_HOLD: // never happen :-(
    Scheduler::del(TAG_LONG_PRESS_TALK);
    hold_talk = true;
    Scheduler::del(TAG_DBL_PRESS_TALK);
    press(VOICE_CTRL);
    break;
  case MFL_RELEASE:
  default:
    Scheduler::del(TAG_LONG_PRESS_TALK);
    if (hold_talk) {
      hold_talk = false;
      release();//Voice control
    } else if (/*current_speed_mph<10 || */Scheduler::del(TAG_DBL_PRESS_TALK)) {
      press(DISP, PRESS_TIME_LONG); // escape from camera
    } else {
      Scheduler::add(TAG_DBL_PRESS_TALK, TALK_DBL_PRESS_TIME, [] (byte) {
        press(MUTE, PRESS_TIME_SHORT);
      });
    }
  }
}

static void pressNav(const MFL_PRESS_TYPE state, bool next)
{
  debugSerial.printf("Nаvigation button %s %s", next?"Next":"Prev", press2str(state));
  switch (state) {
  case MFL_PRESS:  PioneerRemote::press(next?NEXT:PREV);break;
  case MFL_HOLD:   /*debugSerial.print("Hold MFL");*/break;
  case MFL_RELEASE:
  default:         PioneerRemote::release();
  }
}          

static inline void setup()
{
  output_pin(PIN_WR_CS, HIGH);
  output_pin(PIN_WR_SHIFT, HIGH);
  wr_dp_spi.begin(PIN_WR_SCK, -1, PIN_WR_SDO, PIN_WR_CS);
  release_();
#ifdef PIN_IR
  ir_remote.begin();
#endif
}

} // namespace PioneerRemote

////////////////////////////////////////////////////////////////////////////////////
#include "src/IBus/IBus.h"
namespace BMW {

static IBus ibus(Serial2, PIN_IBUS_SENSTA); // Create an instance of the IBus library called ibus.
// LCM dimmer message

// Ambient light sensor switchover values (0x3F-0xFF)
#define LSZ_DARK   100
#define LSZ_BRIGHT 130

// Source      : 0xE8 (RLS)
// Destination : 0xD0 (LCM)
// Command     : 0x59 (Light control)
// Source, length, destination, command, and checksum removed from strings below

// D2 - Reason
typedef enum {
  TWILIGHT, // Bit0
  DARKNESS, // Bit1
  RAIN,     // Bit2
  TUNNEL,   // Bit3
  GARAGE,   // Bit4
  RLS_REASON_LAST
} RLS_REASON;

static inline void rls_lightsensor(byte b, byte reason)
{
  const char *reason_str = "";
  bool illum_new = false;
  bool light_on = (b&1);
  static const char* const rls_reason_str[] = {
      "Twilight",//1<<0
      "Darkness",//1<<1
      "Rain",    //1<<2
      "Tunnel",  //1<<3
      "Garage",  //1<<4
      ""
  };
  int reason_idx = RLS_REASON_LAST;
  if (light_on) {
    for (reason_idx=0;reason_idx<RLS_REASON_LAST && ((reason&(1<<reason_idx))==0);reason_idx++);
    illum_new = (reason&((1<<DARKNESS)|(1<<GARAGE)|(1<<TUNNEL)));
    reason_str = rls_reason_str[reason_idx];
  }
  static byte old_reason_idx = 0;
  b >>= 4;
  oled_show_rls("%02X:%s", b, reason_str);
  debugSerial.printf("Light sensor %02X:%s%s ", b, reason_str, (old_reason_idx != reason_idx)?"(*)":"");
  if (old_reason_idx != reason_idx) {
    old_reason_idx = reason_idx;
//    Speaker::play((Speaker::AUDIO_TAG)(Speaker::TWILIGHT+reason_idx));
  }
  if (illum_new)
    dimmer.enable(TAG_DIMMER_RLS);
  else
    dimmer.disable(TAG_DIMMER_RLS);
}

static inline void lcm_lightsensor(const byte dimmer_value, const byte sensor_value)
{
  //debugSerial.printf("LCM=%d/%d ", dimmer_value, sensor_value);
/*
  if (sensor_value >= LSZ_BRIGHT)
    dimmer.disable(TAG_DIMMER_LCM);
  else if (sensor_value <= LSZ_DARK)
    dimmer.enable(TAG_DIMMER_LCM);
*/
}
////////////////////////////////////////////////////////////////////////////////////
// Gears and speed
typedef enum : byte {
  GEAR_UNK,
  GEAR_P,//0xB0,0x00
  GEAR_R,//0x10
  GEAR_N,//0x70
  GEAR_D,//0x80
  GEAR_4,//0xC0
  GEAR_3,//0xD0
  GEAR_2,//0x40
} GEAR;

static int8_t hood_open = 0;
static GEAR gear  = GEAR_UNK;
//http://www.crypton.co.za/CDE/Abbreviations%20of%20Mechanical%20Terminology.pdf
//KL15 So called for battery voltage turned OFF during park time of the vehicle
//KL30 So called for battery voltage always present
//KL58 So called for the battery voltage to the instrument cluster
typedef enum : byte {
  IGN_OFF   = 0,                    //OFF
  IGN_ACC   = (1<<0),               //ACC  :KL_R
  IGN_ON    = (1<<1)|(1<<0),        //ON   :KL_15
  IGN_START = (1<<2)|(1<<1)|(1<<0), //START:KL_50 starter
  IGN_MASK  = 7,
} IGN_TYPES;

static bool skip_RT = true;
static byte ignition = IGN_OFF;

#define BACKCAM_SWITCH_TIME 100
typedef enum : byte {
  TRIGGER_STOP       = 0,
  TRIGGER_ENABLE     = (1<<0),
  TRIGGER_DOOR_OPEN  = (1<<1),
  TRIGGER_DOOR_CLOSE = (1<<2),
  TRIGGER_START      = (1<<3),
  TRIGGER_LETSGO  = (TRIGGER_ENABLE/*|TRIGGER_DOOR_OPEN*/|TRIGGER_DOOR_CLOSE|TRIGGER_START),
  TRIGGER_GOODBYE = (TRIGGER_ENABLE/*|TRIGGER_DOOR_OPEN*/|TRIGGER_DOOR_CLOSE|TRIGGER_START),
  TRIGGER_HELLO   = (TRIGGER_ENABLE/*|TRIGGER_DOOR_OPEN*/|TRIGGER_DOOR_CLOSE),
} TRIGGER_FLAGS;
static byte play_letsgo  = TRIGGER_ENABLE;
static byte play_hello   = TRIGGER_ENABLE;
static byte play_goodbye = TRIGGER_ENABLE;

static bool check_hood_msg = true;
static inline void gear_switch(byte b)
{
  static const byte xlat[16] = {
    GEAR_P,  //0x0  0000
    GEAR_R,  //0x1  0001
    GEAR_UNK,//0x2  0010
    GEAR_UNK,//0x3  0011
    GEAR_2,  //0x4  0100
    GEAR_UNK,//0x5  0101
    GEAR_UNK,//0x6  0110
    GEAR_N,  //0x7  0111
    GEAR_D,  //0x8  1000
    GEAR_UNK,//0x9  1001
    GEAR_UNK,//0xA  1010
    GEAR_P,  //0xB  1011
    GEAR_4,  //0xC  1100
    GEAR_3,  //0xD  1101
    GEAR_UNK,//0xE  1110
    GEAR_UNK //0xF  1111
  };
  GEAR gear_new = (GEAR)xlat[(b>>4)&0xF];
  oled_show_gear(gear_new-GEAR_P);
  debugSerial.printf("Status: Gear=%02X(%c) ", b, txt_gears[gear_new]);
  if (gear == gear_new)
    return;
  switch (gear_new) {
  case GEAR_R:
    if (check_hood_msg && hood_open) {
//ds2 car contour
//:80 05 0C 11 98 Gong
//:80 09 0C 09 00 01 00 00 8D  car contour
//:80 09 0C 09 20 00 00 00 AC  brake
//:80 09 0C 09 20 01 00 00 AD  car contour & brake lamps
//ds all of diag end()
//:80 04 9F 1B : diag end
//:80 04 A0 24 : reply from IKE=OK
      Speaker::play(Speaker::CLOSE_HOOD);
      check_hood_msg = false;
      ibus.write_ds2((const byte[]){ 0x80, 0x09, 0x0C, 0x09, 0x20, 0x01, 0x00, 0x00, 0xAD }); // car contour & brake lamps
      Scheduler::add(TAG_CLOSE_HOOD, 5000,
          [] (byte) {
            ibus.write_ds2((const byte[]){ 0x80, 0x04, 0x9F, 0x1B }); // Reset diagnostics
      } );
      break;
    }
    if (!Scheduler::find(TAG_IKE_REVERSE))
      Scheduler::add(TAG_IKE_REVERSE, BACKCAM_SWITCH_TIME,
          [] (byte) {
            output_pin(PIN_BUZZER, LOW);
            audioMute.enable(TAG_IKE_REVERSE);
            backupCam.enable(TAG_IKE_REVERSE);
            ibus.write_kbus((const byte[]){ 0x3F, 0x05, 0x00, 0x0C, 0x4E, 0x01 });//0x79 // Turn on clown nose for 3 seconds
      } );
    break;
  case GEAR_P:
    check_hood_msg = true;
//    break;
  default:
    Scheduler::del(TAG_IKE_REVERSE);
    audioMute.disable(TAG_IKE_REVERSE);
    if (backupCam.on && gear_new < GEAR_R)
      backupCam.disable(TAG_IKE_REVERSE);
    if ((gear_new == GEAR_D) && ((play_letsgo&TRIGGER_LETSGO)==TRIGGER_LETSGO)) {
      Speaker::play(Speaker::LETS_GO);
      play_letsgo = TRIGGER_STOP;
    }
  }
  gear = gear_new;
}

static inline void update_speed(int kmh, int rpm100)
{
  int mph = (int)(0.62137119*kmh);
  int rpm = rpm100*100;
  debugSerial.printf("Speed %dmph/%dkmh RPM=%d ", mph, kmh, rpm);

  if (current_speed_mph != mph) {
    current_speed_mph = mph;
    oled_show_speed(current_speed_mph);
  }
  if (backupCam.on && (current_speed_mph > 7) && (gear > GEAR_R)) {
    backupCam.disable(TAG_IKE_REVERSE);
  }
}

static inline void needle_sweep()//indicator celebration
{
  //static const byte ds2_ike_diag[] = {0x80, 0x04, 0x9E};
  //ibus.write_ds2(ds2_ike_diag);
  //:80 04 30 B4
  //:80 04 A0 24 ack
  //static const byte ds2_gauges_test[] = {0x80, 0x04, 0x30, 0xB4};
  debugSerial.print("\r\n*SWEEP GAUGES*\r\n");
  ibus.write_ds2((const byte[]){0x80, 0x04, 0x30, 0xB4});//// Sweep KMB/IKE gauges
  ibus.setSendCallback([] () {
    ibus.setSendCallback(NULL);
    Scheduler::add(TAG_IKE_HELLO, 2500, [] (byte) {
      //:80 04 9F 1B
      ibus.write_ds2((const byte[]){0x80, 0x04, 0x9F, 0x1B});// Done Diagnostic
    });
  });
}

static inline void update_ignition(byte data1)
{
  data1 &= IGN_MASK;
  debugSerial.printf("Ignition key=%s%s ",
    (data1==IGN_START)?"Start":
    (data1==IGN_ON)?"Engine run":
    (data1==IGN_ACC)?"Accessory":"OFF",
    (data1!=ignition)?"(*)":"");

  if (data1!=ignition) {
    if (data1==IGN_START) {
      debugSerial.print("*IGNITION START* ");
      play_goodbye |= TRIGGER_START;
      play_letsgo |= TRIGGER_START;
    }
    if ((data1==IGN_ACC) && (ignition==IGN_OFF)) {
      debugSerial.print("*Turn accessory power* ");
      skip_RT = true;
      if ((play_hello&TRIGGER_HELLO)==TRIGGER_HELLO) {
        play_hello = TRIGGER_STOP;
        Speaker::play(Speaker::HELLO);
        Scheduler::add(TAG_IKE_SWEEP, 500, [] (byte) {
          needle_sweep();
        });
      }
    }
    ignition = data1;
  }
}

static void key_remote_lock(byte data1)
{
  //00 04 BF 72 02 CB,GM --> GLO: No_Button_Pressed
  //00 04 BF 72 12 DB,GM --> GLO: Lock_Pressed
  //00 04 BF 72 22 EB,GM --> GLO: Unlock_Pressed
  //00 04 BF 72 42 8B,GM --> GLO: Boot_Pressed
  byte t = (data1 & 0xF0)>>4;
  debugSerial.printf("Remote %d button %s", (data1 & 0xf),
    (t==0)?"No button pressed":
    (t==(1<<0))?"Lock":
    (t==(1<<1))?"Unlock":
    (t==(1<<2))?"Trunk":"unknown");
}

static int8_t vehicle_enabled = -1;
static int8_t doors_open = -1;
static void immobilizer(byte data1, byte data2)
{
  // (data1 & 1) != 0 Immobilisation_deactivated
  // data2==0xFF no key
  static const char* const imo_state2str[] = {
    "No key",
    "Key present"
    "Key valid"
    "Unknown",
  };
  byte enabled = (data1&1);
  debugSerial.printf("Vehicle %s%s, Key %i %02X=%s",
    enabled?"enabled":"immobilized", (vehicle_enabled!=enabled)?"(*)":"",
    data2, data1,
    imo_state2str[(data1>>1)&3]);
  if (vehicle_enabled!=enabled) {
    if (enabled) {//Immobilizer disabled, car can run
      //key_insert();
    } else {//Immobilizer enabled, car can't go
      //key_remove();
    }
    vehicle_enabled = enabled;
  }
}

static void dump_bits(const char* prefix, byte data, int num, const char* const arr[])
{
  byte mask = ~(0xFF<<num);
  if ((data&mask)==0) return;
  if (prefix) debugSerial.print(prefix);
  for (int i=0;i<num;i++) {
    if (data&(1<<i)) { debugSerial.print(' ');debugSerial.print(arr[i]); }
  }
}

static inline void door_status(byte data1, byte data2)
{
  typedef enum : byte {              //  data1    data2
    WINDOW_FL, //Window Front Left     00000000 00000001
    WINDOW_FR, //Window Front Right    00000000 00000010
    WINDOW_RL, //Window Rear Left      00000000 00000100
    WINDOW_RR, //Window Rear Right     00000000 00001000
    SUNROOF,   //Sunroof               00000000 00010000
    TRUNK,     //Trunk Boot/Bagazhnik  00000000 00100000
    HOOD,      //Hood  Bonnet/Kapot    00000000 01000000
    UNKNOWN,   //Unknown
    DOOR_FL,   //Door Front Left       00000001 00000000
    DOOR_FR,   //Door Front Right      00000010 00000000
    DOOR_RL,   //Door Rear Left        00000100 00000000
    DOOR_RR    //Door Rear Right       00001000 00000000
  } OPEN_STATUS;
  //uint16_t status = ((uint16_t)(data1<<8)|data2)&0xfff;
/*
00 05 BF 7A 51 20 B1    :  GM   --> GLO : Doors/flaps status: Open:  DvrDoorFr Boot   Ctrl_Locking: Unlocked

Locking:
00 04 BF 72 10 D9:    GM --> GLO : ChkCtrl Remote Central Locking: Lock_Pressed
00 05 BF 7A 60 0F AF: GM --> GLO : Doors/flaps status: Open: WndwDvr WndwPgr WndwDvrRr WndwPgrRr Ctrl_Locking: Unknown
00 04 BF 72 00 C9:    GM --> GLO : ChkCtrl Remote Central Locking: No_Button_Pressed
00 05 BF 7A 20 0F EF: GM --> GLO : Doors/flaps status: Open: WndwDvr WndwPgr WndwDvrRr WndwPgrRr Ctrl_Locking: Unknown


Unlocking:
00 04 BF 72 22 EB:    GM --> GLO : ChkCtrl Remote Central Locking: Unlock_Pressed
00 05 BF 7A 50 0F 9F: GM --> GLO : Doors/flaps status: Open: WndwDvr WndwPgr WndwDvrRr WndwPgrRr Ctrl_Locking: Unlocked
3F 0A 00 07 00 00 00 07 41 01 00 75:  DIA --> GM : Write memory, Data="00 00 00 07 41 01 00" [ A ]
00 03 3F A0 9C:       GM --> DIA : Diagnostic command acknowledged
00 04 BF 72 02 CB:    GM --> GLO : ChkCtrl Remote Central Locking: No_Button_Pressed
00 05 BF 7A 50 0F 9F: GM --> GLO : Doors/flaps status: Open: WndwDvr WndwPgr WndwDvrRr WndwPgrRr Ctrl_Locking: Unlocked
*/
  debugSerial.print("Door status ");
  if ((data1&1)!=(doors_open&1)) {
    //driver door open or closed
    if (data1&1) {//open
      debugSerial.print("Driver's DOOR OPEN ");
      if ((play_goodbye&TRIGGER_GOODBYE)==TRIGGER_GOODBYE) {
        play_goodbye = TRIGGER_STOP;
        Speaker::play(Speaker::GOOD_BYE);
      }
      play_letsgo |= TRIGGER_DOOR_OPEN;
      play_hello |= TRIGGER_DOOR_OPEN;
      play_goodbye |= TRIGGER_DOOR_OPEN;
    } else {//closed
      debugSerial.print("Driver's DOOR CLOSE ");
      play_letsgo |= TRIGGER_DOOR_CLOSE;
      play_hello |= TRIGGER_DOOR_CLOSE;
      play_goodbye |= TRIGGER_DOOR_CLOSE;
    }
  }
  doors_open = (data1 & 0xf);
  hood_open = (data2 & (1<<HOOD));
  dump_bits("Door", data1, 4, (const char* const[]){"FL","FR","RL","RR"});
  dump_bits("Wnd", data2, 4, (const char* const[]){"FL","FR","RL","RR"});
  dump_bits(NULL, data2>>4, 3, (const char* const[]){"Sunroof","Trunk","Hood"});
}

static void set_ike_time()
{
  struct tm dt;
  ds3231.get(dt);
  if (dst<0)
    IsDST(dt);

  dt.tm_hour += dst;
    dt.tm_isdst = dst;
//  if (h12) {
//    if (PM
//    hour += 12;
//    hour %= 24;
//  }
  //GT telling IKE to set the time:
  //3B 06 80 40 01 0C 3B cc
  //GT –> IKE : On-board computer set data: Set Time = 12:59
  //40 = OBC  Set data
  //01 = Time
  //0C = hours in hex
  //3B = minutes in hex
  const byte pkt_set_time[] = {0x3B,0x06,0x80,0x40,0x01,(byte)dt.tm_hour,(byte)dt.tm_min};
  ibus.write_kbus(pkt_set_time);

  //GT telling IKE to set the date:
  //3B 07 80 40 02 1B 05 08 cc
  //GT –> IKE : On-board computer set data: Set Date = 27/05/08
  //40 = OBC Set data
  //02 = Date
  //1B = day in hex
  //05 = month in hex
  //08 = year in hex
  const byte pkt_set_date[] = {0x3B,0x07,0x80,0x40,0x02,(byte)dt.tm_mday,(byte)(dt.tm_mon+1),(byte)(dt.tm_year-(2000-1900))};
  ibus.write_kbus(pkt_set_date);
}

////////////////////////////////////////////////////////////////////////////////////
#define DELAY_MIND_SIGNALS 30000
static bool turn_signal = false;

static void ibusPacketHandler(
  const IBUS_DEVICES src,
  const IBUS_DEVICES dst,
  const byte msg,
  byte *IBusBuffer) // callback function - Library calls this when good message received.
{
  byte len = IBusBuffer[IBUS_LEN];
  byte data1 = IBusBuffer[4];
  byte data2 = IBusBuffer[5];
  if (dst==0xBF) { // Broadcast to GLO
    switch (src) {
    case 0x00://GM5
      switch (msg) {
      case 0x02:
        //00 04 BF 02 00 B9,GM --> GLO: Device status ready
        //00 04 BF 02 01 B8,GM --> GLO: Device status ready, after Reset
        debugSerial.printf("Device status %d %s", data1, (data1==1)?"after Reset":"");
        break;
      case 0x72:
        //00 04 BF 72 00 C9,GM --> GLO: ChkCtrl Remote Central Locking, No_Button_Pressed
        //00 04 BF 72 02 CB,GM --> GLO: ChkCtrl Remote Central Locking, No_Button_Pressed
        //00 04 BF 72 10 D9,GM --> GLO: ChkCtrl Remote Central Locking, Lock_Pressed
        //00 04 BF 72 12 DB,GM --> GLO: ChkCtrl Remote Central Locking, Lock_Pressed
        //00 04 BF 72 20 E9,GM --> GLO: ChkCtrl Remote Central Locking, Unlock_Pressed
        //00 04 BF 72 22 EB,GM --> GLO: ChkCtrl Remote Central Locking, Unlock_Pressed
        //00 04 BF 72 42 8B,GM --> GLO: ChkCtrl Remote Central Locking, Boot_Pressed
        key_remote_lock(data1);
        break;
      case 0x76:
        //00 04 BF 76 00 CD,GM --> GLO: Crash Alarm, No_Crash
        //00 04 BF 76 02 CF,GM --> GLO: Crash Alarm, DB1=0x02
        debugSerial.printf("Crash Alarm %d", data1);
        break;
      case 0x7A:
        //00 05 BF 7A 10 00 CS: GM --> GLO : door/flap status:
        //00 05 BF 7A 90 20 CS: Ctrl_Locking: Unlocked Byte1_Bit7 Boot
        door_status(data1, data2);
        break;
      case 0x78:
        debugSerial.printf("Seat Memory %d", data1);
      }
      break;
    case 0x44://EWS
      if (msg==0x74) {
        immobilizer(data1, data2);
      }
      break;
    case 0x72://SMD
      if (msg==0x78) {
        //72 05 BF 78 05 00 B5 Save Memory: Position 1
        debugSerial.printf("Save memory %d", data2+1);
      }
      break;
    case 0x80://IKE
      switch (msg) {
      case 0x15: // change units
        //h12 = (data2&1);
        //USA (data1&0xf)==2
        //CAN (data1&0xf)==7
        debugSerial.printf("Change units. 12H=%d", (data2&1));
        break;
      case 0x11: //Answer to request :> GT --> IKE : Ignition status request  (Jochen)
        //80 04 BF 11 03 29
        update_ignition(data1);
        break;
      case 0x13:// Key/Gear/Motor status
        //http://web.comhem.se/bengt-olof.swing/IBus.htm
        //When gear is put in reverse and ignition is on these messages are sent: (Thanks saft6luck) I will use this to turn on rearview camera
        //Only the bold one bit is significant for these messages.
        //80 0A BF 13 02 10 00 00 00 00 38 0C // in
        //80 0A BF 13 02 00 00 00 00 00 38 1C //out
        //            -- -- -- --
        //Backward signal. Used for retrovisors auto moving.
        //80 0A BF 13 00 13 00 00 00 00 20
        gear_switch(data2);
        return;//break;
      case 0x18://Speed/RPM
        update_speed(2*data1, (data2==0xFF)?0:data2);
        break;
      case 0x19://Temperature
        //80 06 BF 19 17 45 00 72 //Temp
        debugSerial.printf("Temp Air=%dC/Cooling=%dC", data1, data2);
        break;
      }
      break;
    case 0xD0://src==LCM
      switch (msg) {
      case 0x5B://LCM lamp state broadcast
        //D0 08 BF 5B 00 00 00 00 00 //turnsig_stop
        //D0 08 BF 5B 04 00 00 00 00 //lightsig_highbeam
        //D0 08 BF 5B 20 00 04 00 00 //turnsig_left_start
        //D0 08 BF 5B 40 00 04 00 00 //turnsig_right_start
        //D0 08 BF 5B 60 00 04 00 00 //turnsig_warn
        //[4]=00-Stop,04-High beam,20-Left start,40-Right start,60-Warn start
        debugSerial.printf("Light status (Turn signal: %02X)", data1);
        switch (data1 & (0x20|0x40)) {
        case 0x20://left turn signal
        case 0x40://right turn signal
          if (!turn_signal) {
            turn_signal = true;
            backupCam.enable(TAG_TURN_LCM);
            Scheduler::add(TAG_TURN_MIND, DELAY_MIND_SIGNALS, [] (byte) {
              backupCam.disable(TAG_TURN_LCM);
              if (current_speed_mph>10)
                Speaker::play(Speaker::TURN_MIND);
              else {
                //Gong
                //80 05 0C 11 98  DIA IKE Set IO status Data="11"    [ ]
                //80 04 A0 24     IKE DIA Diagnostic command acknowledged Data=""    []
                ibus.write_ds2((const byte[]){ 0x80, 0x05, 0x0C, 0x11, 0x98});
                //30 19 80 1A 37 00 20 46 41 53 54 45 4E 20 53 45 41 54 20 42 45 4C 54 53 20 20 E0
                //CCM --> IKE : IKE Text/Gong: " FASTEN SEAT BELTS " Clear Gong_off Warning_symbol_off
                /*
                 * Describe the text line in the instrument cluster: ADDRESS 80 HEX
Format of the message:
<68 17 80 > 23 62 30 <Option> <Text in ASCII Hex> <XOR>
Note: All characters that can be displayed in the text line must always be sent, otherwise the "old" characters will not be overwritten. Thus, the message length is always "17H".
The message can be viewed with the following options:
35 00 normal display
37 01 Display text between two red files
37 03 Display texts between two red, flashing files
37 04 Display gong and text between two red, blinking files
37 05 Show gong and text
37 08 Gong I (without displaying a message)
37 10 Gong II (without displaying a message)

  putc0wc(0xac); //ac is ehc 30->check control module
  putc0wc(0x19); //25 chars after this incl 20 char msg
  putc0wc(0x80); //to the IKE inst cluster 0x80 or 0xe7? bf->broadcast hopefully from kbus over to ibus
  putc0wc(0x1A); //
  putc0wc(0x35); //to the bottom status line
  putc0wc(0x03); //00=text  01=text and gong  02=steady ><  3=flashing ><
  puts0wc(testmsg);
  putc0(chksm);  //check sum

  putc0wc(0xac); //ac=ehc 80=IKE
  putc0wc(0x0b); //11 chars after this incl 6 char msg
  putc0wc(0x80); //e7=OBC inst cluster
  putc0wc(0x24); //ANZV
  putc0wc(0x08); //count
  putc0wc(0x03); //1 text, 2 gong, 3 ><
  puts0wc(obcmsg);
  putc0(chksm);  //check sum

                 */
                //ibus.write_kbus((const byte[]){0x30,0x05,0x80,0x1A,0x11,0x08,0xB6}); //IKE Text/Gong: "" Immediate Clear Gong_off
              }
            });
          }
          break;
        case 0x60://warning
        default:
          if (turn_signal) {
            turn_signal = false;
            backupCam.disable(TAG_TURN_LCM);
            Scheduler::del(TAG_TURN_MIND);
          }
        }
        break;
      case 0x5C://LCM dimmer msg
        // decode ambient brightness messages from the LCM (Light Control Module)
        // format:
        // # byte
        // 3: 0x5C      (dimmer message)
        // 4: 0x00-0xFF (brightness dial value,0xFF when headlights are turned off)
        // 5: 0x3F-0xFF (ambient brightness sensor value)
        // 6: 0x00-0xFF (unknown)
        // 7: 0x00      (unknown)
        // Ambient light sensor switchover values (0-255)
        debugSerial.printf("dimmer_dial=0x%02X/sensor=%d", data1, data2);
        lcm_lightsensor(data1, data2);
        return;
      case 0x5D://LCM dimmer status
        //50 03 D0 5D DE //MFL requests LCM to get button backlight brightness
        debugSerial.printf("request dimmer status");
      default:;
      }
      break;
    default:;
    }
  } else //!dst==0xBF /////////////////////////////////////////////////////////////////////////////
  switch (src) {
  case 0x3F://src==DIAGNOSTICS
    if (dst==0x72 && msg==0x0C) { //GM5->SM?
      //3F 06 72 0C 02 01 00 Driver Seat: Move to Memory position 1
      //3F 06 72 0C 02 02 00 Driver Seat: Move to Memory position 2
      //3F 06 72 0C 02 04 00 Driver Seat: Move to Memory position 3
      int i;
      for (i=0;i<=2 && data2!=(1<<0);i++);
      debugSerial.printf("Move seat to position %d", (i<3)?(i+1):data2);
    } else {
      //DIA->IHK:3F 03 5B 9E F9
      //DIA->IHK:3F 03 5B 00 67
      if (msg==0x9E || msg==0x00)   //ping
        debugSerial.printf("ping (%02X)", msg);
    }
    break;
  case 0x50://src==MFL
    //if (dst!=0x68 && dst!=0xB0 && dst!=0xC8 && dst!=0xFF) //TEL,RAD,SES or 0xFF
    //  break;
    switch (msg) {
    case 0x01://ping
      //50 03 C8 01  -> TEL
      //50 03 B0 01  -> SES
      {
        debugSerial.print("Ping. Sending pong");
        //TEL --> MFL : Device status ready Bit3 Assist_ready_after_Reset Bit5
        //byte tel_reply[] = { dst, 0x04, 0x50, 0x02, 0x38 };//https://xoutpost.com/electronics/bluetooth/81491-retrofit-bluetoth-using-tcu-ece-european-us-x5-e53.html
        //byte tel_reply[] = { dst, 0x04, 0x50, 0x02, 0x30 };//http://bmwraspcontrol.de/board/printthread.php?tid=343&page=5
        //byte tel_reply[] = {0xC8, 0x04, 0x50/E7, 0x2C, xx };https://translate.google.com/translate?sl=de&tl=en&js=y&prev=_t&hl=en&ie=UTF-8&u=http%3A%2F%2Fwww.alextronic.de%2Fbmw%2Fprojects_bmw_info_ibus.html&edit-text=
        //https://github.com/cgart/OpenBM/blob/master/ibus-logs/mt-logs/NavCoder_Log_20111025_232731.log
        //3B 03 C8 01 F1:  GT   --> TEL : Device status request
        //C8 04 FF 02 30 01:  TEL  --> LOC : Device status ready Assist_ready_after_Reset Bit5
        //xx
        //Bit0 = Handsfree on
        //Bit1 = active call, (Telephone menu on on-board monitor or MID)
        //Bit2 = incoming call
        //Bit3 = Telephone display enabled
        //Bit4 = Telephone on
        //Bit5 = telephone active
        //Bit6 = Telephone adapter installed
        //const byte tel_ready[] = { dst, 0x04, 0xBF, 0x02, 0x01 };
        //ibus.write_kbus(tel_ready);
        //byte tel_reply[] = { dst, 0x04, 0x50, 0x02, 0x00 };
        const byte tel_reply[] = { dst, 0x04, 0x50, 0x02, 0x78 };
        ibus.write_kbus(tel_reply);
      }
      return;
    case 0x3B:
      switch (data1&0xCF) {//button code
      case 0x00://R/T 50 04 68 3B 02 CK: switch to Radio
      case 0x02://R/T 50 04 68 3B 02 CK: MFL R/T press
      case 0x40://R/T 50 04 FF 3B 40 CK: switch to Telephone Siri (RT_PRESS)
                //    50 04 C8 3B 40 CK:
                //    50 04 FF 3B 40 CK: MFL R/T press - activate SIRI
        if (skip_RT) {
          skip_RT = false; // on TEL detection MFL will send RT to switch to RAD
          debugSerial.print("ignore R/T button");
        } else {
          PioneerRemote::pressRT(data1);
        }
        break;
#define CODE2PRESS(code) (MFL_PRESS_TYPE)((code>>4)&3)
      case 0x80://Talk
        if (turn_signal && backupCam.isEnabled(TAG_TURN_LCM)) {
          if (CODE2PRESS(data1)==MFL_RELEASE)
            backupCam.disable(TAG_TURN_LCM);
        } else {
          PioneerRemote::pressTalk(CODE2PRESS(data1));
        }
        return;
      case 0x01://Next/FF
      case 0x08://Prev/Rew
        PioneerRemote::pressNav(CODE2PRESS(data1), (data1&1)!=0);
        return;
#undef CODE2PRESS
      default:
        debugSerial.printf("Unknown button=0x%02X", data1);
      }
      break;
    case 0x32: // Vol+/Vol-
      switch (data1) {
      case 0x10:PioneerRemote::press(PioneerRemote::VOL_DN, PRESS_TIME_SHORT);return;//Vol-
      case 0x11:PioneerRemote::press(PioneerRemote::VOL_UP, PRESS_TIME_SHORT);return;//Vol+
      //case 0x30:// Vol- release
      //case 0x31:// Vol+ release
      }
      break;
    }
    break;
  case 0x68://src==RAD
    if (dst==0xF0 && msg==0x4A) {//RAD -> BMBT
      // 68 04 F0 4A 00 D6 // POWER_OFF [] = Power off
      if (data1==0)
        debugSerial.print("Power off");
      else
        debugSerial.print(data1,HEX);
    }
    break;
  case 0x80://src==IKE
    //IKE->Broadcast text
    //time
    //80 0C FF 24 01 00 20 38 3A 30 34 50 4D 6D  _8:04PM
    //80 0C FF 24 01 00 32 30 3A 31 34 20 20 6B  20:14__
    //80 0C FF 24 01 00 2D 2D 3A 2D 2D 20 20 6C  --:--__
    //date
    //80 0F FF 24 02 00 2D 2D 2E 2D 2D 2E 32 30 30 32 56 --.--.2002
    //???
    //80 09 FF 24 03 00 2B 31 31 38 42
    //                  ^^ ^^ ^^ ^^    +118
    //temp
    //80 09 FF 24 09 00 20 35 30 20 5E           _50_ (degreeC)
    //80 0e e7 24 0e 00 xx xx xx xx xx xx xx xx xx chk Output of the timer as plain text. xx is the text.
    //80 0e e7 24 1a 00xxxxxxxxxxxxxxxx chk Output of the stopwatch, is only sent if the stopwatch has been requested and started before xx is the time in plain text (xx is the text)
    if (dst==0xFF && msg==0x24) {//Update ANZV
      static const char* const update_anzv_str[] = {
        "unk0",//0
        "Time",//1
        "Date",//2
        "Outside Temperature",//3
        "Consumption 1",//4
        "Consumption 2",//5
        "Range",//6
        "Distance",//7
        "Arrival",//8
        "Speed Limit",//9
        "Average Speed"//10
      };
      IBusBuffer[len+1] = '\0';// replace checksum with NULL
      if (data1<countof(update_anzv_str))
        debugSerial.printf("%s: %s", update_anzv_str[data1], (char*)IBusBuffer+6);
      else
        debugSerial.printf("%02Xh: %s", data1, (char*)IBusBuffer+6);
      if (data1==1) oled_show_time((char*)IBusBuffer+6);else
      if (data1==2) oled_show_date((char*)IBusBuffer+6);
    }
    break;
  case 0xD0://src=LCM
    if (dst==0x80 && msg==0x54)//IKE get VIN
      debugSerial.printf("VIN info");
    break;
  case 0xE8://src==RLS
    //RLS->Broadcast: E8 03 00 75          Wiper status request
    //RLS->Broadcast: E8 06 00 58 FF 7F 00 Headlight wipe interval
    if (dst==0xD0 && msg==0x59) {//RLS->LCM
      //RLS->NAV: E8 05 D0 59 50 00
      //RLS->NAV: E8 05 D0 59 40 00
      rls_lightsensor(data1, data2);
      return;
    }
    break;
  case 0x9B://src==MML
    if (dst==0x51 && msg==0x6D) {//MMR  sideview mirror selector switch
      //The sideview mirror selector switch on the left door armrest will send these messages: (Thanks saft6luck) I will use this message to turn on rearview camera
      //9B 04 51 6D 40 E3 // right
      //9B 04 51 6D 80 23 // left
      //saft6luck: "For my LH driven car the 0x9B is the left and 0x51 is the right mirror module.
      //The mirrors are controlled by these modules and the buttons are read in by the 0x9B module in my case."
      debugSerial.printf("Select %s side mirror", (data1==0x40)?"right":(data1==0x80)?"left":"unknown");
      return;
    }
    break;
  default:;
  }//switch(src)
}
static inline void setup()
{
  output_pin(PIN_IBUS_ENABLE, HIGH); // HIGH enables I/K-Bus MCP/Melexis transceiver chip
  ibus.begin(debugSerial, ibusPacketHandler);
}
static inline void loop()
{
  ibus.run(); // This keeps the IBus library running.
}
}//namespace BMW
////////////////////////////////////////////////////////////////////////////////////
// Menu
// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce2
#include "src/Bounce2/src/Bounce2.h"
static Bounce debouncer = Bounce();


namespace Command {

String input;
static inline void input_help()
{
  debugSerial.println(F("Enter K-Bus commands: 01 02 03 ....."));
}
static void input_error(const __FlashStringHelper *msg, int pos)
{
  debugSerial.println(msg);
  debugSerial.println(input);
  while (pos-->0)
    debugSerial.print(' ');
  debugSerial.println('^');
}

static inline void process_input(String &input)
{
  int t = 0;
  input.trim();
  switch (tolower(input[0])) {
  case ':': input.remove(0,1); t = 1; break;//ds2
  case '*': input.remove(0,1); t = 2; break;//as is
  case '^': BMW::needle_sweep(); return;
  case 'h': input_help(); return;
  case 'i':if (input.length()==2) {
      if (input[1]=='0')
        dimmer.disable(TAG_MANUAL);
      else
        dimmer.enable(TAG_MANUAL);
      return;
    }
    break;
  case 'w': {
      int code = atoi(input.c_str()+1);
      debugSerial.printf("Send wired remote code: %d for 4s\r\n", code);
      PioneerRemote::press((PioneerRemote::PIONEER_BTN)code, 4000);
      return;
    }
  }
  // process hex buffer
  int buf_len = 0;
  const char *p = input.c_str();
  char *end = NULL;
  int n;
  while (p!=NULL && *p!='\0') {
    end = NULL;
    n = strtol(p, &end, 16);
    if (end==p) {
      if (end[0]=='\\') {
        end[0] = '0';
        continue;
      }
      if (strchr(",{}", end[0]) != NULL) {
        p = end+1;
        continue;
      }
      input_error(F("Parser error:"), end-input.c_str());
      return;
    }
    if (n>255) {
      input_error(F("Range error:"), p-input.c_str());
      return;
    }
    if (buf_len>=sizeof(IBusBuffer)) {
      input_error(F("Buffer is too long:"), p-input.c_str());
      return;
    }
    IBusBuffer[buf_len++] = n;
    p = end;
  }
  debugSerial.print(F("Sending:"));
  printHex(debugSerial, IBusBuffer, buf_len);
  switch (t) {
  case 1:  BMW::ibus.write_ds2(IBusBuffer);break;
  case 2:  BMW::ibus.write(IBusBuffer, buf_len);break;
  case 0:
  default: BMW::ibus.write_kbus(IBusBuffer);break;
  }
}

static inline void setup()
{
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  // After setting up the button, setup the Bounce instance :
  debouncer.attach(PIN_BUTTON);
  debouncer.interval(20); // interval in ms
}

static inline void loop()
{
  while (debugSerial.available()) {
    char c = debugSerial.read();
    if (c=='\r' || c=='\n') {
      if (input.length()>0)
        process_input(input);
      input = "";
      break;
    }
    input += c;
  }

  debouncer.update();
  //int value = debouncer.read();
  if (debouncer.rose()) {
    Serial.println("release");
    BMW::set_ike_time();
  }
  //if (debouncer.fell())
  //  Serial.println("pressed");
}
}//namespace Command
////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();
  //esp_log_level_set("*", ESP_LOG_VERBOSE);
  Debug::setup();
  Speaker::setup();
  Display::setup();
  PioneerRemote::setup();
  Command::setup();
  Wifi::setup();
  BMW::setup();
  Scheduler::setup([]() {/*oled_show_rtc();*/});
}

void loop()
{
  Scheduler::idle();
  Command::loop();
  //PioneerRemote::loop();
  Wifi::loop();
  Speaker::loop();
  BMW::loop();
}
