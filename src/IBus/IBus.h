/*
  Copyright (c) 2015 Ian R. Haynes.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#include "Arduino.h"
#include "Print.h"
#include "RingBuffer.h"
////////////////////////////////////////////
extern void printHex(Print &print, const byte b);
extern void printHex(Print &print, const byte *buffer, int8_t length, const char *terminator = "\r\n");

//---------------DEBUG SETTINGS-------------
// debugSerial settings are located in the sketch file (Hardware serial, AltSoftSerial, or SoftwareSerial)
//
//---------------Standard debug printout-------------
//#define STD_DEBUG // Standard debug - Prints only: - valid received messages and - source byte of discarded messages (if source filtering is enabled)
//---------------Verbose debug printout-------------
// Disable standard debug if you will use these
#define PROTOCOL_DEBUG // Print messages, discarded messages listed under the "interested sources" in IbusSerial.cpp
#define BAD_DEBUG       // Print messages, bad messages with an invalid checksum byte
#define TX_DEBUG        // Print messages, sent to the I-Bus
#define RX_DEBUG        // Print messages, valid messages from our "interested sources" list in IbusSerial.cpp
//////////////////////////////////////////////////
//---------------MESSAGE FILTER SETTINGS-------------
// Enables the message source filter. Any messages coming from a source not listed below, will be discarded.
typedef enum : byte {
  GM5	= 0x00, // GM=General Module, Central Body, Comfort Module (windows, locks, wipers, etc.)
  DME	= 0x12, // Engine
  CDC	= 0x18, // CD Changer
  //FUH	= 0x28, // Radio controlled clock
  //TRAN= 0x32, // Automatic Trnsmission
  NAV	= 0x3B, // GT=Graphics driver (in navigation system)/Video Module
  DIA	= 0x3F, // Diagnostic
  FBZV	= 0x40, // Remote control central locking
  SCR	= 0x43, // MenuScreen
  EWS	= 0x44, // Immobiliser (Alarm)
  MFL	= 0x50, // Multi function steering wheel
  MML	= 0x51, // Mirror Memory Driver
  ABS	= 0x56, // ABS, ASC, DSC (Traction control)
  IHKA	= 0x5B, // Integrated heating and air conditioning, Climate control
  //PDC	= 0x60, // Park distance control
  RAD	= 0x68, // Radio
  //DSP	= 0x6A, // Digital signal processing audio amplifier
  RDC	= 0x70,
  SMD	= 0x72, // Seat memory driver
  CDCD	= 0x76, // CD changer, DIN size.
  //NAVE= 0x7F, // NAVE/Navigation (Europe),SCR_MENU
  IKE	= 0x80, // KMB, Instrument cluster electronics,
  MMR	= 0x9B, // Mirror Memory Passenger (also 9C?)
  //ABM	= 0xA4, // SRS Airbag module
  SES	= 0xB0, // Voice input, Speach recognition system
  GLO	= 0xBF, // Global, broadcast address/LCM Light Control Module?
  //MID	= 0xC0, // Multi-info display buttons
  TEL	= 0xC8, // Telephone
  LCM	= 0xD0, // LCM/LSZ=Light control module
  RLS	= 0xE8, // Rain/Light Sensor
  //ANZV= 0xE7, // OBC, Front display, OBC TextBar
  //SM	= 0xED, // Seat memory (also 0x72,0xDA), Lights, Wipers, TV=Television
  //BMBT= 0xF0, // On-board monitor Buttons, Screen buttons
  LOC	= 0xFF  // Local broadcast
} IBUS_DEVICES;

typedef enum : byte {
  IBUS_SRC = 0,
  IBUS_LEN = 1,
  IBUS_DST = 2,
  IBUS_MSG = 3,
  IBUS_DTA = 4,
} IBUS_MSG_IDX;

static void ibus2str(Print &debugSerial, const byte dev)
{
  const __FlashStringHelper *message = NULL;
  switch (dev) {
#define C(a) case a:message = F(#a); break;
  C(GM5);
  C(CDC);
  C(DIA);
  C(EWS);
  C(MFL);
  C(MML);
  C(IHKA);
  C(SMD);
  C(IKE);
  C(MMR);
  C(SES);
  C(GLO);
  C(TEL);
  C(LCM);
  C(RLS);
  C(LOC);
#undef C
  default: printHex(debugSerial, dev); return;
  }
  debugSerial.print(message);
}
#if 0
static inline void dump_ibus(const byte msg)
{
  const __FlashStringHelper *message = NULL;
  switch (msg) {
  //https://web.archive.org/web/20120427234707/http://ibus.stuge.se:80/Main_Page
#define C(a,b) case a:message = F(b);break;
  C(0x01, "Device status request (ping)");
  C(0x02, "Device status ready (pong)"); // response to 0x01
  C(0x03, "Bus status request");
  C(0x04, "Bus status");
  C(0x06, "DIAG read memory");
  C(0x07, "DIAG write memory");
  C(0x08, "DIAG read coding data");
  C(0x09, "DIAG write coding data");
  C(0x0C, "Vehicle control");
  C(0x10, "Ignition status request");
  C(0x11, "Ignition status");
  C(0x12, "IKE sensor status request");
  C(0x13, "IKE sensor status");
  C(0x14, "Country coding status request");
  C(0x15, "Country coding status");
  C(0x16, "Odometer request");
  C(0x17, "Odometer");
  C(0x18, "Speed/RPM");
  C(0x19, "Temperature");
  C(0x1A, "IKE text display/Gong");
  C(0x1B, "IKE text status");
  C(0x1C, "Gong");
  C(0x1D, "Temperature request");
  C(0x1F, "UTC time and date");
  C(0x23, "Update MID");
  C(0x24, "Update ANZV");
  C(0x2A, "On-Board Computer State Update");
  C(0x34, "DSP Equalizer Button");
  C(0x38, "CD status request");
  C(0x39, "CD status");
  C(0x40, "Set On-Board Computer Data");
  C(0x41, "On-Board Computer Data Request");
  C(0x48, "BMBT buttons");
  C(0x49, "BMBT buttons");
  C(0x4F, "RGB Control");
  C(0x53, "Vehicle data request");
  C(0x54, "Vehicle data status");
  C(0x59: RLS update");
  C(0x5A, "Lamp state request");
  C(0x5B, "Lamp state");
  C(0x5D: "LCM dimmer value");
  C(0x71, "Rain sensor status request");
  C(0x76, "Crash Alarm");
  C(0x7A, "Door status");
  C(0x7D, "Sunroof control");
  C(0x30, "DIAG IKE self test");
  C(0x9F, "DIAG stop");
  C(0xA0, "DIAG data reply");
  C(0xAA, "Navigation Control");
#undef C
  }
  default:;
    debugSerial.println();
    return;
  }
  debugSerial.println(message);
}
#endif
class IBus {
public:
    IBus(HardwareSerial &newIbusSerial, const byte senSta);
    typedef void IbusPacketHandler_t(const IBUS_DEVICES src, const IBUS_DEVICES dst, const byte msg, byte *packet);
    typedef void IbusSendCallback_t();
    void begin(Print &newIbusDebug, const IbusPacketHandler_t newHandler);
    void setSendCallback(const IbusSendCallback_t newCallback) {pSendCallback = newCallback;};
    void run();
    void write_kbus(const void *message) { writeCS((const byte*)message, ((const byte*)message)[IBUS_LEN]+2);}
    void write_ds2(const void *message)  { writeCS((const byte*)message, ((const byte*)message)[IBUS_LEN]); };
    void write(const byte *message, int8_t len);

private:
    void writeCS(const byte *message, int8_t len);

    static void IRAM_ATTR startTimerISR();
    bool readIbus();
    void sendIbus();

    IbusPacketHandler_t *pPacketHandler = NULL;
    IbusSendCallback_t *pSendCallback = NULL;
    HardwareSerial &ibusSerial;
};

#define IBUS_BUF_SIZE 32
extern byte IBusBuffer[IBUS_BUF_SIZE];
