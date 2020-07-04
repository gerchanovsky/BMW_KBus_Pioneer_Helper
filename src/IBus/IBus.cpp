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

#include "IBus.h"

static byte senStaPin = 35; // sen/Sta output from Melexix TH3122.
static volatile boolean clearToSend = true;
static RingBuffer sendBuffer(128);
static RingBuffer recvBuffer(128);
#define PACKET_GAP_MS 10
#define SENSTA_GAP_MS 1500
static signed long packetGapTarget;
static signed long clearToSendTarget;
byte IBusBuffer[IBUS_BUF_SIZE];
static Print *debug = &Serial;

void printHex(Print &print, const byte b)
{
  static const char hex[] = "0123456789ABCDEF";
  byte str[3] = {hex[b>>4], hex[b&0xf], ' '};
  print.write(str, 3);
}

void printHex(Print &print, const byte *buffer, int8_t length, const char *terminator)
{
  while (length-->0)
    printHex(print, *buffer++);
  print.print(terminator);
}

IBus::IBus(HardwareSerial &newIbusSerial, const byte senSta) : uart(newIbusSerial)
{
  senStaPin = senSta;
  debug = &Serial;
}

void IBus::begin(Print &newIbusDebug, IbusPacketHandler_t newHandler)
{
  debug = &newIbusDebug;
  uart.begin(9600, SERIAL_8E1); // ibus always 9600 8E1
#ifdef ESP32
  uart.setRxBufferSize(1024);
#elif TEENSYDUINO
  uart.attachCts(senStaPin);
//  uart.transmitterEnable(senStaPin);
#endif

  pPacketHandler = newHandler;

#if TEENSYDUINO
  pinMode(senStaPin, INPUT);
  clearToSend = (digitalRead(senStaPin)==LOW);
#endif
  clearToSendTarget = 0;
  packetGapTarget = millis()+PACKET_GAP_MS;

  attachInterrupt(digitalPinToInterrupt(senStaPin), startTimerISR, CHANGE);
  Serial.printf("IBus Transmission buffer size: %d\n", uart.availableForWrite());
}

//========================
void IRAM_ATTR IBus::startTimerISR()
{
  if (digitalRead(senStaPin)==LOW) {
    clearToSendTarget = millis()+SENSTA_GAP_MS;
    if (clearToSendTarget==0)
      clearToSendTarget++;
  } else {
    clearToSend = false;
    clearToSendTarget = 0;
  }
}

void IBus::run()
{
  readIbus();
  signed long _millis = millis();
  if (clearToSendTarget!=0 && ((_millis-clearToSendTarget) >= 0)) {
    clearToSend = true;
    clearToSendTarget = 0;
  }
  if (clearToSend  &&
      (digitalRead(senStaPin) == LOW) &&
      (sendBuffer.available() > 0) &&
      ((_millis-packetGapTarget) > 0)) {
    packetGapTarget = _millis+PACKET_GAP_MS;
    sendIbus();
    if (pSendCallback)
      pSendCallback();
  }
}

void IBus::write(const byte *message, int8_t len)
{
  if (len > 32) return;
  sendBuffer.push(len);
  while (--len>0) {
    sendBuffer.push(*message++);
  }
}

void IBus::writeCS(const byte *message, int8_t len)
{
  if (len > 32) return;
  byte checksum = 0;
  sendBuffer.push(len);
  while (--len>0) {
    byte b = *message++;
    checksum ^= b;
    sendBuffer.push(b);
  }
  sendBuffer.push(checksum);
}

bool IBus::readIbus()
{
#if 1
  register int available = uart.available();
  while (available-->0)
    recvBuffer.push((byte)uart.read());
  while ((available = recvBuffer.available()) >= 4) {

    int length = recvBuffer.peek(IBUS_LEN);
    if (length < 2 || length > sizeof(IBusBuffer)) { // Check if length byte between decimal 3 & 36
#ifdef PROTOCOL_DEBUG
      debug->printf("Bad length: 0x%02X avail=%d tail=%d remove %02X| ", (byte)length, available, recvBuffer.tail, recvBuffer.pop());
      recvBuffer.dump();
#else
      recvBuffer.remove(); // remove first byte and start over
#endif
      continue;
    }
    if (length > available)
      return false;

    int n = length-1;
    byte i = 0, checksum = 0, b;
    int pos = recvBuffer.tail;
    while (--n>=0) {
      b = recvBuffer.buffer[pos];
      checksum ^= b;
      IBusBuffer[i++] = b;
      pos = (pos+1) % recvBuffer.size;
    }
    b = recvBuffer.buffer[pos];
    IBusBuffer[i++] = b;
    // check DS2 packet
    if (b==checksum) {
      recvBuffer.remove(length);
#ifdef RX_DEBUG
      debug->printf("ds2:");
      printHex(*debug, IBusBuffer, length, ": >");
      ibus2str(*debug, IBusBuffer[0]);
      debug->printf("$%d:\r\n", length-3);
#endif
      return true;
    }

    length += 2;
    if (length > available)
      return false;

    pos = (pos+1) % recvBuffer.size;
    checksum ^= b;
    b = recvBuffer.buffer[pos];
    IBusBuffer[i++] = b;

    pos = (pos+1) % recvBuffer.size;
    checksum ^= b;
    b = recvBuffer.buffer[pos];
    IBusBuffer[i++] = b;

    // check K-Bus packet
    if (b==checksum) {
      recvBuffer.remove(length);
      if (length==4) {
        int c = recvBuffer.pop();
        if (c!=b)
          debug->printf("d(%02x!=%x)",b,c);
        else
          debug->print('!');
      }
#ifdef RX_DEBUG
      debug->printf("kbus:");
      printHex(*debug, IBusBuffer, length, ": ");
      ibus2str(*debug, IBusBuffer[IBUS_SRC]);
      debug->print(">");
      ibus2str(*debug, IBusBuffer[IBUS_DST]);
      debug->printf("$%d: ", length-4);
#endif
      if (pPacketHandler)
      pPacketHandler(
        (IBUS_DEVICES)IBusBuffer[IBUS_SRC],
        (IBUS_DEVICES)IBusBuffer[IBUS_DST],
        IBusBuffer[IBUS_MSG],
        IBusBuffer);
      debug->println();
      return true;
    }
#ifdef BAD_DEBUG
    debug->printf("Bad checksum: 0x%02X!=0x%02X len=%d/%d tail=%d remove=%02X|", b, checksum, (byte)length, available,
      recvBuffer.tail, recvBuffer.pop());
    printHex(*debug, IBusBuffer, length);
#else
    recvBuffer.remove(); // remove first byte and start over
#endif
  }
#else
  static int pos = 0;
  while (uart.available()>0) {
    byte b=uart.read();
    printHex(*debug, b);
    if (((++pos) % 32)==0) debug->println();
  }
#endif
  return false;
}

void IBus::sendIbus()
{
  int8_t len = sendBuffer.pop();
#ifdef TX_DEBUG
  debug->printf("Sending %d bytes(avail=%d): ", len, sendBuffer.available());
  sendBuffer.dump(len);
#endif
  while (len-->0) {
    byte b = sendBuffer.pop();
    uart.write(b); // write byte to IBUS.
  }
}
