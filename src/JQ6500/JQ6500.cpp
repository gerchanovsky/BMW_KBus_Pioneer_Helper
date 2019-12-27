/**
 * Arduino Library for JQ6500 MP3 Module
 *
 * Copyright (C) 2019 Alex Gerchanovsky
 * Copyright (C) 2014 James Sleeman, <http://sparks.gogo.co.nz/jq6500/index.html>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @author James Sleeman, http://sparks.gogo.co.nz/
 * @license MIT License
 * @file
 */

#include "JQ6500.h"

// The response from a status query we get is  for some reason
// a bit... iffy, most of the time it is reliable, but sometimes
// instead of a playing (1) response, we get a paused (2) response
// even though it is playing.  Stopped responses seem reliable.
// So to work around this when getStatus() is called we actually
// request the status this many times and only if one of them is STOPPED
// or they are all in agreement that it is playing or paused then
// we return that status.  If some of them differ, we do another set
// of tests etc...
#define STATUS_CHECKS_IN_AGREEMENT 4

void JQ6500::begin(byte rxPin, byte txPin)
{
    uart.begin(9600, SERIAL_8N1, rxPin, txPin);
}

//////////////////////////////////////////////////////////////////
JQ6500::STATUS_TYPE JQ6500::getStatus()
{
    byte statTotal;
    do {
        statTotal = 0;
        for (byte x = 0; x < STATUS_CHECKS_IN_AGREEMENT; x++) {
            byte stat = sendCommandWithResponse(CMD_GET_STATUS);
            if (stat == STATUS_STOPPED) return STATUS_STOPPED; // STOP is fairly reliable
            statTotal += stat;
        }
    } while (statTotal != 1 * STATUS_CHECKS_IN_AGREEMENT && statTotal != 2 * STATUS_CHECKS_IN_AGREEMENT);
    return (STATUS_TYPE)(statTotal / STATUS_CHECKS_IN_AGREEMENT);
}

int JQ6500::sendCommand(const CMD_TYPE command)
{
    byte buf[6] = {CMD_BEGIN, 2, command, CMD_END};            // [7E][number bytes following including command and terminator][command][EF]
    return sendCommand(buf);
}

int JQ6500::sendCommand(const CMD_TYPE command, const byte arg1)
{
    byte buf[6] = {CMD_BEGIN, 3, command, arg1, CMD_END};      // [7E][number bytes following including command and terminator][command][arg1][EF]
    return sendCommand(buf);
}

int JQ6500::sendCommand(const CMD_TYPE command, const byte arg1, const byte arg2)
{
    byte buf[6] = {CMD_BEGIN, 4, command, arg1, arg2, CMD_END};// [7E][number bytes following including command and terminator][command][arg1][arg2][EF]
    return sendCommand(buf);
}

int JQ6500::sendCommand(const byte *buf, bool readStatus)
{
    // The device appears to send some sort of status information (namely "STOP" when it stops playing)
    // just discard this right before we send the command
    while (wait(10)>0)
        uart.read();

    uart.write(buf, buf[1]+2);
    if (!readStatus)
        return 0;
    int ms = 1000;
    int len, ret = 0;
    while ((len = wait(ms))>0) {
        while (len-->0) {
            ++ret;
            uart.read();
        }
        ms = 50;
    }
    return ret;
}

// Used for the status commands, they mostly return an 8 to 16 bit integer
// and take no arguments
unsigned int JQ6500::sendCommandWithResponse(const CMD_TYPE command)
{
    byte buffer[5] = {CMD_BEGIN, 2, command, CMD_END};

    // The device appears to send some sort of status information (namely "STOP" when it stops playing)
    // just discard this right before we send the command
    while (wait(10)>0)
        uart.read();

    uart.write(buffer, buffer[1]+2);

    // Allow some time for the device to process what we did and
    // respond, up to 1 second, but typically only a few ms.
    int bufferLength = sizeof(buffer);//5
    char *responseBuffer = (char*)&buffer;
    memset(responseBuffer, 0, bufferLength);

    int ms = 1000;
    int len, ret = 0;
    while ((len = wait(ms))>0) {
        while (len-->0) {
            ++ret;
            char j = (char)uart.read();
            if (bufferLength-->0)
                *responseBuffer++ = j;
            taskYIELD();
        }
        ms = 150;
    }
    return (unsigned int)strtoul((const char*)&buffer, NULL, 16);
}

// Waits until data becomes available, or a timeout occurs
int JQ6500::wait(const signed long maxWaitTime)
{
    signed long endTime = ((signed long)millis()) + maxWaitTime;
    int c = 0;
    do {
        c = uart.available();
        taskYIELD();
    } while ((c==0) && ((endTime - (signed long)millis()) > 0));
    return c;
}
