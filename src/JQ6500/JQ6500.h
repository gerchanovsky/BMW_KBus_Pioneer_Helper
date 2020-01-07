/**
 * Arduino Library for JQ6500 MP3 Module
 *
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
#pragma once

#include "Arduino.h"
#include "HardwareSerial.h"

class JQ6500 {
    HardwareSerial &uart;
public:
    typedef enum : byte {
        EQ_NORMAL,
        EQ_POP,
        EQ_ROCK,
        EQ_JAZZ,
        EQ_CLASSIC,
        EQ_BASS
    } EQ_TYPE;

    typedef enum : byte {
        SRC_USB,
        SRC_SDCARD,
        SRC_AUX,
        SRC_SLEEP,
        SRC_FLASH
    } SRC_TYPE;

    typedef enum : byte {
        LOOP_ALL,      // ALL plays all the tracks in a repeating loop
        LOOP_FOLDER,   // FOLDER plays all the tracks in the same folder in a repeating loop
        LOOP_ONE,      // ONE plays the same track repeating
        LOOP_RAM,      // RAM seems to play one track and someties disables the ability to move to next/previous track, really weird.
        LOOP_ONE_STOP  // ONE_STOP does not loop, plays the track and stops
    } LOOP_TYPE;

    typedef enum : byte {
        STATUS_STOPPED,
        STATUS_PLAYING,
        STATUS_PAUSED
    } STATUS_TYPE;

    /*  Example, create global instance:
     *     JQ6500 mp3(8,9);
     */
    JQ6500(HardwareSerial &port) : uart(port) {}
    void begin(byte rxPin, byte txPin);

    // Start playing the current file.
    void play() { sendCommand(CMD_PLAY); }
    void stop() { sendCommand(CMD_STOP); }
    // Pause the current file.  To unpause, use play(), to unpause and go back to beginning of track use restart()
    void pause() { sendCommand(CMD_PAUSE); }
    // Play the next file.
    void next() { sendCommand(CMD_NEXT); }
    // Play the previous file.
    void prev() { sendCommand(CMD_PREV); }
    // Play the next folder.
    void nextFolder() { sendCommand(CMD_NEXT_FOLDER, 0x01); }
    // Play the previous folder.
    void prevFolder() { sendCommand(CMD_PREV_FOLDER, 0x00); }

    /** Play a specific file based on it's (FAT table) index number.  Note that the index number
     *  has nothing to do with the file name (except if you uploaded/copied them to the media in
     *  order of file name).
     *
     *  To sort your SD Card FAT table, search for a FAT sorting utility for your operating system
     *  of choice.
     */
    void play(const uint16_t fileNumber) { sendCommand(CMD_PLAY_IDX, (fileNumber>>8) & 0xFF, fileNumber & 0xFF); }
    unsigned int currentFile() { return sendCommandWithResponse(CMD_GET_FILE_IDX_FLASH)+1; }
    // Count the number of files on the specified media.
    unsigned int countFiles() { return sendCommandWithResponse(CMD_GET_COUNT_FLASH); }
    // For the currently playing or paused file, return the current position in seconds.
    unsigned int currentFilePosition() { return sendCommandWithResponse(CMD_GET_FILE_POS_SEC); }
    // For the currently playing or paused file, return the total length of the file in seconds.
    unsigned int currentFileLength()   { return sendCommandWithResponse(CMD_GET_FILE_LEN_SEC); }

    /** Play a specific file in a specific folder based on the name of those folder and file.
     *
     * Only applies to SD Card.
     *
     * To use this function, folders must be named from 00 to 99, and the files in those folders
     * must be named from 000.mp3 to 999.mp3
     *
     * So to play the file on the SD Card "/03/006.mp3" use mp3.playFileNumberInFolderNumber(3, 6);
     */
    void play(const byte folderNumber, const byte fileNumber) { sendCommand(CMD_PLAY_FILE_FOLDER, folderNumber & 0xFF, fileNumber & 0xFF); }

    // Increase the volume by 1 (volume ranges 0 to 30).
    void volumeUp() { sendCommand(CMD_VOL_UP); }
    // Decrease the volume by 1 (volume ranges 0 to 30).
    void volumeDn() { sendCommand(CMD_VOL_DN); }
    // Set the volume to a specific level (0 to 30).
    // @param volumeFrom0To30 Level of volume to set from 0 to 30
    void setVolume(const byte volumeFrom0To30) { sendCommand(CMD_VOL_SET, volumeFrom0To30); }
    byte getVolume() { return sendCommandWithResponse(CMD_GET_VOL); }

    //* Set the equalizer to one of 6 preset modes.
    void setEqualizer(const EQ_TYPE equalizerMode) { sendCommand(CMD_EQ_SET, equalizerMode); } // EQ_NORMAL to EQ_BASS
    byte getEqualizer() { return sendCommandWithResponse(CMD_GET_EQ); }

    //* Set the looping mode.
    void setLoopMode(const LOOP_TYPE loopMode) { sendCommand(CMD_LOOP_ALL, loopMode); }
    byte getLoopMode() { return sendCommandWithResponse(CMD_GET_LOOP); }

    //* Set the source to read mp3 data from.
    void setSource(const SRC_TYPE source) { sendCommand(CMD_SOURCE_SET, source); }

    /** Put the device to sleep.
     *  Not recommanded if you are using SD Card as for some reason
     *  it appears to cause the SD Card to not be recognised again
     *  until the device is totally powered off and on again :-/
     */
    void sleep() { sendCommand(CMD_SLEEP); }

    /** Reset the device (softly).
     *
     *  It may be necessary in practice to actually power-cycle the device
     *  as sometimes it can get a bit confused, especially if changing
     *  SD Cards on-the-fly which really doesn't work too well.
     *
     *  So if designing a PCB/circuit including JQ6500 modules it might be
     *  worth while to include such ability (ie, power the device through
     *  a MOSFET which you can turn on/off at will).
     */
    void reset() { sendCommand(CMD_RESET); delay(500); } // We need some time for the reset to happen

    // Status querying commands
    /** Get the status from the device.
     *
     * CAUTION!  This is somewhat unreliable for the following reasons...
     *
     *  1. When playing from the on board memory (SRC_FLASH), STOPPED sems
     *     to never be returned, only PLAYING and PAUSED
     *  2. Sometimes PAUSED is returned when it is PLAYING, to try and catch this
     *     getStatus() actually queries the module several times to ensure that
     *     it is really sure about what it tells us.
     *
     * @return One of STATUS_PAUSED, STATUS_PLAYING and STATUS_STOPPED
     */
    STATUS_TYPE getStatus();

protected:
    typedef enum : byte {
        CMD_BEGIN                = 0x7E,
        CMD_END                  = 0xEF,

        CMD_NEXT                 = 0x01,
        CMD_PREV                 = 0x02,
        CMD_PLAY_IDX             = 0x03,

        CMD_VOL_UP               = 0x04,
        CMD_VOL_DN               = 0x05,
        CMD_VOL_SET              = 0x06,

        CMD_EQ_SET               = 0x07,
        CMD_LOOP                 = 0x08,//sendStack(0x08, int fileNumber);
        CMD_SOURCE_SET           = 0x09,
        CMD_SLEEP                = 0x0A,
        CMD_RESET                = 0x0C,
        CMD_PLAY                 = 0x0D,
        CMD_PAUSE                = 0x0E,

        CMD_NEXT_FOLDER          = 0x0F,
        CMD_PREV_FOLDER          = 0x0F,// Note the same as next, the data byte indicates direction
        CMD_STOP                 = 0x10,//reserved???
        //CMD_OUTPUT_SETTINGS      = 0x10,//sendStack(0x10, bool enable, byte gain);
        CMD_LOOP_ALL             = 0x11,

        CMD_PLAY_FILE_FOLDER     = 0x12,//sendStack(0x12, int fileNumber);
        //CMD_ADVERTISE            = 0x13,//sendStack(0x13, int fileNumber);
        //CMD_PLAY_LARGE_FOLDER    = 0x14,//sendStack(0x14, (((uint16_t)folderNumber) << 12) | fileNumber);
        //CMD_ADVERTISE_STOP       = 0x15,//sendStack(0x15);
        //CMD_STOP                 = 0x16,//sendStack(0x16);
        //CMD_LOOP_FOLDER          = 0x17,//sendStack(0x17, int folderNumber);
        //CMD_RANDOM               = 0x18,//sendStack(0x18);
        //CMD_LOOP_ENABLE          = 0x19,//sendStack(0x19, 0x00 or 0x01);
        //CMD_DAC_DISABLE          = 0x1A,//sendStack(0x1A, 0x00 or 0x01);

        CMD_RETRY                = 0x40,
        CMD_GET_STATUS           = 0x42,
        CMD_GET_VOL              = 0x43,
        CMD_GET_EQ               = 0x44,
        CMD_GET_LOOP             = 0x45,
        CMD_GET_VER              = 0x46,
        CMD_GET_COUNT_SDCARD     = 0x47,
        CMD_GET_COUNT_USB        = 0x48,
        CMD_GET_COUNT_FLASH      = 0x49,
        CMD_GET_FILE_IDX_SDCARD  = 0x4B,
        CMD_GET_FILE_IDX_USB     = 0x4C,
        CMD_GET_FILE_IDX_FLASH   = 0x4D,
        //CMD_GET_COUNT_FILES_IN_FOLDER = 0x4E,//sendStack(0x4E, folderNumber);
        //CMD_GET_COUNT_FOLDERS2   = 0x4F,//sendStack(0x4F);
        CMD_GET_FILE_POS_SEC     = 0x50,
        CMD_GET_FILE_LEN_SEC     = 0x51,
        CMD_GET_FILE_NAME        = 0x52,
        CMD_GET_COUNT_FOLDERS    = 0x53
    } CMD_TYPE;

    //* Send a command to the JQ6500 module,
    int sendCommand(const CMD_TYPE command);
    int sendCommand(const CMD_TYPE command, const byte arg1);
    int sendCommand(const CMD_TYPE command, const byte arg1, const byte arg2);
    int sendCommand(const byte *buf, bool readStatus = false);

    /** Send a command to the JQ6500 module, and get a response.
     *
     * For the query commands, the JQ6500 generally sends an integer response
     * (over the UART as 4 hexadecimal digits).
     *
     * @param command        Byte value of to send as from the datasheet.
     * @return Response from module.
     */
    unsigned int sendCommandWithResponse(const CMD_TYPE command);

    int wait(const signed long maxWaitTime = 1000);
};
