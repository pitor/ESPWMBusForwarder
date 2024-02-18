/*
 Copyright (C) 2020 chester4444@wolke7.net
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _CC1101RECEIVER_H_
#define _CC1101RECEIVER_H_

#include <Arduino.h>
#include <SPI.h>
#include <Crypto.h>
#include <AES.h>
#include <CTR.h>
#include "config.h"
#include "utils.h"
#include "WMBusPacket.h"
#include "CC1101Constants.h"

class CC1101Receiver
{
  private:
    const uint32_t RECEIVE_TIMEOUT = 300000UL;  // in millis
    const uint32_t PACKET_TIMEOUT = 180000UL; // in seconds
    uint32_t lastPacketDecoded = -PACKET_TIMEOUT;
    uint32_t lastFrameReceived = 0;
    volatile boolean packetAvailable = false;
    inline void selectCC1101(void);
    inline void deselectCC1101(void);
    inline void waitMiso(void);
    static const uint8_t MAX_LENGTH = 64;
    CTR<AESSmall128> aes128;
    uint8_t cipher[MAX_LENGTH];
    uint8_t plaintext[MAX_LENGTH];
    uint8_t iv[16];
    bool isValid = false; // true, if meter information is valid for the last received frame
    uint8_t length = 0; // payload length
    uint32_t totalWater;
    uint32_t targetWater;
    uint32_t lastTarget=0;
    uint8_t flowTemp;
    uint8_t infoCodes;


 // reset HW and restart receiver
    void restartRadio(void);

    // flush fifo and (re)start receiver
    void startReceiver(void);

    //void writeBurstReg(uint8_t regaddr, uint8_t* buffer, uint8_t len);
    void readBurstReg(uint8_t * buffer, uint8_t regaddr, uint8_t len);
    void cmdStrobe(uint8_t cmd);
    uint8_t readReg(uint8_t regaddr, uint8_t regtype);
    uint8_t readByteFromFifo(void);
    void writeReg(uint8_t regaddr, uint8_t value);
    void initializeRegisters(void);
    void reset(void);

    // static ISR calls instanceISR via this pointer
    IRAM_ATTR static void cc1101Isr(void *p);

        // return true if a valid frame is available
    int16_t receive(WMBusPacket* packet); // read frame from CC1101
    
  public:

    // constructor
    CC1101Receiver();
  

    // startup CC1101 for receiving wmbus mode c 
    void begin();



    // must be called frequently
    bool loop(WMBusPacket* packet);

    IRAM_ATTR void instanceCC1101Isr();
};

#endif // _CC1101RECEIVER_H_
