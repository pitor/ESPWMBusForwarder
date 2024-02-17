#ifndef _WMBUSPACKET_H_
#define _WMBUSPACKET_H_

#include <Arduino.h>
#include <SPI.h>
#include <Crypto.h>
#include <AES.h>
#include <CTR.h>
#include <PubSubClient.h>
#include "config.h"
#include "utils.h"

#define WMBUS_FRAME_A_PREAMBLE 0x0000
#define WMBUS_FRAME_B_PREAMBLE 0xFFFF

class WMBusPacket
{
  private:
    // 16 bit unsigned preamble word
    uint16_t preAmble = 0;

    // 8 bit unsigned sync word
    uint8_t syncWord = 0;

    // 8 bit unsigned frame control field
    uint8_t frameControl = 0;

    // 8 bit unsigned frame length
    uint8_t frameLength = 0;

    // 8 bit unsigned control field
    uint8_t controlField = 0;

    // 8 bit unsigned address field
    uint8_t addressField = 0;

    // 8 bit unsigned control information field
    uint8_t controlInformationField = 0;

    // 8 bit unsigned data field
    uint8_t dataField = 0;

    // 8 bit unsigned checksum
    uint8_t checksum = 0;

  public:
    // Constructor
    WMBusPacket();
};

#endif // _WMBUSPACKET_H_

