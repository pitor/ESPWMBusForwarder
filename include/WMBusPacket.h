#ifndef _WMBUSPACKET_H_
#define _WMBUSPACKET_H_

#include <Arduino.h>
#include "utils.h"

#define WMBUS_FRAME_A_PREAMBLE 0x54CD
#define WMBUS_FRAME_B_PREAMBLE 0x543D

enum WMBusFrameType
{
  WMBusFrameType_A = 0,
  WMBusFrameType_B = 1
};



class WMBusPacket
{
  public:
    // 16 bit unsigned preamble word
    uint16_t preAmble = 0;

    //payload buffer (max 256 bytes, 1 byte for length)
    uint8_t payload[256];
    
    // Constructor
    WMBusPacket();

    bool checkCRC();
};

#endif // _WMBUSPACKET_H_

