#include "WMBusPacket.h"
#include "utils.h"

WMBusPacket::WMBusPacket()
{
}

bool WMBusPacket::checkCRC()
{
  uint8_t payloadLength = payload[0];
  Serial.printf("Checking frame length %d\n", payloadLength);
  for (int i = 0; i <= payloadLength; i++)
  {
    Serial.printf("%02X", payload[i]);
  }
  Serial.println();


  uint16_t crc = crcEN13575(payload, payloadLength - 1); // -2 (CRC) + 1 (L-field)
  uint16_t expectedCrc = payload[payloadLength - 1] << 8 | payload[payloadLength];
  if (crc != expectedCrc)
  {
    Serial.println("CRC Error");
    Serial.printf("%04x - %02x%02x\n", crc, payload[payloadLength - 1], payload[payloadLength]);
    return false;
  }
  else {
    Serial.println("CRC OK");
  }

  return true;
}

