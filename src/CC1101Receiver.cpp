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

#include "CC1101Receiver.h"

CC1101Receiver::CC1101Receiver()
{
}


// ChipSelect assert
inline void CC1101Receiver::selectCC1101(void)
{
  digitalWrite(SS, LOW);
}

// ChipSelect deassert
inline void CC1101Receiver::deselectCC1101(void)
{
  digitalWrite(SS, HIGH);
}

// wait for MISO pulling down
inline void CC1101Receiver::waitMiso(void)
{
  while (digitalRead(MISO) == HIGH)
    ;
}

// write a single register of CC1101
void CC1101Receiver::writeReg(uint8_t regAddr, uint8_t value)
{
  selectCC1101();
  waitMiso();            // Wait until MISO goes low
  SPI.transfer(regAddr); // Send register address
  SPI.transfer(value);   // Send value
  deselectCC1101();
}

// send a strobe command to CC1101
void CC1101Receiver::cmdStrobe(uint8_t cmd)
{
  selectCC1101();
  delayMicroseconds(5);
  waitMiso();        // Wait until MISO goes low
  SPI.transfer(cmd); // Send strobe command
  delayMicroseconds(5);
  deselectCC1101();
}

// read CC1101 register (status or configuration)
uint8_t CC1101Receiver::readReg(uint8_t regAddr, uint8_t regType)
{
  uint8_t addr, val;

  addr = regAddr | regType;
  selectCC1101();
  waitMiso();               // Wait until MISO goes low
  SPI.transfer(addr);       // Send register address
  val = SPI.transfer(0x00); // Read result
  deselectCC1101();

  return val;
}

//
void CC1101Receiver::readBurstReg(uint8_t *buffer, uint8_t regAddr, uint8_t len)
{
  uint8_t addr, i;

  addr = regAddr | READ_BURST;
  selectCC1101();
  delayMicroseconds(5);
  waitMiso();         // Wait until MISO goes low
  SPI.transfer(addr); // Send register address
  for (i = 0; i < len; i++)
    buffer[i] = SPI.transfer(0x00); // Read result byte by byte
  delayMicroseconds(2);
  deselectCC1101();
}

// power on reset
void CC1101Receiver::reset(void)
{
  deselectCC1101();
  delayMicroseconds(3);

  digitalWrite(MOSI, LOW);
  digitalWrite(SCK, HIGH); // see CC1101 datasheet 11.3

  selectCC1101();
  delayMicroseconds(3);
  deselectCC1101();
  delayMicroseconds(45); // at least 40 us

  selectCC1101();

  waitMiso();                // Wait until MISO goes low
  SPI.transfer(CC1101_SRES); // Send reset command strobe
  waitMiso();                // Wait until MISO goes low

  deselectCC1101();
}

// set IDLE state, flush FIFO and (re)start receiver
void CC1101Receiver::startReceiver(void)
{
  uint8_t regCount = 0;
  cmdStrobe(CC1101_SIDLE); // Enter IDLE state
  while (readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) != MARCSTATE_IDLE)
  {
    if (regCount++ > 100)
    {
      Serial.println("Enter idle state failed!\n");
      restartRadio();
    }
  }

  cmdStrobe(CC1101_SFRX); // flush receive queue
  delay(5);

  regCount = 0;
  cmdStrobe(CC1101_SRX); // Enter RX state
  delay(10);
  while (readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) != MARCSTATE_RX)
  {
    if (regCount++ > 100)
    {
      Serial.println("Enter RX state failed!\n");
      restartRadio();
    }
  }
}

// initialize all the CC1101 registers
void CC1101Receiver::initializeRegisters(void)
{
  writeReg(CC1101_IOCFG2, CC1101_DEFVAL_IOCFG2);
  writeReg(CC1101_IOCFG0, CC1101_DEFVAL_IOCFG0);
  writeReg(CC1101_FIFOTHR, CC1101_DEFVAL_FIFOTHR);
  writeReg(CC1101_PKTLEN, CC1101_DEFVAL_PKTLEN);
  writeReg(CC1101_PKTCTRL1, CC1101_DEFVAL_PKTCTRL1);
  writeReg(CC1101_PKTCTRL0, CC1101_DEFVAL_PKTCTRL0);
  writeReg(CC1101_SYNC1, CC1101_DEFVAL_SYNC1);
  writeReg(CC1101_SYNC0, CC1101_DEFVAL_SYNC0);
  writeReg(CC1101_ADDR, CC1101_DEFVAL_ADDR);
  writeReg(CC1101_CHANNR, CC1101_DEFVAL_CHANNR);
  writeReg(CC1101_FSCTRL1, CC1101_DEFVAL_FSCTRL1);
  writeReg(CC1101_FSCTRL0, CC1101_DEFVAL_FSCTRL0);
  writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2);
  writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1);
  writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0);
  writeReg(CC1101_MDMCFG4, CC1101_DEFVAL_MDMCFG4);
  writeReg(CC1101_MDMCFG3, CC1101_DEFVAL_MDMCFG3);
  writeReg(CC1101_MDMCFG2, CC1101_DEFVAL_MDMCFG2);
  writeReg(CC1101_MDMCFG1, CC1101_DEFVAL_MDMCFG1);
  writeReg(CC1101_MDMCFG0, CC1101_DEFVAL_MDMCFG0);
  writeReg(CC1101_DEVIATN, CC1101_DEFVAL_DEVIATN);
  writeReg(CC1101_MCSM1, CC1101_DEFVAL_MCSM1);
  writeReg(CC1101_MCSM0, CC1101_DEFVAL_MCSM0);
  writeReg(CC1101_FOCCFG, CC1101_DEFVAL_FOCCFG);
  writeReg(CC1101_BSCFG, CC1101_DEFVAL_BSCFG);
  writeReg(CC1101_AGCCTRL2, CC1101_DEFVAL_AGCCTRL2);
  writeReg(CC1101_AGCCTRL1, CC1101_DEFVAL_AGCCTRL1);
  writeReg(CC1101_AGCCTRL0, CC1101_DEFVAL_AGCCTRL0);
  writeReg(CC1101_FREND1, CC1101_DEFVAL_FREND1);
  writeReg(CC1101_FREND0, CC1101_DEFVAL_FREND0);
  writeReg(CC1101_FSCAL3, CC1101_DEFVAL_FSCAL3);
  writeReg(CC1101_FSCAL2, CC1101_DEFVAL_FSCAL2);
  writeReg(CC1101_FSCAL1, CC1101_DEFVAL_FSCAL1);
  writeReg(CC1101_FSCAL0, CC1101_DEFVAL_FSCAL0);
  writeReg(CC1101_FSTEST, CC1101_DEFVAL_FSTEST);
  writeReg(CC1101_TEST2, CC1101_DEFVAL_TEST2);
  writeReg(CC1101_TEST1, CC1101_DEFVAL_TEST1);
  writeReg(CC1101_TEST0, CC1101_DEFVAL_TEST0);
}

IRAM_ATTR void CC1101Receiver::instanceCC1101Isr()
{
  // set the flag that a package is available
  packetAvailable = true;
}

// static ISR method, that calls the right instance
IRAM_ATTR void CC1101Receiver::cc1101Isr(void *p)
{
  CC1101Receiver *ptr = (CC1101Receiver *)p;
  ptr->instanceCC1101Isr();
}

// should be called frequently, handles the ISR flag
// does the frame checkin and decryption
bool CC1101Receiver::loop(WMBusPacket* packet)
{

  bool packetReceived = false;

  if (packetAvailable)
  {
    // Serial.println("packet received");
    //  Disable wireless reception interrupt
    detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));

    // clear the flag
    packetAvailable = false;

    packetReceived = receive(packet);

    // Enable wireless reception interrupt
    attachInterruptArg(digitalPinToInterrupt(CC1101_GDO0), cc1101Isr, this, FALLING);
  }

  if (millis() - lastFrameReceived > RECEIVE_TIMEOUT)
  {
    Serial.println("CC1101Receiver: no frame received for a long time, restarting radio");
    // workaround: reset CC1101, since it stops receiving from time to time
    restartRadio();
  }

  return packetReceived;
}

// Initialize CC1101 to receive WMBus MODE C1
void CC1101Receiver::begin()
{
  pinMode(SS, OUTPUT);         // SS Pin -> Output
  SPI.begin();                 // Initialize SPI interface
  pinMode(CC1101_GDO0, INPUT); // Config GDO0 as input

  restartRadio();
  attachInterruptArg(digitalPinToInterrupt(CC1101_GDO0), cc1101Isr, this, FALLING);
  lastFrameReceived = millis();
}

void CC1101Receiver::restartRadio()
{
  Serial.println("resetting CC1101");

  reset(); // power on CC1101

  // Serial.println("Setting CC1101 registers");
  initializeRegisters(); // init CC1101 registers

  cmdStrobe(CC1101_SCAL);
  delay(1);

  startReceiver();
  lastFrameReceived = millis();
}


// reads a single byte from the RX fifo
uint8_t CC1101Receiver::readByteFromFifo(void)
{
  return readReg(CC1101_RXFIFO, CC1101_CONFIG_REGISTER);
}

// handles a received frame and restart the CC1101 receiver
int16_t CC1101Receiver::receive(WMBusPacket *packet)
{
  // read preamble, should be 0x543D
  uint8_t p1 = readByteFromFifo();
  uint8_t p2 = readByteFromFifo();
  bool frameOk = false;
  
#if DEBUG
  Serial.printf("PREAMBLE %02x%02x\n", p1, p2);
#endif
    uint16_t bufferIx = 0;

    packet->preAmble = p1 << 8 | p2;
 
  // is it Mode C1, frame B and does it fit in the buffer
  if ( packet->preAmble == WMBUS_FRAME_B_PREAMBLE)
  {
    length = readByteFromFifo();

    packet->payload[bufferIx++] = length;

    // starting with 1! index 0 is lfield
    for (int i = 0; i < length; i++)
    {
      packet->payload[bufferIx++] = readByteFromFifo();
    }

    lastFrameReceived = millis();

    // check meterId, CRC. Skip the preamble bytes.
    if (packet->checkCRC() == true)
    {
      frameOk = true;
    }
    else
    {
      return -1;
    }
    // received packet is ok
    lastPacketDecoded = millis();
    return bufferIx;
  }

  // flush RX fifo and restart receiver
  startReceiver();
  // Serial.printf("rxStatus: 0x%02x\n\r", readStatusReg(CC1101_RXBYTES));

  return frameOk;
}