/**
 * @file APSLora.hpp
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief See APSLora.hpp
 * @version 0.1
 * @date 2023-12-02
 *
 * @copyright Copyright (C) 2023 Apogeo Space srl
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Refer to the attached LICENSE.md file for more information.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <Arduino.h>
#include <APSLora.hpp>
#include <SPI.h>

#ifndef DEBUG_PRINT_RW_REG
#define DEBUG_PRINT_RW_REG 0
#endif

using namespace ApogeoSpace::LoRa;

// The crystal oscillator frequency of the module
static constexpr double k_FX_OSC{32000000.0};

// The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
static constexpr double k_F_STEP{k_FX_OSC / 524288};

RFM98::RFM98(uint8_t rst, uint8_t ss, uint8_t d0) : RST_Pin{rst}, SS_Pin{ss}, D0_Pin{d0} ,Initialized{false}
{
  pinMode(D0_Pin, INPUT);
  pinMode(RST_Pin, OUTPUT);
  pinMode(SS_Pin, OUTPUT);
  digitalWrite(SS_Pin, HIGH);
}

bool RFM98::Init(uint32_t frequency, Mode currentMode, bool skip)
{
  if (!skip) {
    SPI.begin();
    delay(100);
  }

  // Hardware reset the module
  digitalWrite(RST_Pin, LOW);
  delay(10);
  digitalWrite(RST_Pin, HIGH);
  delay(50);

  // Set power mode to sleep and operating mode to Lora
  WriteRegister(Register::Mode, static_cast<uint8_t>(OperatingMode::kSleep) | static_cast<uint8_t>(LongRangeMode::kLora), true);
  // Set TX FIFO address to 0x0
  WriteRegister(Register::TxFifoAddress, 0x00); // Punto a 0 il registro FiFo TX
  // Set RX FIFO address to 0x0
  WriteRegister(Register::RxFifoAddress, 0x00);
  // Put module to standby
  WriteRegister(Register::Mode, static_cast<uint8_t>(OperatingMode::kStandby));

  // Set bandwidth to 41.7 kHz, CR to 4/6, header mode to explicit
  uint8_t tmp{
    static_cast<uint8_t>(BW::k41_7kHz) |
    static_cast<uint8_t>(CR::k6) |
    static_cast<uint8_t>(HeaderMode::kExplicit)
  };

  if (not WriteRegister(Register::ModemCfg1, tmp, true))
  {
    return false;
  }

  // Set spreading Factor to 128, TX mode to normal, CRC on, RX time out = 0
  tmp = 
    static_cast<uint8_t>(SF::k128) |
    static_cast<uint8_t>(TransmitMode::kNormal) |
    static_cast<uint8_t>(CRCMode::kOn);
  if (not WriteRegister(Register::ModemCfg2, tmp, true))
  {
    return false;
  }

  // Used for static node, LNA gains set by register
  if (not WriteRegister(Register::ModemCfg3, 0x00, true))
  {
    return false;
  }

  // Set preamble length to 8
  WriteRegister(Register::PreambleLenMSB, 0x00);
  WriteRegister(Register::PreambleLenLSB, 0x08);

  WriteRegister(Register::SymbTimeout, 0x64);

  // Set frequency
  uint32_t freqV = static_cast<double>(frequency) / k_F_STEP;
  uint8_t freq_msb = (freqV >> 16) & 0xFF;
  uint8_t freq_mid = (freqV >> 8) & 0xFF;
  uint8_t freq_lsb = freqV & 0xFF;

  if (not WriteRegister(Register::FrequencyMsb, freq_msb, true))
  {
    return false;
  }
  if (not WriteRegister(Register::FrequencyMid, freq_mid, true))
  {
    return false;
  }
  if (not WriteRegister(Register::FrequencyLsb, freq_lsb, true))
  {
    return false;
  }
  
  // PA_BOOST +20dBm
  if (not WriteRegister(Register::PADac, 0x07, true))
  {
    return false;
  }
  if (not WriteRegister(Register::PAConfiguration, 0x9F, true))
  {
    return false;
  }

  // Set DIO0 in TXDone mode
  if(currentMode == Mode::TxOnly){
    if(not WriteRegister(Register::DI0_1, 0x40, true))
    {
      return false;
    }
  }
  else {
    if(not WriteRegister(Register::DI0_1, 0x00, true))
    {
      return false;
    }
  }

  if(currentMode == Mode::RxOnly){
    if(not WriteRegister(Register::Mode, 0x05)) // rx continuous mode
    {
      return false;
    }
  }

  Initialized = true;
  return true;
}

bool RFM98::Transmit(const uint8_t* src, uint8_t length)
{
  if (not Initialized)
  {
    return false;
  }
  if(not src or not length)
  {
    return false;
  }
  WriteRegister(Register::Mode, static_cast<uint8_t>(OperatingMode::kStandby));
  WriteRegister(Register::SPIFifoAddress, 0x00);

  // Compatibility w/ RadioHead
#if RH_COMPATIBILITY_ENABLED 
  // Header to (broadcast)
  WriteRegister(Register::SimpleWrite, 0xff);
  // Header from (broadcast)
  WriteRegister(Register::SimpleWrite, 0xff);
  // Id (0)
  WriteRegister(Register::SimpleWrite, 0x0);
  // Flags (0)
  WriteRegister(Register::SimpleWrite, 0x0);
#endif
  noInterrupts();
  digitalWrite(SS_Pin, LOW);

  // write command
  SPI.transfer(static_cast<uint8_t>(Register::SimpleWrite)); 

  for (uint8_t idx = 0; idx < length; idx++)
  {
    SPI.transfer(src[idx]);
  }

  digitalWrite(SS_Pin, HIGH);
  interrupts();

  // Once FIFO is written, trigger the transmission
  #if RH_COMPATIBILITY_ENABLED 
  WriteRegister(Register::PayloadLen, length + 4U);
  #else
  WriteRegister(Register::PayloadLen, length);
  #endif // #if RH_COMPATIBILITY_ENABLED 

  WriteRegister(Register::Mode, static_cast<uint8_t>(OperatingMode::kTx));

  // Set DI0 to TX_DONE mode
  WriteRegister(Register::DI0_1, 0x40);

  return true;
}

bool RFM98::Receive(uint8_t* dest, uint8_t * length, int16_t *lastRssi, int8_t *lastSnr)
{
  uint8_t irq_flags = ReadRegister(Register::IrqFlags);

  if (not Initialized)
  {
    return false;
  }
  if(not dest)
  {
    return false;
  }

  if(irq_flags & 0x40){ // if interrupt flag is RxDone
    * length = ReadRegister(Register::RxNbBytes);

    // Reset the fifo read ptr to the beginning of the packet
    WriteRegister(Register::SPIFifoAddress, ReadRegister(Register::FifoRxCurrentAddr));

    ReadBurst(Register::RW, dest, *length);

	  WriteRegister(Register::IrqFlags, 0xff); // Clear all IRQ flags

    * lastSnr = (int8_t)ReadRegister(Register::PktSnrValue) / 4; //TODO: check this value
    * lastRssi = ReadRegister(Register::PktRssiValue)-137; //TODO: check this value
    return true;
  } 
  else {
    return false;
  }
}

bool RFM98::IsTxDone()
{
  return digitalRead(D0_Pin) == LOW;
}

bool RFM98::WriteRegister(const Register address, const uint8_t value, const bool verify) const
{
  uint8_t reg_addr = static_cast<uint8_t>(address);
  // Enable write bit
  reg_addr |= 0x80;

  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SS_Pin, LOW);
  SPI.transfer(reg_addr);
  SPI.transfer(value);
  digitalWrite(SS_Pin, HIGH);
  SPI.endTransaction();

#if DEBUG_PRINT_RW_REG
  Serial.print(value, HEX);
  Serial.print(" ---> ");
  Serial.println(static_cast<uint8_t>(address), HEX);
#endif

  if(not verify)
  {
    return true;
  }
  uint8_t readback_value = ReadRegister(address);
  return readback_value == value;
}

uint8_t RFM98::ReadRegister(const Register address) const
{
  uint8_t reg_addr = static_cast<uint8_t>(address);

  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SS_Pin, LOW);
  SPI.transfer(reg_addr);
  // Transfer 0x01 to read the register
  uint8_t value = SPI.transfer(0x01);
  digitalWrite(SS_Pin, HIGH);
  SPI.endTransaction();

#if DEBUG_PRINT_RW_REG
  Serial.print(value, HEX);
  Serial.print(" <--- ");
  Serial.println(static_cast<uint8_t>(address), HEX);
#endif 

  return value;
}


uint8_t RFM98::ReadBurst(const Register address, uint8_t * dest, uint8_t len) const
{
  uint8_t status = 0;
  uint8_t reg_addr = static_cast<uint8_t>(address);
  // noInterrupts();
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SS_Pin, LOW);
  status = SPI.transfer(reg_addr);
 
  while (len--){
	  *dest++ = SPI.transfer(0);
  }

  digitalWrite(SS_Pin, HIGH);
  SPI.endTransaction();
//  interrupts();
  return status;
}

///////////////////////////////
// OPERATING MODE GET/SET
///////////////////////////////

template<>
bool RFM98::isvalid(const RFM98::OperatingMode mode) const
{
  switch(mode)
  {
    case RFM98::OperatingMode::kSleep:
    case RFM98::OperatingMode::kStandby:
    case RFM98::OperatingMode::kFS_Tx:
    case RFM98::OperatingMode::kTx:
    case RFM98::OperatingMode::kFS_Rx:
    case RFM98::OperatingMode::kRxContinuous:
      return true;
    default:
      return false;
  }
}

RFM98::OperatingMode RFM98::GetOperatingMode() const
{
  return GetParam<OperatingMode, Register::Mode, 0x07>();
}

bool RFM98::SetOperatingMode(const OperatingMode mode)
{
  return SetParam<OperatingMode, Register::Mode, 0x07>(mode);
}

///////////////////////////////
// BANDWIDTH GET/SET
///////////////////////////////

template<>
bool RFM98::isvalid(const RFM98::BW mode) const
{
  switch(mode)
  {
    case RFM98::BW::k7_8kHz:
    case RFM98::BW::k10_4kHz:
    case RFM98::BW::k15_6kHz:
    case RFM98::BW::k20_8kHz:
    case RFM98::BW::k31_25kHz:
    case RFM98::BW::k41_7kHz:
    case RFM98::BW::k62_5kHz:
    case RFM98::BW::k125kHz:
    case RFM98::BW::k250kHz:
    case RFM98::BW::k500kHz:
      return true;
    default:
      return false;
  }
}

RFM98::BW RFM98::GetBW() const
{
  return GetParam<BW, Register::ModemCfg1, 0xF0>();
}

bool RFM98::SetBW(const RFM98::BW bw)
{
  return SetParam<BW, Register::ModemCfg1, 0xF0>(bw);
}

///////////////////////////////
// CR GET/SET
///////////////////////////////

template<>
bool RFM98::isvalid(const RFM98::CR cr) const
{
  switch(cr)
  {
    case RFM98::CR::k5:
    case RFM98::CR::k6:
    case RFM98::CR::k7:
    case RFM98::CR::k8:
      return true;
    default:
      return false;
  }
}

RFM98::CR RFM98::GetCR() const
{
  return GetParam<CR, Register::ModemCfg1, 0x0E>();
}

bool RFM98::SetCR(const RFM98::CR cr)
{
  return SetParam<CR, Register::ModemCfg1, 0x0E>(cr);
}

///////////////////////////////
// SF GET/SET
///////////////////////////////

template<>
bool RFM98::isvalid(const RFM98::SF sf) const
{
  switch(sf)
  {
    case RFM98::SF::k64:
    case RFM98::SF::k128:
    case RFM98::SF::k256:
    case RFM98::SF::k512:
    case RFM98::SF::k1024:
    case RFM98::SF::k2048:
    case RFM98::SF::k4096:
      return true;
    default:
      return false;
  }
}

RFM98::SF RFM98::GetSF() const
{
  return GetParam<SF, Register::ModemCfg2, 0xF0>();
}

bool RFM98::SetSF(const RFM98::SF sf)
{
  return SetParam<SF, Register::ModemCfg2, 0xF0>(sf);
}

///////////////////////////////
// HEADER MODE GET/SET
///////////////////////////////

template<>
bool RFM98::isvalid(const RFM98::HeaderMode hm) const
{
  switch(hm)
  {
    case RFM98::HeaderMode::kImplicit:
    case RFM98::HeaderMode::kExplicit:
      return true;
    default:
      return false;
  }
}

RFM98::HeaderMode RFM98::GetHeaderMode() const
{
  return GetParam<HeaderMode, Register::ModemCfg1, 0x01>();
}

bool RFM98::SetHeaderMode(const RFM98::HeaderMode hm)
{
  return SetParam<HeaderMode, Register::ModemCfg1, 0x01>(hm);
}

///////////////////////////////
// TRANSMIT MODE GET/SET
///////////////////////////////

template<>
bool RFM98::isvalid(const RFM98::TransmitMode tm) const
{
  switch(tm)
  {
    case RFM98::TransmitMode::kNormal:
    case RFM98::TransmitMode::kContinuous:
      return true;
    default:
      return false;
  }
}

RFM98::TransmitMode RFM98::GetTransmitMode() const
{
  return GetParam<TransmitMode, Register::ModemCfg2, 0x08>();
}

bool RFM98::SetTransmitMode(const RFM98::TransmitMode tm)
{
  return SetParam<TransmitMode, Register::ModemCfg2, 0x08>(tm);
}

///////////////////////////////
// CRC MODE GET/SET
///////////////////////////////

template<>
bool RFM98::isvalid(const RFM98::CRCMode cm) const
{
  switch(cm)
  {
    case RFM98::CRCMode::kOff:
    case RFM98::CRCMode::kOn:
      return true;
    default:
      return false;
  }
}

RFM98::CRCMode RFM98::GetCRCMode() const
{
  return GetParam<CRCMode, Register::ModemCfg2, 0x04>();
}

bool RFM98::SetCRCMode(const RFM98::CRCMode cm)
{
  return SetParam<CRCMode, Register::ModemCfg2, 0x04>(cm);
}

///////////////////////////////
// DEVICE MODE GET/SET
///////////////////////////////

template<>
bool RFM98::isvalid(const RFM98::DeviceMode mode) const
{
  switch(mode)
  {
    case RFM98::DeviceMode::kSleep:
    case RFM98::DeviceMode::kStandby:
    case RFM98::DeviceMode::kFsTx:
    case RFM98::DeviceMode::kTx:
    case RFM98::DeviceMode::kFsRx:
    case RFM98::DeviceMode::kRxContinuous:
    case RFM98::DeviceMode::kRxSingle:
    case RFM98::DeviceMode::kCad:
      return true;
    default:
      return false;
  }
}

RFM98::DeviceMode RFM98::GetDeviceMode() const
{
  return GetParam<DeviceMode, Register::Mode, 0x07>();
}

bool RFM98::SetDeviceMode(const RFM98::DeviceMode mode)
{
  return SetParam<DeviceMode, Register::Mode, 0x07>(mode, false);
}
