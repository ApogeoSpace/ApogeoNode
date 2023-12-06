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

bool RFM98::Init(uint32_t frequency)
{
  SPI.begin();
  delay(100);

  // Hardware reset the module
  digitalWrite(RST_Pin, LOW);
  delay(10);
  digitalWrite(RST_Pin, HIGH);
  delay(50);

  // Set power mode to sleep and operating mode to Lora
  WriteRegister(Register::Mode, static_cast<uint8_t>(OperatingMode::kSleep) | static_cast<uint8_t>(OperatingMode::kLora));
  // Set TX FIFO address to 0x0
  WriteRegister(Register::TxFifoAddress, 0x00); // Punto a 0 il registro FiFo TX
  // Set RX FIFO address to 0x0
  WriteRegister(Register::RxFifoAddress, 0x00);
  // Put module to standby
  WriteRegister(Register::Mode, static_cast<uint8_t>(OperatingMode::kStandby));

  // Set bandwidth to 41.7 kHz, CR to 6 bit, header mode to explicit
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
  if(not WriteRegister(Register::DI0_1, 0x40, true))
  {
    return false;
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
  uint8_t status = 0;
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
  WriteRegister(Register::PayloadLen, length + 4U);
  WriteRegister(Register::Mode, static_cast<uint8_t>(OperatingMode::kTx));

  // Set DI0 to TX_DONE mode
  WriteRegister(Register::DI0_1, 0x40);

  return true;
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
  noInterrupts();
  digitalWrite(SS_Pin, LOW);
  SPI.transfer(reg_addr);
  SPI.transfer(value);
  digitalWrite(SS_Pin, HIGH);
  interrupts();

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
  noInterrupts();
  digitalWrite(SS_Pin, LOW);
  SPI.transfer(reg_addr);
  // Transfer 0x01 to read the register
  uint8_t value = SPI.transfer(0x01);
  digitalWrite(SS_Pin, HIGH);
  interrupts();
  return value;
}
