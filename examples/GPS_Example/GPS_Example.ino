/**
 * @file GPS_Example.ino
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief Simple sketch to demonstrate the usage of an Apogeo Space node transmitter
 * with a GNSS receiver attached.
 * The hardware used for the demonstration is a generic u-blox NEO-8M module communicating
 * via UART @ 9600 baud, connected as below.
 * --------            ---------------
 * |    TX|____________|RX (7)       |
 * | GPS  |            |       Node  |
 * |    RX|____________|TX (8)       |
 * --------            ---------------
 * Once the receiver has a fix, it'll start parsing latitude and longitude data
 * and send them every 20". The UTC timestamp retrieved from the GPS data is also
 * fed to the transmission process to correctly encrypt the data.
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

#include <SoftwareSerial.h>
#include "src/GNSS.hpp"

#include "APSNode.hpp"
// Node personal id. Must be equal to the hexadecimal string
//  provided by ApogeoSpace.
const NodeId id{
    0x4E, 0x4F, 0x44, 0x45};

// Node personal key. Must be equal to the hexadecimal string
//  provided by ApogeoSpace.
const NodeKey key{
    0x1D, 0x37, 0x10, 0x3B, 0xE0, 0xEE, 0xFC, 0x64,
    0x3B, 0xC2, 0xC6, 0x97, 0x3D, 0x1D, 0xE8, 0x09,
    0x34, 0xAD, 0xB5, 0x5E, 0x4B, 0x6A, 0x79, 0x4D,
    0x91, 0x79, 0xA1, 0xFE, 0x2F, 0x0A, 0x00, 0xB6};

// Create a node with the above credentials.
APSNode node(id, key);

// Create a GPS receiver connected to pin 7 (RX) and 8 (TX)
SoftwareSerial GPS_Serial(7, 8);
APS_GNSS gnss(GPS_Serial);

// Initialize the hardware (serial, radio etc.)
void setup()
{
  Serial.begin(9600);
  GPS_Serial.begin(9600);

  // Initialize the radio
  if (not node.Init())
  {
    Serial.println("Transmitter initialization failed!");
    while (1)
      ;
  }
  else
  {
    Serial.println("Transmitter initialization ok!");
  }
}

static constexpr uint64_t kTransmissionPeriodMs{20000};
static constexpr uint64_t kReportPeriodMs{1000};

void loop()
{
  static uint64_t tx_time_counter{0};
  static uint64_t report_time_counter{0};
  gnss.Process();

  if (gnss.IsFixAvailable())
  {
    if (millis() - report_time_counter > kReportPeriodMs)
    {
      Serial.print("Fix ok: Lat ");
      Serial.print(gnss.GetLatitude());
      Serial.print(" ° ");
      Serial.print("Lon ");
      Serial.print(gnss.GetLongitude());
      Serial.println(" °");
      report_time_counter = millis();
    }

    if ((millis() - tx_time_counter > kTransmissionPeriodMs))
    {
      Serial.println("Transmission.");
      TimeStamp now{
          0x00, 0x00, 0x00, 0x00};
      Payload payload{};
      gnss.FillPayload(payload, now);
      node.Send(payload, now);

      tx_time_counter = millis();
    }
  }
  else
  {
    if (millis() - report_time_counter > kReportPeriodMs)
    {
      Serial.println("No fix");
      report_time_counter = millis();
    }
  }
}
