/**
 * @file SimpleEncryption.ino
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief Demonstrate the more low-level operations taking place during encryption of OTA data.
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

#include "APSCrypto.hpp"
using namespace ApogeoSpace::Crypto;
#include <stdint.h>

// Node personal key
static constexpr ::NodeKey_t kNodeKey{
    0x1D, 0x37, 0x10, 0x3B, 0xE0, 0xEE, 0xFC, 0x64,
    0x3B, 0xC2, 0xC6, 0x97, 0x3D, 0x1D, 0xE8, 0x09,
    0x34, 0xAD, 0xB5, 0x5E, 0x4B, 0x6A, 0x79, 0x4D,
    0x91, 0x79, 0xA1, 0xFE, 0x2F, 0x0A, 0x00, 0xB6};

// Node personal id
static constexpr ::NodeId_t kNodeId{
    0x00, 0x00, 0x00, 0x00};

/**
 * @brief Example function that can use the payload as byte array
 * 
 * @param arr payload
 */
void get_data_to_payload(uint8_t *arr)
{
    if (not arr)
        return;
    arr[0] = 0x03;
}

void setup()
{
    Serial.begin(115200);

    SecureNode mynode{kNodeId, kNodeKey};

    Timestamp_t now{
        0x00, 0x00, 0x00, 0x00};

    Payload_t payload{};
    get_data_to_payload(payload);

    Payload_t encrypted_payload{};
    Tag_t encryption_tag{};

    // Call Node::Encrypt to get the encrypted packet and its tag.
    OpStatus status = mynode.Encrypt(now, payload, encryption_tag, encrypted_payload);

    Packet_t packet{};
    // Call Node::BuildPacket instead to encrypt and insert data into a data structure that is ready to be sent.
    status = mynode.BuildPacket(now, payload, packet);

    for (const uint8_t el : packet)
    {
        if (el < 16)
        {
            Serial.print("0");
        }
        Serial.print(el, HEX);
    }
    Serial.write("\n");

    memset(payload, 0, 10);

    // Call Node::Decrypt to retrieve the original content of a payload given the encrypted version and its tag
    status = mynode.Decrypt(now, encrypted_payload, encryption_tag, payload);

    if (status != OpStatus::kSuccess)
    {
        Serial.println("Invalid mac!");
    }
    else
    {
        Serial.println("Decrypted payload:");
        for (const uint8_t el : payload)
        {
            if (el < 16)
            {
                Serial.print("0");
            }
            Serial.print(el, HEX);
        }
        for (uint8_t i = 0; i < sizeof(Payload_t); i++)
            Serial.write("\n");
    }
}

void loop()
{
}
