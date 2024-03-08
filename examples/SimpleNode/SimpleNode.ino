/**
 * @file SimpleNode.ino
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief This sketch provides the basics to fully make use of an Apogeo Space node.
 * Simply provide credentials, initialize and transmit. As easy as that!
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

void setup()
{
    Serial.begin(9600);

    // Initialize the hardware of the node.
    Serial.println("Init radio...");
    if (not node.Init())
    {
        Serial.println("failed! Please check your wiring and pin definition.");
        while (1)
            ;
    }
    Serial.println("ok!");
}

void loop()
{
    // Read a signal from a sensor connected to the analog pin 0.
    int val = analogRead(A0);

    // You can send any value or message that fits in 10 bytes
    //  with the .Send() function.
    if (not node.Send(val))
    {
        Serial.println("Error in transmission!");
    }
    else
    {
        Serial.println("Transmission successful");
    }
    delay(10000);
}
