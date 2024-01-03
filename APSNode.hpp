/**
 * @file APSNode.hpp
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief Basic functionality to send data within the Apogeo Space network.
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

#ifndef AS_NODE_ARDUINO_HPP
#define AS_NODE_ARDUINO_HPP

#include "APSCrypto.hpp"
#include "APSLora.hpp"
#include <string.h>

namespace ApogeoSpace
{
    namespace Node
    {
        /**
         * @brief High level representation of a node connected to the APS network.
         * This class effectively contains the necessary information to encrypt data
         * for secure OTA transmission and takes care of sending it via the default
         * transmitter interface, which corresponds to the HopeRF RFM98 module
         * found in the hardware sold by Apogeo Space.
         * As this library stands, only a minimal freedom in configuration is provided
         * by using the APSNode.hpp interface, which allows e.g. to switch the receiver pins
         * to avoid conflicts with additional hardware, or to input a different frequency of operation.
         * 
         * To use other modules you could take a look at how the APSLora interface is implemented.
         */
        class Node
        {
        public:
            /**
             * @brief Construct a new Node object. If you don't need any reconfiguration in hardware,
             * simply input the node id code and the secure key as provided by Apogeo Space.
             * 
             * @param id node identifier
             * @param key secure key for encryption
             * @param rst hardware reset pin of the radio module
             * @param ss SPI slave select pin of the radio module
             * @param d0 "TX DONE" pin of the radio module
             */
            Node(
                const Crypto::NodeId_t &id, const Crypto::NodeKey_t &key,
                uint8_t rst = LoRa::RFM98::kDefault_RST_Pin,
                uint8_t ss = LoRa::RFM98::kDefault_SS_Pin,
                uint8_t d0 = LoRa::RFM98::kDefault_D0_Pin)
                : SecureNode{id, key}, Radio{rst, ss, d0}
            {
            }

            /**
             * @brief Initialize the underlying hardware (i.e., the transmitter)
             * Leave frequency as default unless instructed otherwise!
             * @param frequency center carrier frequency (in Hz)
             * @return true radio initialization was successful.
             * @return false hw initialization failed.
             * Common causes of failures include incorrect wiring or electrical conflicts with other hw.
             */
            bool Init(uint32_t frequency = LoRa::kDefaultFrequency)
            {
                return Radio.Init(frequency);
            }

            /**
             * @brief Transmit a payload via radio. This function is best suited when
             * you wish to input a custom message, or if you're using other functions from the
             * APS library that make use of the Payload_t type.
             * @param payload byte buffer of known size. Current version defines this size to be 10.
             * @param now current UTC timestamp, used for encrypting packets before sending them
             * You can actually leave this parameter empty, so that the timestamp will be simply 0.
             * Future versions of this library might require a valid timestamp to be provided to this function.
             * In any case, the timestamp is a 32 bits number split in a 4 byte buffer, enclosed in the Timestamp_t type
             * @return true transmission succeeded.
             * @return false transmission failed. Causes include a missing initialization (call to Init()),
             * or invalid data
             */
            bool Send(
                const Crypto::Payload_t &payload,
                const Crypto::Timestamp_t &now = Crypto::EmptyTimestamp)
            {
                Crypto::Packet_t packet;
                SecureNode.BuildPacket(now, payload, packet);
                return Radio.Transmit(packet, sizeof(packet));
            }

            /**
             * @brief Transmit a value via radio. This value can be of any type,
             * as long as it fits in the maximum allowed size for payloads.
             * Use this function to send numerical values, characters, sensor readings, etc.
             * 
             * @tparam T 
             * @param value data to send
             * @param now current UTC timestamp. See documentation for the Payload_t version of Send().
             * @return true transmission succeeded.
             * @return false transmission failed. Most likely a missing initialization
             */
            template<typename T>
            bool Send(T value, const Crypto::Timestamp_t &now = Crypto::EmptyTimestamp)
            {
                static_assert(sizeof(T) <= sizeof(Crypto::Payload_t),
                "The value you're trying to send won't fit in a single payload!");
                Crypto::Payload_t payload;
                memset(payload, static_cast<uint8_t>(0U), sizeof(payload));
                memcpy(payload, &value, sizeof(T));
                return Send(payload, now);
            }

            /**
             * @brief Transmit a stream of bytes via radio.
             * 
             * @param src data to be sent (pointer to externally allocated memory)
             * @param len length of the transmission. Values above the maximum size of the payload will result in a failure!
             * @param now 
             * @param now current UTC timestamp. See documentation for the Payload_t version of Send().
             * @return true transmission succeeded.
             * @return false transmission failed. Most likely a missing initialization, or specified length was above payload limits
             */
            bool Send(const void * src, const uint8_t len, const Crypto::Timestamp_t &now = Crypto::EmptyTimestamp)
            {
                if(len>sizeof(Crypto::Payload_t))
                {
                    return false;
                }
                Crypto::Payload_t payload;
                memset(payload, static_cast<uint8_t>(0U), sizeof(payload));
                memcpy(payload, src, len);
                return Send(payload, now);
            }

             LoRa::RFM98 & GetRadio()
             {return Radio;}

        private:
            class Crypto::SecureNode SecureNode;
            LoRa::RFM98 Radio;
        };

    }
}

// Exported types
using APSNode = ApogeoSpace::Node::Node;
using NodeId = ApogeoSpace::Crypto::NodeId_t;
using NodeKey = ApogeoSpace::Crypto::NodeKey_t;

using TimeStamp = ApogeoSpace::Crypto::Timestamp_t;
using Payload = ApogeoSpace::Crypto::Payload_t;
using Packet = ApogeoSpace::Crypto::Packet_t;

#endif
