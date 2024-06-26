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

#include <string.h>

#include "APSCore.hpp"
#include "APSCrypto.hpp"
#include "APSLora.hpp"

using namespace ApogeoSpace::Core::ustd;
using type_traits::is_same, type_traits::is_pointer_type, type_traits::static_all_of;

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
                return Radio.Init(frequency, LoRa::Transmitter::Mode::TxOnly, false);
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
            bool Send(const T& value, const Crypto::Timestamp_t &now = Crypto::EmptyTimestamp)
            {
                static_assert(sizeof(T) <= sizeof(Crypto::Payload_t),
                "The value you're trying to send won't fit in a single payload!");
                static_assert(not is_same<T, const char*>::value, "Please use ::SendString when dealing with char* variables (C-style string literals)");
                static_assert(not is_same<T, char*>::value, "Please use ::SendString when dealing with char* variables (C-style strings)");
                static_assert(not is_pointer_type<T>::value, "Please use ::SendStream when dealing with non-string pointers to memory.");
                Crypto::Payload_t payload;
                memset(payload, static_cast<uint8_t>(0U), sizeof(payload));
                memcpy(payload, &value, sizeof(T));
                return Send(payload, now);
            }

            /**
             * @brief Transmit a series of values via radio. The provided values can be of any type,
             * as long as the sum of their sizes fits in the maximum allowed size for payloads.
             * Use this function to send numerical values, characters, sensor readings, etc.
             * 
             * In the resulting payload, the values will be packed (depending on platform endianness) one after the other.
             * 
             * @example
             * uint8_t a = 1;
             * char b = 'k';
             * uint32_t c = 0xDEADBEEF
             * 
             * Payload size : 10
             * Platform: Arduino R3, standard C++ compiler shipped with Arduino IDE
             * 
             * Send(a, b, c)
             * 
             * a_______________    b    c______________________
             *                |    |    |        |      |      |
             *                v    v    v        v      v      v
             * Payload :    | 1 | 'k' | 0xEF | 0xBE | 0xAD | 0xDE | 0 | 0 | 0 | 0 |
             *                0                                                    9
             * @tparam Ts 
             * @param args 
             * @param now 
             */
            template<class ... Ts>
            static void Pack(Crypto::Payload_t& pl, const Ts&... args)
            {
                static_assert(
                    (sizeof(args) + ... + 0)<=sizeof(Crypto::Payload_t), 
                    "Provided arguments exceed the available space for the packet"
                );
                static_assert(
                    static_all_of<not is_same<Ts, const char*>::value...>::value, 
                    "Please use ::SendString when dealing with char* variables (C-style string literals)"
                );
                static_assert(
                    static_all_of<not is_same<Ts, char*>::value...>::value, 
                    "Please use ::SendString when dealing with char* variables (C-style strings)"
                );
                static_assert(
                    static_all_of<not is_pointer_type<Ts>::value...>::value, 
                    "Please use ::SendStream when dealing with non-string pointers to memory."
                );

                memset(pl, 0U, sizeof(pl));
                
                size_t offset = 0;
                pack_args_impl(pl, offset, args...);
            }

            /**
             * @brief Transmit a stream of bytes via radio.
             * 
             * @param src data to be sent (pointer to externally allocated memory)
             * @param len length of the transmission. Values above the maximum size of the payload will result in a failure!
             * @param now current UTC timestamp. See documentation for the Payload_t version of Send().
             * @return true transmission succeeded.
             * @return false transmission failed. Most likely a missing initialization, or specified length was above payload limits
             */
            bool SendStream(const void * src, const uint8_t len, const Crypto::Timestamp_t &now = Crypto::EmptyTimestamp)
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

            /**
             * @brief Send a C-style string literal as a payload. 
             * @note The string should be NULL terminated, as this function relies on strlen() to determine the size of the string.
             * 
             * @param src string pointer
             * @param auto_trim control whether to clip the string content to available payload size
             * false: if the content of the string (determined by strlen) exceeds the available number of bytes of the payload,
             * this function won't attempt to trim and send the string anyway, and will instead return an error.
             * true: if the condition above applies, then the string content is trimmed to allowable length and sent. Note that this may
             * naturally lead to loss of information!
             * @note The presence of the NULL terminator within the payload is _not_ guaranteed, when @p auto_trim is set to true!
             * @param now current UTC timestamp. See documentation for the Payload_t version of Send().
             * @return true The string (or a part thereof, depending on length and @p auto_trim) was sent successfully.
             * false The transmission failed, possibly due to the following reasons:
             * - provided data was invalid (NULL, or len = 0)
             * - size of string exceeded available space and @p auto_trim wasn't enabled
             * - an error occured in ::Send
             * 
             * @code {.c++}
                // Suppose the Payload type only allows for 10 bytes to be sent
                char mystr[20]{};
                strcpy(mystr, "Hello world!");
                            * 
                // With no autotrim
                node.SendString(mystr, false, ts); // -> returns false
                            * 
                // With autotrim
                node.SendString(mystr, true, ts); // -> returns true, but the content copied in the payload is 'Hello worl' (_without_ NULL terminator)
             * @endcode
             * 
             */
            bool SendString(const char * const src, bool auto_trim, const Crypto::Timestamp_t &now = Crypto::EmptyTimestamp)
            {
                if(not src)
                {
                    return false;
                }

                // Actual number of bytes to copy over from the string to also include the NULL terminator
                auto src_len_with_null{strlen(src) + 1};
                if(src_len_with_null == 0)
                {
                    return false;
                }

                if(not auto_trim and src_len_with_null > sizeof(Crypto::Payload_t))
                {
                    // Original string or its terminator won't fit in the payload
                    return false;
                }
                auto to_copy = min(src_len_with_null, sizeof(Crypto::Payload_t));
                Crypto::Payload_t payload{};
                strncpy(reinterpret_cast<char*>(payload), src, to_copy);

                return Send(payload, now);
            }

            /**
             * @brief Send a C-style string as a payload. 
             * @note The string should be NULL terminated, as this function relies on strlen() to determine the size of the string.
             * @note The presence of the NULL terminator within the payload is _not_ guaranteed!
             * @param src string pointer
             * @param auto_trim control whether to clip the string content to available payload size
             * false: if the content of the string (determined by strlen) exceeds the available number of bytes of the payload,
             * this function won't attempt to trim and send the string anyway, and will instead return an error.
             * true: if the condition above applies, then the string content is trimmed to allowable length and sent. Note that this may
             * naturally lead to loss of information!
             * @note The presence of the NULL terminator within the payload is _not_ guaranteed, when @p auto_trim is set to true!
             * @param now current UTC timestamp. See documentation for the Payload_t version of Send().
             * @return true The string (or a part thereof, depending on length and @p auto_trim) was sent successfully. 
             * @return false The transmission failed, possibly due to the following reasons:
             * - provided data was invalid (NULL, or len = 0)
             * - size of string exceeded available space and @p auto_trim wasn't enabled
             * - an error occured in ::Send
             */
            bool SendString(char * const src, bool auto_trim, const Crypto::Timestamp_t &now = Crypto::EmptyTimestamp)
            {
                return SendString(const_cast<const char *>(src), auto_trim, now);
            }

            /**
             * @brief Get a reference to the internal radio object.
             * 
             * @return LoRa::RFM98& 
             */
            LoRa::RFM98 & GetRadio()
            {return Radio;}

        private:

            /**
             * @brief Helper function for :: Pack to update the offset to copy 
             * a given argument into  the target buffer 
             * 
             * @tparam T argument type
             * @param offset reference to offset
             */
            template<typename T>
            static void add_offset(size_t& offset, const T&) {
                offset += sizeof(T);
            }

            /**
             * @brief Recursion base for the pack_args_impl function
             * 
             * @param buff 
             * @param offset 
             */
            static void pack_args_impl(uint8_t* buff, size_t& offset) {
                // do nothing
            }

            template<typename T, typename... Args>
            static void pack_args_impl(uint8_t* buff, size_t& offset, const T& arg, const Args&... args) {
                memcpy(buff + offset, &arg, sizeof(T));
                add_offset(offset, arg);
                pack_args_impl(buff, offset, args...);
            }

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
