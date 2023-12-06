/**
 * @file APSCrypto.hpp
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief Cryptography library for sending data securely within the Apogeo Space network.
 * All functionality that is implemented here is fairly low-level, so you use it at your own risk:
 * an incorrect usage might result in missed reception of transmitted data.
 * @version 0.1
 * @date 2023-11-20
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

#ifndef AS_CRYPTO_ARDUINO_HPP
#define AS_CRYPTO_ARDUINO_HPP
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "AES.h"

namespace ApogeoSpace
{
    namespace Crypto
    {
        /* Packet structure
            | PVN | HEADER    | LEN | PAYLOAD | TAG |
            | --- | ID | Time | --- | ------- | --- |
            | 3   | 4  | 4    | 1   | 10      | 8   |

        */

        using NodeKey_t = uint8_t[32];

        using PVN_t = uint8_t[3];
        using Header_t = uint8_t[8];
        using NodeId_t = uint8_t[4];
        using Timestamp_t = uint8_t[4];
        constexpr Timestamp_t EmptyTimestamp{0x00,0x00,0x00,0x00};

        using Payload_t = uint8_t[10];
        using Tag_t = uint8_t[8];

        using Packet_t = uint8_t[sizeof(PVN_t) + sizeof(Header_t) + 1 + sizeof(Payload_t) + sizeof(Tag_t)];
        //static_assert(sizeof(Packet_t) == 30U);

        struct __attribute__((packed)) PacketStruct_t
        {
            PVN_t PVN;
            NodeId_t Id;
            Timestamp_t Ts;
            uint8_t PayloadLen;
            Payload_t Payload;
            Tag_t Tag;
        };

        /**
         * @brief Exit status for the cryptography operations
         *
         */
        enum class OpStatus : uint8_t
        {
            /** No errors encountered, operation successful */
            kSuccess = 0,
            /** The given data is not valid */
            kInvalidInput = 1,
            /** MAC check failed */
            kInvalidMAC = 2,
        };

        /**
         * @brief Represent the cryptography core for a given node. It holds the node ID and the
         * node-locked key for encryption.
         */
        class SecureNode
        {
        public:
            /**
             * @brief Construct a new Node object
             *
             * @param _id Node ID, given as a byte array
             * @param _key 32-byte cryptographic key. It must correspond to the one given
             * for this specific node to connect to the Apogeo Space network.
             */
            SecureNode(const NodeId_t &_id, const NodeKey_t &_key)
            {
                memcpy(Id, _id, sizeof(NodeId_t));
                memcpy(Key, _key, sizeof(NodeKey_t));
            }

            SecureNode(const SecureNode &rhs) = delete;
            SecureNode &operator=(const SecureNode &rhs) = delete;
            SecureNode(SecureNode &&rhs) = delete;
            SecureNode &operator=(SecureNode &&rhs) = delete;

            /**
             * @brief Encrypt a payload with the node key.
             * This encryption method uses an UTC timestamp representing the current time
             * so that the network can ensure that received data is never stale.
             *
             * @param now UTC timestamp represented as a 4 bytes array. Unless otherwise specified
             * by the network provider, you can just pass an array of zeros.
             * @param payload The payload you wish to encrypt. If the actual content to be sent occupies
             * less than 10 bytes, you can just leave the remaining space to 0.
             * @param tag The tag resulting from the encryption operation.
             * @param encrypted_payload Resulting encrypted payload, which can be sent to the network
             * @note This method works at a lower level than the one normally used for simply building a packet
             * ready to be sent given some input data. For this reason, you should normally use ::BuildPacket
             * @return OpStatus kSuccess if no errors were encountered.
             */
            OpStatus Encrypt(
                const Timestamp_t &now,
                const Payload_t &payload,
                Tag_t &tag,
                Payload_t &encrypted_payload);

            /**
             * @brief Decrypt a payload with the node key.
             * @param now UTC timestamp represented as a 4 bytes array. Unless otherwise specified
             * by the network provider, you can just pass an array of zeros.
             * @param encrypted_payload The payload to be decrypted.
             * @param tag The tag to be used for decrypting the packet, which resulted from the previous
             * encryption operation.
             * @param payload Resulting plaintext payload
             * @return OpStatus
             * @retval kSuccess if no errors were encountered.
             * @retval kInvalidMAC if the packet could not be decrypted.
             */
            OpStatus Decrypt(
                const Timestamp_t &now,
                const Payload_t &encrypted_payload,
                const Tag_t &tag,
                Payload_t &payload);

            /**
             * @brief Build a packet ready to be sent. Use of this function should be preferred
             * w.r.t. ::Encrypt, as it doesn't require the user to retrieve the encrypted packet and/or the tag.
             *
             * @param now UTC timestamp represented as a 4 bytes array. Unless otherwise specified
             * by the network provider, you can just pass an array of zeros.
             * @param payload The payload you wish to encrypt. If the actual content to be sent occupies
             * less than 10 bytes, you can just leave the remaining space to 0.
             * @param packet_to_build Buffer of size equal to Packet_t, which will eventually
             * hold all the necessary data for successfully transmitting your payload to the AS network.
             * @retval kSuccess if no errors were encountered.
             */
            OpStatus BuildPacket(
                const Timestamp_t &now,
                const Payload_t &payload,
                Packet_t &packet_to_build);

            /**
             * @brief Extract a plaintext payload from a packet.
             * @param now UTC timestamp represented as a 4 bytes array. Unless otherwise specified
             * by the network provider, you can just pass an array of zeros.
             * @param packet_to_disassemble Contains the encrypted payload and other data necessary to decrypt the payload
             * @param payload Plaintext payload resulting from the decryption operation
             * @return OpStatus
             * @retval kSuccess if no errors were encountered.
             * @retval kInvalidMAC if the packet could not be decrypted.
             */
            OpStatus ExtractFromPacket(
                const Timestamp_t &now,
                const Packet_t &packet_to_disassemble,
                Payload_t &payload);

        private:
            /** Node personal ID. */
            NodeId_t Id;
            /** Node personal key. */
            NodeKey_t Key;
        };

        /**
         * @brief This is an adapted version of the original AES_CMAC from Piotr Obst, adapter for usage
         * in conjunction with AES-256. Please refer to the original library's documentation
         * for further information, as changes made here are minimal and aimed to keep this
         * version as close as possible as the original, and thusdo not affect its high level
         * functionality in any sensible way.
         *
         */
        class AES_CMAC
        {
        public:
            explicit AES_CMAC(AESTiny256 &aes256);

        public:
            void generateMAC(uint8_t *mac, const uint8_t *key, const uint8_t *data, size_t dataLen);

        private:
            void shiftLeft(uint8_t *buff, uint8_t buffLen);
            void xor128(uint8_t *out, const uint8_t *a, const uint8_t *b);
            void padding(uint8_t *pad, const uint8_t *lastb, int len);

        private:
            AESTiny256 &aes256;
            uint8_t X[16];
            uint8_t Y[16];
        };
    }
}

#endif // AS_CRYPTO_ARDUINO_HPP
