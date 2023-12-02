/**
 * @file APSCrypto.cpp
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief See APSCrypto.hpp for more information.
 * @version 0.1
 * @date 2023-11-20
 *
 * @copyright Copyright (c) Apogeo Space 2023 TODO:put license details
 */

#include "APSCrypto.hpp"
#include "AES.h"
#include "CTR.h"
#include <string.h>

using namespace ApogeoSpace;
static constexpr Crypto::PVN_t kPVN{0x0};
static constexpr uint8_t kPayloadLength{10U};

Crypto::OpStatus Crypto::SecureNode::Encrypt(
    const Timestamp_t &now,
    const Payload_t &payload,
    Tag_t &tag,
    Payload_t &encrypted_payload)
{
    AESTiny256 aes256;
    Crypto::AES_CMAC cmac(aes256);
    uint8_t mac[16];

    uint8_t data_raw[sizeof(Header_t) * sizeof(Payload_t)]{0x0};

    memcpy(data_raw, Id, sizeof(NodeId_t));
    memcpy(data_raw + sizeof(NodeId_t), now, sizeof(Timestamp_t));
    memcpy(data_raw + sizeof(Header_t), payload, sizeof(Payload_t));
    cmac.generateMAC(mac, Key, data_raw, sizeof(data_raw));

    memcpy(tag, mac, sizeof(Tag_t));
    memset(mac + sizeof(Tag_t), 0, sizeof(mac) - sizeof(Tag_t));

    CTR<AESTiny256> ctraes256;

    ctraes256.setKey(Key, ctraes256.keySize());
    ctraes256.setIV(mac, 16);
    ctraes256.setCounterSize(8);
    ctraes256.encrypt(encrypted_payload, payload, 10);

    return OpStatus::kSuccess;
}

Crypto::OpStatus Crypto::SecureNode::Decrypt(
    const Timestamp_t &now,
    const Payload_t &encrypted_payload,
    const Tag_t &tag,
    Payload_t &payload)
{
    AESTiny256 aes256;
    Crypto::AES_CMAC cmac(aes256);
    uint8_t mac[16];
    uint8_t verify_mac[16];

    memcpy(mac, tag, sizeof(Tag_t));
    memset(mac + sizeof(Tag_t), 0, sizeof(mac) - sizeof(Tag_t));

    CTR<AESTiny256> ctraes256;

    ctraes256.setKey(Key, ctraes256.keySize());
    ctraes256.setIV(mac, 16);
    ctraes256.setCounterSize(8);
    ctraes256.decrypt(payload, encrypted_payload, sizeof(Payload_t));

    // | HEADER        | PAYLOAD       ... |
    // | ID    | Time  |               ... |
    // | | | | | | | | | | |           ... |
    // 0       4       8                  18
    uint8_t data_raw[sizeof(Header_t) * sizeof(Payload_t)]{0x0};

    memcpy(data_raw, Id, sizeof(NodeId_t));
    memcpy(data_raw + sizeof(NodeId_t), now, sizeof(Timestamp_t));
    memcpy(data_raw + sizeof(Header_t), payload, sizeof(Payload_t));
    cmac.generateMAC(verify_mac, Key, data_raw, sizeof(data_raw));

    if (memcmp(mac, verify_mac, 8) != 0)
    {
        return OpStatus::kInvalidMAC;
    }

    return OpStatus::kSuccess;
}

Crypto::OpStatus Crypto::SecureNode::BuildPacket(
    const Timestamp_t &now,
    const Payload_t &payload,
    Packet_t &packet_to_build)
{
    Tag_t tag{0x0};
    Payload_t encrypted_payload{0x0};

    OpStatus status = Encrypt(now, payload, tag, encrypted_payload);
    if (status != OpStatus::kSuccess)
    {
        return OpStatus::kInvalidInput;
    }

    // Build packet
    memcpy(packet_to_build+offsetof(PacketStruct_t, PVN), kPVN, sizeof(PVN_t));
    memcpy(packet_to_build+offsetof(PacketStruct_t, Id), Id, sizeof(NodeId_t));
    memcpy(packet_to_build+offsetof(PacketStruct_t, Ts), now, sizeof(Timestamp_t));
    packet_to_build[offsetof(PacketStruct_t, PayloadLen)] = kPayloadLength;
    memcpy(packet_to_build+offsetof(PacketStruct_t, Payload), encrypted_payload, sizeof(Payload_t));
    memcpy(packet_to_build+offsetof(PacketStruct_t, Tag), tag, sizeof(Tag_t));

    return OpStatus::kSuccess;
}

static const uint8_t const_Rb[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87};
static const uint8_t const_Zero[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

Crypto::AES_CMAC::AES_CMAC(AESTiny256 &aes256) : aes256(aes256)
{
}

void Crypto::AES_CMAC::generateMAC(uint8_t *mac, const uint8_t *key, const uint8_t *data, size_t dataLen)
{
    aes256.setKey(key, aes256.keySize());

    for (int i = 0; i < 16; i++)
    {
        mac[i] = 0;
    }

    aes256.encryptBlock(mac, const_Zero);
    for (int i = 0; i < 16; ++i)
    {
        X[i] = mac[i];
    }

    shiftLeft(X, sizeof(X));
    if (mac[0] & 0x80)
    {
        xor128(X, X, const_Rb);
    }

    for (int i = 0; i < 16; ++i)
    {
        Y[i] = X[i];
    }
    shiftLeft(Y, sizeof(Y));
    if (X[0] & 0x80)
    {
        xor128(Y, Y, const_Rb);
    }

    int n = (dataLen + 15) / 16;
    int flag = 0;
    if (n == 0)
    {
        n = 1;
    }
    else if ((dataLen % 16) == 0)
    {
        flag = 1;
    }

    if (flag)
    {
        xor128(mac, &data[16 * (n - 1)], X);
    }
    else
    {
        padding(mac, &data[16 * (n - 1)], dataLen % 16);
        xor128(mac, mac, Y);
    }
    for (int i = 0; i < 16; i++)
    {
        X[i] = 0;
    }
    for (int i = 0; i < n - 1; i++)
    {
        xor128(Y, X, &data[16 * i]);
        aes256.encryptBlock(X, Y);
    }

    xor128(Y, mac, X);
    aes256.encryptBlock(mac, Y);
}

void Crypto::AES_CMAC::shiftLeft(uint8_t *buff, uint8_t buffLen)
{
    while (buffLen--)
    {
        uint8_t next = buffLen ? buff[1] : 0;

        uint8_t val = (*buff << 1);
        if (next & 0x80)
        {
            val |= 1;
        }
        *buff++ = val;
    }
}

void Crypto::AES_CMAC::xor128(uint8_t *out, const uint8_t *a, const uint8_t *b)
{
    for (int i = 0; i < 16; i++)
    {
        *out++ = (*a++) ^ (*b++);
    }
}

void Crypto::AES_CMAC::padding(uint8_t *pad, const uint8_t *lastb, int len)
{
    for (int i = 0; i < 16; i++)
    {
        if (i < len)
        {
            *pad++ = *lastb++;
        }
        else if (i == len)
        {
            *pad++ = 0x80;
        }
        else
        {
            *pad++ = 0x00;
        }
    }
}