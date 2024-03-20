# Apogeo Space Node Library
This library provides all necessary functions to connect your node to the Apogeo Space LoRa network!
It comprises a high level set of functionalities to quickly start sending data from your device(s), as well as
more in-depth functions to e.g. use the cryptographic layer with different connections of the hardware, or even
for custom board with different transmitters.

## Installation

### Requirements
To correctly use this library, the following support libraries must also be installed:
- [Crypto](https://rweather.github.io/arduinolibs/crypto.html) (0.4.0) by Rhys Weatherley.

Unless stated otherwise, these libraries can be found in the Arduino IDE library manager.

### Manual installation
To install this library, just place this entire folder as a subfolder in your
Arduino/libraries folder. You usually find the *libraries* folder in the sketch folder.

When installed, this library should look like:

> \<Arduino Sketches\>/libraries/ApsNode
>
> \<Arduino Sketches\>/libraries/ApsNode/APSNode.hpp
> 
> \<Arduino Sketches\>/libraries/ApsNode/APSCrypto.cpp
>
> \<Arduino Sketches\>/libraries/ApsNode/APSCrypto.hpp
>
> \<Arduino Sketches\>/libraries/ApsNode/APSLora.cpp
>
> \<Arduino Sketches\>/libraries/ApsNode/APSLora.hpp
> 
> \<Arduino Sketches\>/libraries/ApsNode/keywords.txt
>
> \<Arduino Sketches\>/libraries/ApsNode/examples
>
> \<Arduino Sketches\>/libraries/ApsNode/readme.md   (this file)

--------------------------------------------------------------------------------

## Basic Usage

Please refer to the **examples** folder for more information on how to use this library.
At a quick glance, here's how you can send data with your credentials.
The same code is found under examples/SimpleNode

```cpp
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
    if (not node.Init())
    {
        Serial.println("Initialization failed!");
        while (1)
            ;
    }
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
```

### Configuring your Apogeo LoRa Shield

If you received an Arduino 169 MHz LoRa Shield from us, or built one using the design files freely retrievable from https://github.com/ApogeoSpace/ArduinoLoRaShield, you might have a different configuration of the wiring pins (RST, SS, D0) than the ones configured by default within the library.

The default configuration for the current version of this library is:
| Signal | Pin |
| --- | --- |
| D0 | 3 |
| RST | 5 |
| SS | 6 |


If that's different from the one present on the hardware you're using, then simply make sure to change the creation of a **APSNode** object to include the updated configuration.
For example, if your configuration differs from the one above in the following way:
| Signal | Pin |
| --- | --- |
| D0 | **2** |
| RST | 5 |
| SS | **10** |

Then you'd need to instantiate the APSNode object in the following way:

```c++
//                         RST     SS      D0
APSNode node(id, key,       5,     10,     2);
```

### Sending data

The library provides the `::Send`, `::SendString` and `::SendStream` APIs to transmit data via LoRa, which hide lower level details when dealing with some common types of variables. 

#### Sending `Payload` or generic objects with `::Send`

The `::Send` function can deal with either `Payload` variables or any non-pointer type, as long as its size does not exceed the allowable length of the payload.

```c++

/* When applicable, using basic types with ::Send instead of converting them to a Payload type is more convenient
    and user-friendly.
*/
float myFloat;
// ... process myFloat
node.Send(myFloat);

// TinyStruct is a POD with size <= sizeof(Payload)
TinyStruct tinys;
// ... process tinys
node.Send(tinys);

/* Note that types with size > sizeof(Payload) are rejected at compile time */
struct BigStruct
{
    int buff[20];
};

BigStruct bigs{};

// Suppose sizeof(Payload) < 20 * sizeof(int)
node.Send(bigs); // <--- will not compile! "error: static assertion failed: The value you're trying to send won't fit in a single payload!"

/* When using Payload variables, the user must deal with the "low-level" byte nature of that payload, 
    so using memcpy or other serialization techniques can become necessary to e.g. send a non uint8_t type 
    by copying it into a Payload
*/
int myInt;
// ... process myInt
Payload pl{};
memcpy(pl, myInt, sizeof(myInt));
node.Send(pl);

```
#### Sending raw memory with `::SendStream`

By design, `::Send` cannot be used to deal with pointers. If you have a pointer to an area of memory and want to send its content, you can use `::SendStream` instead, by passing the pointer itself and the amount of bytes you want to copy over to the payload.

```c++

uint8_t* ptr = new uint8_t[10]{};

// ... process ptr

node.SendStream(ptr, 10);

```

Note that this is still subjected to the limitation of the Payload size!

```c++

uint8_t* ptr = new uint8_t[20]{};

// ... process ptr

// Suppose Payload has max size 10

node.SendStream(ptr, 20);   // --> will return false and not transmit anything

```

#### Sending C-style strings and string literals with `::SendString`

Sending strings is **not** particularly recommended, however you can still do it by means of `::SendString`. This function deals with **NULL-terminated** strings and string literals, and builds a Payload by copying in character by character.

```c++

const char * str1 {"Hello"};

// Setting false as the second argument (auto_trim) inhibits transmission in case the string is longer than the Payload size
//  and needs to be trimmed down. 
node.SendString(str1, false);

const char * str2 {"Hello beautiful world!"};

// Suppose the Payload max. size is 10 bytes. Then the following function will fail as it cannot send the whole string + its terminator
node.SendString(str2, false);

// In case you do not care about only sending a part of it, you can ask the function to "trim down" the string 
// !!! Calling this will not guarantee the presence of a NULL terminator in the final payload!!! Use it only if you know what you're doing.
node.SendString(str2, true);    // --> Will send "Hello beau" _without_ NULL terminator!!
```

--------------------------------------------------------------------------------


### APSCrypto - Cryptography Library for Apogeo Space Network
This library is to be used on ground nodes that wish to upload data to the Apogeo Space PiCO network.
It provides a way to encrypt data with the node personal key, in accordance to the network format.

#### Usage

```cpp
// Include the APSCrypto library in your sketch
#include "APSCrypto.hpp"
using namespace ApogeoSpace::Crypto;

// Declare the node personal key as a NodeKey_t array
constexpr NodeKey_t my_node_key {
    0x0D, 0x0E, 0x0A, 0x0D, 0x0B, 0x0E, 0x0E, 0x0F, 
    // ... (must be 32 bytes long)

};

// Declare the node personal ID as a NodeId_t array
constexpr NodeId_t my_node_id {
    0x01, 0x03, 0x03, 0x07
};

void setup()
{
    // Create a Node object to encrypt your data.
    Node mynode{my_node_id, my_node_key};

    // Assign data to a Payload_t. You can treat it as a regular buffer of 10 bytes!
    // You can also pass the payload object to other function that take array of bytes as input.
    Payload_t my_payload{};
    my_payload[0] = 10;
    //...
    my_payload[9] = 1;

    // Create a Packet_t object for holding the packet.
    // You also have to provide a Timestamp_t object, containing an UTC timestamp, to the encryption
    // functions. Unless otherwise specified, you can also leave it to 0 if you don't have a valid timestamp provider.
    Timestamp_t ts{};
    Packet_t packet{};

    // Build the packet with your payload!
    mynode.BuildPacket(now, packet, my_payload);

    // Now you can use the resulting packet object as a byte array to e.g. send it to a LoRa radio
    // for transmission. Easy peasy!

}
```

## Credits
--------------------------------------------------------------------------------
Thanks to Piotr Obst for the [AES_CMAC](https://www.arduino.cc/reference/en/libraries/aes_cmac/) library, which has been adapted to work with AES-256 (originally only purposed for AES-128).

