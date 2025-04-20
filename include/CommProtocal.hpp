#include <Arduino.h>

#define PACKET_ACK 0x01
#define PACKET_START_BYTE 0x55
#define PACKET_HEADER 0xAA55
#define PACKET_END 0xFF

#define ENCODER_PACKET_ID 0x01
#define COMMAND_PACKET_ID 0x02
#define LANDMARK_PACKET_ID 0x03
#define STATUS_PACKET_ID 0x04

struct LandmarkPacket 
{
    uint16_t header = PACKET_HEADER;
    uint8_t packetID = LANDMARK_PACKET_ID;
    uint16_t Checksum = 0x00; // checksum placeholder
    uint8_t LandmarkID = 0x00; // anchor ID (A or B)
    float range = 0.0; // range in meters
    float rxPower; // new field to store the received power
};

// Data packet structures
struct EncoderDataPacket 
{
    uint16_t header = PACKET_HEADER;
    uint8_t packetID = ENCODER_PACKET_ID; // Encoder Data Packet ID
    uint16_t Checksum = 0x00; // checksum placeholder
    float encA = 0.0;
    float encB = 0.0;
    float velA = 0.0;
    float velB = 0.0;
}; // 19 Bytes Total (In theory)

struct CommandPacket 
{
    uint16_t header = PACKET_HEADER;
    uint8_t packetID = COMMAND_PACKET_ID; // Command Packet ID
    uint16_t Checksum = 0x00; // checksum placeholder
    float VelA = 0.0;   
    float VelB = 0.0;
};

struct StatusPacket
{
    uint16_t header = PACKET_HEADER;
    uint8_t packetID = STATUS_PACKET_ID;
    uint16_t Checksum = 0x00; // checksum placeholder
    bool connected = false;
};

uint16_t calculateChecksum(uint8_t* data, size_t size) 
{
    uint16_t checksum = 0;
    for (size_t i = 0; i < size; i++) 
    {
        //Exlcude the checksum byte itself from the checksum calculation
        if (i != 3 && i != 4)
        {
            checksum += data[i];
        }
    }
    return checksum;
}

// Writes packet data in 32-byte blocks to the serial port
void writePacketToSerial(uint8_t* data, size_t size)
{
    Serial.write(data, size);
    for (unsigned int i = size; i < 31; i++)
    {
        Serial.write(0x00);
    }
    Serial.write(PACKET_END);
}