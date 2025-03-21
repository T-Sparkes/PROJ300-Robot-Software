#include <Arduino.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include "MotorController.hpp"

#define CE_PIN 7
#define CSN_PIN 8
#define PC_ADDRESS 0xA5
#define ROBOT_ADDRESS 0xB5

#define PACKET_ACK 0x01
#define PACKET_START_BYTE 0x55
#define PACKET_HEADER 0xAA55
#define ENCODER_PACKET_ID 0x01
#define RANGE_PACKET_ID 0x03
#define SERIAL_MAX_WAIT_MS 10


#define PACKET_ACK 0x01
#define PACKET_START_BYTE 0x55
#define PACKET_HEADER 0xAA55
#define ENCODER_PACKET_ID 0x01
#define RANGE_PACKET_ID 0x03
#define SERIAL_MAX_WAIT_MS 10

void ENC_ISR_A();
void ENC_ISR_B();

RF24 radio(CE_PIN, CSN_PIN);

MotorController motorA(6, 2, ENC_ISR_A);
MotorController motorB(5, 3, ENC_ISR_B);
volatile uint32_t countA = 0;
volatile uint32_t countB = 0;

float targetVelA = 0;
float targetVelB = 0;

struct EncoderDataPacket 
{
    uint16_t header;
    uint8_t packetID;
    float encA;
    float encB;
    float velA;
    float velB;
}; // 19 Bytes Total (In theory)

struct CommandPacket 
{
    uint16_t header;
    uint8_t packetID;
    float VelA;
    float VelB;
};

struct AnchorRangePacket 
{
    uint16_t header;
    uint8_t packetID;
    uint8_t anchorID;
    float range;
};

void setup() 
{
  Serial.begin(115200);
  while (!Serial);

  if (!radio.begin()) 
  {
    Serial.println("Radio Module Not Responding");
    while (true);
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.openWritingPipe(PC_ADDRESS);
  radio.openReadingPipe(1, ROBOT_ADDRESS); 
  radio.startListening();
}

void loop() 
{
  static bool packetAck = false;
  static uint32_t lastTx = 0;

  static CommandPacket rxCommand;
  static EncoderDataPacket EncoderTX;
  static AnchorRangePacket RangeTX;


  if (radio.available())
  {
    uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
    radio.read((uint8_t*)&rxCommand, sizeof(rxCommand));
    
    targetVelA = rxCommand.VelA;
    targetVelB = rxCommand.VelB;

    Serial.print("Vel Command: ");
    Serial.print(rxCommand.VelA);
    Serial.print(", ");
    Serial.println(rxCommand.VelB);

    EncoderTX = {PACKET_HEADER, ENCODER_PACKET_ID, motorA.GetPosition(), motorB.GetPosition(), motorA.GetVelocity(), motorB.GetVelocity()};

    radio.stopListening();
    radio.write((uint8_t*)&EncoderTX, sizeof(EncoderDataPacket));
    radio.startListening();
  }
  
  motorA.Update(targetVelA, countA);
  motorB.Update(targetVelB, countB);  
}

void ENC_ISR_A() { countA++; }
void ENC_ISR_B() { countB++; }