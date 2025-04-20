#include <Arduino.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#include "MotorController.hpp"
#include "CommProtocal.hpp"

// Pin definitions for RF24 module
#define CE_PIN 7
#define CSN_PIN 8

// Communication addresses
#define PC_ADDRESS 0xA5
#define ROBOT_ADDRESS 0xB5

// Packet definitions
#define PACKET_ACK 0x01
#define PACKET_START_BYTE 0x55
#define PACKET_HEADER 0xAA55
#define ENCODER_PACKET_ID 0x01
#define RANGE_PACKET_ID 0x03
#define SERIAL_MAX_WAIT_MS 10

// Function prototypes for encoder interrupts
void ENC_ISR_A();
void ENC_ISR_B();

// RF24 radio object
RF24 radio(CE_PIN, CSN_PIN);

// Closed Loop motor controller objects
MotorController motorA(6, 2, ENC_ISR_A);
MotorController motorB(5, 3, ENC_ISR_B);

// Volatile variables to store encoder counts
volatile uint32_t countA = 0;
volatile uint32_t countB = 0;

// Target velocities for motors
float targetVelA = 0;
float targetVelB = 0;

// Flags for acknowledgments
bool radioAck = false;
bool serialAck = false;

void setup() 
{
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial);

    // Initialize RF24 radio module
    if (!radio.begin()) 
    {
        Serial.println("Radio Module Not Responding");
        while (true); // Halt execution if radio fails to initialize
    }

    // Configure RF24 settings
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_2MBPS);
    radio.openWritingPipe(PC_ADDRESS);
    radio.openReadingPipe(1, ROBOT_ADDRESS); 
    radio.startListening();
}

void loop() 
{
    static uint32_t lastTx = 0; // Timestamp for last transmission
    static CommandPacket rxCommand; // Received command packet
    static EncoderDataPacket EncoderTX; // Encoder data packet to transmit

    // Check if data is available from the radio
    if (radio.available())
    {
        // Read the received command packet
        radio.read((uint8_t*)&rxCommand, sizeof(rxCommand));
        
        // Update target velocities based on received command
        targetVelA = rxCommand.VelA;
        targetVelB = rxCommand.VelB;
    }

    // Check if data is available on the serial port from the UWB Module
    if (Serial.available())
    {
        if (Serial.peek() == PACKET_ACK)
        {
            // Handle acknowledgment packet
            Serial.read();
            serialAck = true;
        }
        else if (Serial.peek() == PACKET_START_BYTE)
        {
            // Handle range packet
            LandmarkPacket rxSerialPacket;
            Serial.readBytes((uint8_t*)&rxSerialPacket, sizeof(LandmarkPacket));
            uint16_t checksum = calculateChecksum((uint8_t*)&rxSerialPacket, sizeof(LandmarkPacket)); // Check packet is valid

            if (rxSerialPacket.header == PACKET_HEADER && rxSerialPacket.packetID == RANGE_PACKET_ID && rxSerialPacket.Checksum == checksum)
            {
                // Forward range packet to the radio to be sent to PC
                radio.stopListening();
                radioAck = radio.write((uint8_t*)&rxSerialPacket, sizeof(LandmarkPacket));
                radio.startListening();
            }
        }
        else
        {
            Serial.read();
        }
    }

    // Transmit encoder data every 20ms / at 50Hz
    if (millis() - lastTx > 20)
    {
        lastTx = millis();

        // Prepare encoder data packet
        EncoderTX.encA = motorA.GetPosition();
        EncoderTX.encB = motorB.GetPosition();
        EncoderTX.velA = motorA.GetVelocity();
        EncoderTX.velB = motorB.GetVelocity();
        EncoderTX.Checksum = calculateChecksum((uint8_t*)&EncoderTX, sizeof(EncoderDataPacket));
        
        // Transmit encoder data packet
        radio.stopListening();
        radio.write((uint8_t*)&EncoderTX, sizeof(EncoderDataPacket));
        radio.startListening();
    }

    // Update motor controllers with target velocities and encoder counts
    motorA.Update(targetVelA, countA);
    motorB.Update(targetVelB, countB);  
}

// Interrupt for Encoder A
void ENC_ISR_A() 
{ 
    countA++; 
}

// Interrupt for Encoder B
void ENC_ISR_B() 
{ 
    countB++; 
}