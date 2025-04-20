#include <Arduino.h>

class MotorController
{
private:

    // Pin Definitions
    uint8_t FWD_PIN;
    uint8_t INTR_PIN;

    // Encoder Variables
    uint32_t encoderCount = 0;
    uint32_t countLast = 0;

    // PI Controller 
    const float kp = 0.1, ki = 1.5;
    float errorIntegral = 0;
    float targetV = 0;

    // Velocity Filter
    float velFilt = 0;
    float velPrev = 0;

    // Constants
    const float countsPerRev = 823.1 * 2;
    const float countsToRadians = (2.0f * PI) / countsPerRev; // Precompute conversion

    // Timing
    uint32_t lastUpdate = 0;
    const uint16_t sampleRate = 50; // Hz
    const uint32_t sampleTime = (1.0 / (float)sampleRate) * 1e6; 

    // Pointer to ISR function
    void (*EncoderISR)(void) = nullptr;

public:
    MotorController(uint8_t FWD_PIN, uint8_t INTR_PIN, void (*EncoderISR)(void));
    ~MotorController();
    void Update(float setpoint, volatile uint32_t& count);
    float GetVelocity();
    float GetPosition();
};

MotorController::MotorController(uint8_t FWD_PIN, uint8_t INTR_PIN, void (*EncoderISR)(void))
{
    this->FWD_PIN = FWD_PIN;
    this->INTR_PIN = INTR_PIN;
    this->EncoderISR = EncoderISR;
    pinMode(FWD_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(INTR_PIN), EncoderISR, CHANGE);
}

MotorController::~MotorController()
{
    detachInterrupt(digitalPinToInterrupt(INTR_PIN));
}

void MotorController::Update(float setpoint, volatile uint32_t& count)
{
    uint32_t now = micros();
    if (now - lastUpdate < sampleTime) return; // Only update at sample Rate
    float dt = (now - lastUpdate) * 1e-6f; // Convert to seconds
    lastUpdate = now;

    // Safely read count
    noInterrupts();
    encoderCount = count;
    interrupts();

    // Calculate velocity
    float vel = (encoderCount - countLast) / dt;
    countLast = encoderCount;

    // Apply low-pass filter to velocity
    velFilt = 0.614 * velFilt + 0.193 * vel + 0.193 * velPrev; // 10Hz
    velPrev = vel;

    // Add bounds to target Velocity
    targetV = setpoint;
    targetV = (targetV > 2 * PI) ? 2 * PI : targetV;
    targetV = (targetV < 0.5) ? 0 : targetV;

    // PI Controller
    float e = targetV / countsToRadians - velFilt;
    float u = kp * e + ki * (errorIntegral += e * dt);
  
    // Anti-windup
    errorIntegral = (errorIntegral > 255) ? 255 : errorIntegral;
    errorIntegral = (errorIntegral < -255) ? -255 : errorIntegral;
  
    // Add bounds to control signal
    u = (u > 255) ? 255 : u;
    u = (u < 0) ? 0 : u;
  
    //Set PWM signal
    analogWrite(FWD_PIN, u);
}

// Get the velocity in radians per second
float MotorController::GetVelocity()
{
    return velFilt * countsToRadians;
}

// Get the position in radians
float MotorController::GetPosition()
{
    return encoderCount * countsToRadians;
}