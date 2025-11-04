#include <Arduino.h>

// Define pins
const int pin1 = 12;
const int pin2 = 13;

// Each PWM channel must be unique
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;

// 50 Hz signal (20 ms period)
const int pwmFreq = 50;
const int pwmResolution = 16; // Higher resolution to allow fine timing control

// Convert microseconds (1000–2000 µs) to duty
uint32_t pulseToDuty(int microseconds) {
  return (uint32_t)((microseconds / 20000.0) * (1 << pwmResolution));
}

void setup() {
  // Set up PWM channels
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);

  // Attach pins
  ledcAttachPin(pin1, pwmChannel1);
  ledcAttachPin(pin2, pwmChannel2);

  // Precompute duty values
  uint32_t dutyOff = pulseToDuty(1000); // minimum throttle
  uint32_t dutyOn  = pulseToDuty(1400); // 5% throttle

  // Startup: send min throttle for 3 seconds to arm ESCs
  ledcWrite(pwmChannel1, dutyOff);
  ledcWrite(pwmChannel2, dutyOff);
  delay(3000);

  // Store for later use in loop
  ledcWrite(pwmChannel1, dutyOff);
  ledcWrite(pwmChannel2, dutyOff);
}

void loop() {
  uint32_t dutyOff = pulseToDuty(1000);
  uint32_t dutyOn  = pulseToDuty(1400);

  // Turn "on" for 5 seconds
  ledcWrite(pwmChannel1, dutyOn);
  ledcWrite(pwmChannel2, dutyOn);
  delay(2000);

  // Return to "off" (minimum throttle) for 5 seconds
  ledcWrite(pwmChannel1, dutyOff);
  ledcWrite(pwmChannel2, dutyOff);
  delay(5000);
}