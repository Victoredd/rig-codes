/* #include <Arduino.h>

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
*/

/*
  Using the BNO08x IMU

  This example shows how to output the parts of the calibrated gyro.

  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Originally written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  Adjusted by Pete Lewis @ SparkFun Electronics, June 2023 to incorporate the
  CEVA Sensor Hub Driver, found here:
  https://github.com/ceva-dsp/sh2

  Also, utilizing code from the Adafruit BNO08x Arduino Library by Bryan Siepert
  for Adafruit Industries. Found here:
  https://github.com/adafruit/Adafruit_BNO08x

  Also, utilizing I2C and SPI read/write functions and code from the Adafruit
  BusIO library found here:
  https://github.com/adafruit/Adafruit_BusIO

  Hardware Connections:
  IoT RedBoard --> BNO08x
  QWIIC --> QWIIC
  A4  --> INT
  A5  --> RST

  BNO08x "mode" jumpers set for I2C (default):
  PSO: OPEN
  PS1: OPEN

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/22857

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

BNO08x gyro;

void setup() {
  delay(7000);
  Serial.begin(115200);

  Serial.println("Initializing I2C...");
  Wire.begin(21, 22); 
  Wire.setClock(100000); // 100kHz

  gyro.begin(0x4B, Wire, -1, -1); 

  // Enable Game Rotation Vector (No magnetometer, just Gyro+Accel)
  // 50ms report interval = 20Hz updates for reading
  gyro.enableGameRotationVector(50); 
  
  Serial.println("Sensor Active. Rotate the drone.");
  Serial.println("Time\tYaw(Z)\tPitch(X)\tRoll(Y)");
}

void loop() {
  if (gyro.getSensorEvent()) {
    if (gyro.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      
      float yaw = gyro.getYaw();
      float pitch = gyro.getPitch();
      float roll = gyro.getRoll();

      Serial.print(millis());
      Serial.print("\t");
      Serial.print(yaw, 3);
      Serial.print("\t");
      Serial.print(pitch, 3);
      Serial.print("\t");
      Serial.println(roll, 3);
    }
  }
}
*/
/* i2c scanner */
#include <Arduino.h>
#include <Wire.h>

// Change these if you use custom pins:
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

// Uncomment if you want to scan a second I2C bus
// constexpr int SDA_PIN_2 = 25;
// constexpr int SCL_PIN_2 = 26;

void scanBus(TwoWire &bus, const char *busName) {
    Serial.printf("\nScanning %s...\n", busName);

    int count = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        bus.beginTransmission(addr);
        uint8_t error = bus.endTransmission();

        if (error == 0) {
            Serial.printf(" - Device found at 0x%02X\n", addr);
            count++;
        }
    }

    if (count == 0) {
        Serial.println("No I2C devices found.");
    } else {
        Serial.printf("Found %d device(s).\n", count);
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("I2C Scanner starting...");

    // Init primary I2C bus
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.printf("Primary I2C bus on SDA=%d, SCL=%d\n", SDA_PIN, SCL_PIN);

    // Init secondary I2C bus (optional)
    // Wire1.begin(SDA_PIN_2, SCL_PIN_2);
    // Serial.printf("Secondary I2C bus on SDA=%d, SCL=%d\n", SDA_PIN_2, SCL_PIN_2);
}

void loop() {
    scanBus(Wire, "Wire");

    // Uncomment if using second bus:
    // scanBus(Wire1, "Wire1");

    Serial.println("\n--- Scan complete. Next scan in 3 seconds ---");
    delay(3000);
}