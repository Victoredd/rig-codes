#include <Arduino.h>
#include <vector>
#include <Wire.h> 
#include "SparkFun_BNO08x_Arduino_Library.h"

// Web functionality declarations
void initWebServer();
void handleWebServer();

// Gyro
BNO08x gyro;
float lastYaw = 0.0;

// CONSTANT DECLARATIONS
constexpr int MOTOR1 = 13;
constexpr int MOTOR2 = 12;
constexpr int PWM_CH1 = 0;
constexpr int PWM_CH2 = 1;
constexpr int PWM_FREQ = 50; // Hz control signal
constexpr int PWM_RESOLUTION = 16; // resolution in bits

// Tweak later
float gain_p = 0.2;
float gain_d = 0.05;
float gain_i = 0.1;
unsigned long lastLoopTime = 0;
float integralSum = 0.0;
float error = 0.0;
float lastError = 0.0;

// Calibration Variables
float calibMiddle = 0.0;
float calibLowVal = 0.0; 
float calibHighVal = 0.0;

struct DataPoint {
    uint32_t timestamp;
    float sensorValue;
    int selectedSensor;
    float error;
    float controlOutput;
    int strategyUsed;
};

volatile bool running = false;
volatile int selectedStrategy = 0;
volatile int selectedSensor = 0; // Despite the rotational oscillator only having one sensor, this makes it more homogenous with the vertical oscillator code
bool wasRunning = false; // was running in last loop?
std::vector<DataPoint> dataLog;

// ESC range (microseconds)
constexpr uint32_t ESC_MIN_PULSE = 1000;
constexpr uint32_t ESC_MAX_PULSE = 2000;

// Control strategy output range (rotational oscillator: [-1, 1], but magnitude is what matters and we interpret sign manually)
constexpr float STRATEGY_OUTPUT_MIN = 0.0;
constexpr float STRATEGY_OUTPUT_MAX = 1.0;

// Derived constants
constexpr float PERIOD_US = 1000000.0f / PWM_FREQ; // 20000.0 microseconds
constexpr uint32_t MAX_DUTY = (1UL << PWM_RESOLUTION) - 1UL;
constexpr uint32_t MIN_DUTY = (uint32_t)((ESC_MIN_PULSE / PERIOD_US) * (float)MAX_DUTY + 0.5f);

// FUNCTIONS

// Constrain to [a,b]
float clamp(float v, float a, float b) {
  if (v < a) return a;
  if (v > b) return b;
  return v;
}

// Linear map from range to another
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  float t = (x - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}

// Control strategy output to duty value for ESC
uint32_t controlToDuty(float controlValue) {
  // Clamp control input to valid range
  float v = clamp(controlValue, STRATEGY_OUTPUT_MIN, STRATEGY_OUTPUT_MAX);

  // Control to pulse width
  float pulse = mapFloat(v, STRATEGY_OUTPUT_MIN, STRATEGY_OUTPUT_MAX, ESC_MIN_PULSE, ESC_MAX_PULSE);

  // Pulse width to duty fraction of the period
  float fraction = pulse / PERIOD_US;
  fraction = clamp(fraction, 0.0f, 1.0f);

  // Duty units and round
  uint32_t duty = (uint32_t)(fraction * (float)MAX_DUTY + 0.5f);

  return duty;
}

void setMotorPower(float controlValue) {
  // swing one way or the other depending on sign of controlValue
  if (controlValue < 0) {
    uint32_t duty = controlToDuty(-controlValue);
    ledcWrite(PWM_CH1, MIN_DUTY);
    ledcWrite(PWM_CH2, duty);
  }
  else if (controlValue > 0) {
    uint32_t duty = controlToDuty(controlValue);
    ledcWrite(PWM_CH1, duty);
    ledcWrite(PWM_CH2, MIN_DUTY);
  }
  else {
    ledcWrite(PWM_CH1, MIN_DUTY);
    ledcWrite(PWM_CH2, MIN_DUTY);
  }
}

float sensorRead(int selectedSensor) {
  if (gyro.getSensorEvent() && gyro.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    lastYaw = gyro.getYaw();
  }
  return lastYaw;
}

// Split calib functions
void calibrateLowStep(int selectedSensor) {
    float sum = 0.0;
    for(int i = 0; i < 100; i++) {
        sum += sensorRead(selectedSensor);
        delay(10);
    }
    calibLowVal = sum / 100.0;
    calibMiddle = (calibLowVal + calibHighVal) / 2.0;
}

void calibrateHighStep(int selectedSensor) {
    float sum = 0.0;
    for(int i = 0; i < 100; i++) {
        sum += sensorRead(selectedSensor);
        delay(10);
    }
    calibHighVal = sum / 100.0;
    calibMiddle = (calibLowVal + calibHighVal) / 2.0;
}

float runControl(float sensorValue, int selectedStrategy) {
  unsigned long now = micros();
  float dt = (now - lastLoopTime) / 1000000.0;
  lastLoopTime = now;
  lastError = error;
  error = calibMiddle - sensorValue;
  // 0/other = dummy, 1 = P, 2 = on/off, 3 = PID
  switch(selectedStrategy) {
    default: return 0.0;
    case 1: {
      // proportional
      return gain_p * error;
    }
    case 2: {
      // on/off
      if (error > 0) return 1.0; //max power one way
      else if (error < 0) return -1.0; //max power other way
      else return 0.0; // if it's exactly down the middle

    }
    case 3: {
      // PID
      integralSum += error * dt;
      float derivative = (error - lastError) / dt;
      return gain_p * error + gain_i * integralSum + gain_d * derivative;
    }
  }
}

// SETUP AND LOOP

void setup() {
  delay(7000);
  Serial.begin(115200);
  Serial.println("Booting ESP32...");
  // Gyroscope init
  Wire.begin(21, 22); // SDA, SCL
  Wire.setClock(400000); // 400 kHz
  gyro.begin(0x4B, Wire, -1, -1); // 0x4B is default I2C address
  delay(500);
  gyro.enableRotationVector();
  // PWM
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR1, PWM_CH1);
  ledcAttachPin(MOTOR2, PWM_CH2);

  // Web server (http://esprig.local)
  Serial.println("Starting web server...");
  initWebServer();
  Serial.println("Web server started.");

  // ESC arming
  ledcWrite(PWM_CH1, MIN_DUTY);
  ledcWrite(PWM_CH2, MIN_DUTY);
  delay(2000);
}

void loop() {
  handleWebServer();
  if (running) {

    if (!wasRunning) {
      dataLog.clear();
      wasRunning = true;
      integralSum = 0.0;
      lastLoopTime = micros();
    }
    // Avoid race conditions
    int currentSensor = selectedSensor;
    int currentStrategy = selectedStrategy;

    float sensorValue = sensorRead(currentSensor);
    float controlOutput = runControl(sensorValue, currentStrategy);
    setMotorPower(controlOutput);

    // Log data
    DataPoint dp;
    dp.timestamp = micros();
    dp.sensorValue = sensorValue;
    dp.selectedSensor = selectedSensor;
    dp.error = error;
    dp.controlOutput = controlOutput;
    dp.strategyUsed = selectedStrategy;
    dataLog.push_back(dp);
    if (dataLog.size() > 7000) running = false; // stop after 7k data points, about 35 seconds at 200 Hz

  } else {
    // Not running
    setMotorPower(0.0);
    if (wasRunning) wasRunning = false;
  }
  delay(5); 
}