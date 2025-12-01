#include <Arduino.h>
#include <vector>
#include "SparkFun_BNO08x_Arduino_Library.h"

// Web functionality declarations
void initWebServer();
void handleWebServer();


// CONSTANT DECLARATIONS
constexpr int MOTOR1 = 12;
constexpr int MOTOR2 = 13;
constexpr int PWM_CH1 = 0;
constexpr int PWM_CH2 = 1;
constexpr int PWM_FREQ = 50; // Hz control signal
constexpr int PWM_RESOLUTION = 16; // resolution in bits

// Tweak later
float gain_p = 0.5;
float gain_d = 0.1;
float gain_i = 0.1;
unsigned long lastLoopTime = 0;
float integralSum = 0.0;
float error = 0.0;
float lastError = 0.0;

float calibMiddle = 0.0;

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
    ledcWrite(PWM_CH1, duty);
    ledcWrite(PWM_CH2, MIN_DUTY);
  }
  else if (controlValue > 0) {
    uint32_t duty = controlToDuty(controlValue);
    ledcWrite(PWM_CH1, MIN_DUTY);
    ledcWrite(PWM_CH2, duty);
  }
  else {
    ledcWrite(PWM_CH1, MIN_DUTY);
    ledcWrite(PWM_CH2, MIN_DUTY);
  }
}

float sensorRead(int selectedSensor) {
  return gyro.getYaw();
}

void calibrate(int selectedSensor) {
  float calibLow = sensorRead(selectedSensor);
  delay(5000);
  float calibHigh = sensorRead(selectedSensor);
  calibMiddle = (calibLow + calibHigh) / 2.0;
}

float runControl(float sensorValue, int selectedStrategy) {
  unsigned long now = millis();
  float dt = (now - lastLoopTime) / 1000.0;
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

// Gyro
BNO08x gyro;

// SETUP AND LOOP

void setup() {
  // Sensor
  Wire.begin();
  gyro.begin(0x4A, Wire, -1, -1); // 0x4A is default I2C address
  gyro.enableRotationVector();
  // PWM
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR1, PWM_CH1);
  ledcAttachPin(MOTOR2, PWM_CH2);

  // Web server (http://esprig.local)
  initWebServer();

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
      lastLoopTime = millis();
    }
    // Avoid race conditions
    int currentSensor = selectedSensor;
    int currentStrategy = selectedStrategy;

    float sensorValue = sensorRead(currentSensor);
    float controlOutput = runControl(sensorValue, currentStrategy);
    setMotorPower(controlOutput);

    // Log data
    DataPoint dp;
    dp.timestamp = millis();
    dp.sensorValue = sensorValue;
    dp.selectedSensor = selectedSensor; // get from web
    dp.error = error;
    dp.controlOutput = controlOutput;
    dp.strategyUsed = selectedStrategy; // get from web
    dataLog.push_back(dp);

  } else {
    // Not running
    setMotorPower(0.0);
    if (wasRunning) wasRunning = false;
  }
  delay(5); 
}