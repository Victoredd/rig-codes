#include <Arduino.h>
// CONSTANT DECLARATIONS
constexpr int MOTOR1 = 12;
constexpr int MOTOR2 = 13;
constexpr int PWM_CH1 = 0;
constexpr int PWM_CH2 = 1;
constexpr int PWM_FREQ = 50;      // Hz control signal
constexpr int PWM_RESOLUTION = 16;      // resolution in bits

// ESC range (microseconds)
constexpr uint32_t ESC_MIN_PULSE = 1000;
constexpr uint32_t ESC_MAX_PULSE = 2000;

// Control strategy output range (rotational oscillator: [-1, 1], but magnitude is what matters and we interpret sign manually)
constexpr float STRATEGY_OUTPUT_MIN = 0.0;
constexpr float STRATEGY_OUTPUT_MAX = 1.0;

// Derived constants
constexpr float PERIOD_US = 1000000.0f / PWM_FREQ;     // 20000.0 microseconds
constexpr uint32_t MAX_DUTY = (1UL << PWM_RESOLUTION) - 1UL;
constexpr uint32_t MIN_DUTY = (uint32_t)((ESC_MIN_PULSE / PERIOD_US) * (float)MAX_DUTY + 0.5f);

// FUNCTION DECLARATIONS

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

// SETUP AND LOOP