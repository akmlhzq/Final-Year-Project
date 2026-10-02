#include <Arduino.h>

// Works with ESP32 Arduino core 2.x (PlatformIO: platform = espressif32@6.9.0)

struct Motor {
  int rpwm, lpwm, ren, len;
  int chFwd, chRev;
  bool invert;   // true = flip this motor's direction
};

//              RPWM LPWM R_EN L_EN chF chR invert
Motor motor1 = {25,  26,  27,  14,  0,   1,  false};
Motor motor2 = {32,  33,  18,  19,  2,   3,  false};

// ===== SETTINGS: change these =====
// Motor 1
const int   M1_FWD_SPEED = 100;    // percent (0-100)
const float M1_FWD_TIME  = 5.0;   // seconds
const int   M1_REV_SPEED = 60;
const float M1_REV_TIME  = 5.0;

// Motor 2
const int   M2_FWD_SPEED = 100;
const float M2_FWD_TIME  = 5.0;
const int   M2_REV_SPEED = 60;
const float M2_REV_TIME  = 5.0;

// Both motors together
const int   BOTH_SPEED   = 100;
const float BOTH_TIME    = 4.0;

const float PAUSE        = 1.0;   // stop time between moves, seconds
const int   REPEAT_COUNT = 4;     // how many times to repeat everything (0 = forever)
// ==================================

void motorInit(Motor &m) {
  pinMode(m.ren, OUTPUT);
  pinMode(m.len, OUTPUT);
  digitalWrite(m.ren, HIGH);
  digitalWrite(m.len, HIGH);
  ledcSetup(m.chFwd, 20000, 8);   // 20 kHz, 8-bit
  ledcSetup(m.chRev, 20000, 8);
  ledcAttachPin(m.rpwm, m.chFwd);
  ledcAttachPin(m.lpwm, m.chRev);
}

// percent: 0-100, forward: true = forward, false = reverse
void motorRun(Motor &m, int percent, bool forward) {
  if (m.invert) forward = !forward;
  int duty = map(constrain(percent, 0, 100), 0, 100, 0, 255);
  ledcWrite(m.chFwd, forward ? duty : 0);
  ledcWrite(m.chRev, forward ? 0 : duty);
}

void motorStop(Motor &m) {
  ledcWrite(m.chFwd, 0);
  ledcWrite(m.chRev, 0);
}

// Run one motor in one direction for a time, then stop and pause
void runStep(Motor &m, int percent, bool forward, float seconds) {
  motorRun(m, percent, forward);
  delay(seconds * 1000);
  motorStop(m);
  delay(PAUSE * 1000);
}

// Run both motors together in one direction for a time, then stop and pause
void runBoth(int percent, bool forward, float seconds) {
  motorRun(motor1, percent, forward);
  motorRun(motor2, percent, forward);
  delay(seconds * 1000);
  motorStop(motor1);
  motorStop(motor2);
  delay(PAUSE * 1000);
}

void setup() {
  motorInit(motor1);
  motorInit(motor2);

  int cycles = 0;
  while (REPEAT_COUNT == 0 || cycles < REPEAT_COUNT) {
    // Motor 1: forward then reverse
    runStep(motor1, M1_FWD_SPEED, true,  M1_FWD_TIME);
    runStep(motor1, M1_REV_SPEED, false, M1_REV_TIME);

    // Motor 2: forward then reverse
    runStep(motor2, M2_FWD_SPEED, true,  M2_FWD_TIME);
    runStep(motor2, M2_REV_SPEED, false, M2_REV_TIME);

    // Both together: forward then reverse
    runBoth(BOTH_SPEED, true,  BOTH_TIME);
    runBoth(BOTH_SPEED, false, BOTH_TIME);

    cycles++;
  }

  motorStop(motor1);
  motorStop(motor2);
}

void loop() {
  // nothing, the sequence runs in setup()
  // press the ESP32 EN (reset) button to run it again
}