#include <Arduino.h>

// ---------- BTS7960 driver 1 ----------
const int RPWM1 = 25;
const int LPWM1 = 26;
const int EN1   = 14;   // R_EN + L_EN tied together

// ---------- BTS7960 driver 2 ----------
const int RPWM2 = 32;
const int LPWM2 = 33;
const int EN2   = 5;    // R_EN + L_EN tied together (GPIO5 is HIGH at boot)

// ---------- Relays (active LOW) ----------
const int RELAY1 = 4;
const int RELAY3 = 13;
const int RELAY4 = 18;

// ---------- PWM settings ----------
const int PWM_FREQ = 20000;   // 20 kHz, above audible range
const int PWM_BITS = 8;       // duty 0..255

// Speed trim per motor (use to compensate RPM difference)
const float TRIM1 = 0.6;
const float TRIM2 = 0.6;

// Direction per motor: 1 = normal, -1 = flipped
// If one motor spins the wrong way, change its value here
const int DIR1 = 1;
const int DIR2 = 1;

// ---------- PWM helpers (work on ESP32 Arduino core 2.x and 3.x) ----------
void pwmInit(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// ---------- Motor ----------
struct Motor {
  int rpwm, lpwm, en;
  int chR, chL;
  float trim;
  int dir;
};

Motor m1 = {RPWM1, LPWM1, EN1, 0, 1, TRIM1, DIR1};
Motor m2 = {RPWM2, LPWM2, EN2, 2, 3, TRIM2, DIR2};

void motorInit(Motor &m) {
  pinMode(m.en, OUTPUT);
  digitalWrite(m.en, LOW);              // disabled while PWM is set up
  pwmInit(m.rpwm, m.chR);
  pwmInit(m.lpwm, m.chL);
  pwmWrite(m.rpwm, m.chR, 0);
  pwmWrite(m.lpwm, m.chL, 0);
  digitalWrite(m.en, HIGH);             // enable driver, motor stopped
}

// speed: -255 (full reverse) .. 0 (stop) .. +255 (full forward)
void motorSet(Motor &m, int speed) {
  speed = constrain((int)(speed * m.trim * m.dir), -255, 255);
  if (speed > 0) {
    pwmWrite(m.lpwm, m.chL, 0);         // never drive both sides at once
    pwmWrite(m.rpwm, m.chR, speed);
  } else if (speed < 0) {
    pwmWrite(m.rpwm, m.chR, 0);
    pwmWrite(m.lpwm, m.chL, -speed);
  } else {
    pwmWrite(m.rpwm, m.chR, 0);
    pwmWrite(m.lpwm, m.chL, 0);
  }
}

// ---------- Relays (non-blocking: ON 10 s, OFF 5 s) ----------
const unsigned long RELAY_ON_MS  = 10000;   // relays ON time
const unsigned long RELAY_OFF_MS = 5000;    // relays OFF time
bool relaysOn = false;
unsigned long relayTimer = 0;

void relaysSet(bool on) {
  int level = on ? LOW : HIGH;          // active LOW module
  digitalWrite(RELAY1, level);
  digitalWrite(RELAY3, level);
  digitalWrite(RELAY4, level);
}

void updateRelays() {
  unsigned long now = millis();
  if (relaysOn && now - relayTimer >= RELAY_ON_MS) {
    relaysSet(false);
    relaysOn = false;
    relayTimer = now;
  } else if (!relaysOn && now - relayTimer >= RELAY_OFF_MS) {
    relaysSet(true);
    relaysOn = true;
    relayTimer = now;
  }
}

// ---------- Motor demo sequence (non-blocking) ----------
const int SPEED = 128;                  // about 50%

struct Step { int s1, s2; unsigned long ms; };
Step steps[] = {
  { SPEED,  SPEED, 5000},   // both motors forward
  {     0,      0,  300},   // short pause before changing direction
  {-SPEED, -SPEED, 5000},   // both motors reverse
  {     0,      0,  300},
};
const int NUM_STEPS = sizeof(steps) / sizeof(steps[0]);
int stepIndex = 0;
unsigned long stepTimer = 0;

void applyStep(int i) {
  motorSet(m1, steps[i].s1);
  motorSet(m2, steps[i].s2);
}

void updateMotors() {
  if (millis() - stepTimer >= steps[stepIndex].ms) {
    stepIndex = (stepIndex + 1) % NUM_STEPS;
    stepTimer = millis();
    applyStep(stepIndex);
  }
}

// ---------- Arduino ----------
void setup() {
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  relaysSet(true);                      // start with relays ON
  relaysOn = true;
  relayTimer = millis();

  motorInit(m1);
  motorInit(m2);
  stepTimer = millis();
  applyStep(0);
}

void loop() {
  updateRelays();
  updateMotors();
}