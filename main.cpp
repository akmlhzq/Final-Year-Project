#include <Arduino.h>
#include <esp_arduino_version.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// ===== RELAY PINS (match the wiring table) =====
const int RELAY1 = 13;   // mop motor (IN1)
const int RELAY2 = 19;   // light (IN2)
const int RELAY3 = 4;    // mop motor (IN3)
const int RELAY4 = 18;   // water motor (IN4)
const int RELAY5 = 2;    // underlight (IN5)

// In this robot HIGH = relay ON, LOW = relay OFF.
// If your relay module works the opposite way, swap these two lines only.
const int RELAY_ON  = HIGH;
const int RELAY_OFF = LOW;

// ===== WHEEL DRIVER PINS =====
const int RPWM1 = 25;
const int LPWM1 = 26;
const int EN1   = 14;

const int RPWM2 = 32;
const int LPWM2 = 33;
const int EN2   = 5;

const char FIRMWARE_BUILD[] = "wheels_mop_r4";

const int PWM_FREQ = 20000;
const int PWM_BITS = 8;

// Wheel speed: 1.0 was the old setting, 0.85 = 15% slower.
// Change these two numbers to make the wheels faster or slower.
const float TRIM1 = 0.85f;
const float TRIM2 = 0.85f;

// Change the affected direction to -1 if a wheel runs backward.
const int DIR1 = 1;
const int DIR2 = 1;

const uint32_t COMMAND_TIMEOUT_MS = 500;
const uint32_t REVERSE_PAUSE_MS   = 100;

// Mop motors stay ON this long after the wheels stop.
const uint32_t MOP_OFF_DELAY_MS = 3000;

// ===== PWM: ESP32 ARDUINO CORE 2.x AND 3.x =====
bool pwmInit(int pin, int channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  if (ledcSetup(channel, PWM_FREQ, PWM_BITS) == 0) {
    return false;
  }
  ledcAttachPin(pin, channel);
  return true;
#endif
}

void pwmWrite(int pin, int channel, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// ===== RELAY HELPERS =====
// Set the level first, then make the pin an output.
// This avoids a short click on the relay during boot.
void relayInit(int pin, int level) {
  digitalWrite(pin, level);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, level);
}

// States of the relays (used by STATUS and by the database later)
bool lightOn = true;    // relay 2 + 5: stays ON for now (database later)
bool waterOn = false;   // relay 4: stays OFF for now (database later)
bool mopOn   = false;   // relay 1 + 3: follows the wheels

void applyLights() {
  int level = lightOn ? RELAY_ON : RELAY_OFF;
  digitalWrite(RELAY2, level);
  digitalWrite(RELAY5, level);
}

void applyWater() {
  digitalWrite(RELAY4, waterOn ? RELAY_ON : RELAY_OFF);
}

void applyMop() {
  int level = mopOn ? RELAY_ON : RELAY_OFF;
  digitalWrite(RELAY1, level);   // relay 1 and 3 always switch together
  digitalWrite(RELAY3, level);
}

// ===== WHEEL MOTOR =====
struct Motor {
  int rpwm, lpwm, en;
  int channelR, channelL;
  float trim;
  int direction;
  int output;
  int lastDirection;
  uint32_t stoppedAt;

  bool begin() {
    pinMode(en, OUTPUT);
    digitalWrite(en, LOW);

    bool r = pwmInit(rpwm, channelR);
    bool l = pwmInit(lpwm, channelL);

    if (r) pwmWrite(rpwm, channelR, 0);
    if (l) pwmWrite(lpwm, channelL, 0);

    return r && l;
  }

  void stop() {
    digitalWrite(en, LOW);
    pwmWrite(rpwm, channelR, 0);
    pwmWrite(lpwm, channelL, 0);

    if (output != 0) {
      stoppedAt = millis();
    }
    output = 0;
  }

  void update(int requested, uint32_t now) {
    int duty = static_cast<int>(
      requested * trim * direction
    );

    if (duty > 255) duty = 255;
    if (duty < -255) duty = -255;

    if (duty == 0) {
      stop();
      return;
    }

    int sign = duty > 0 ? 1 : -1;

    // Non-blocking pause before reversing.
    if (lastDirection != 0 && sign != lastDirection) {
      if (output != 0) {
        stop();
        return;
      }

      if (static_cast<uint32_t>(now - stoppedAt)
          < REVERSE_PAUSE_MS) {
        return;
      }
    }

    if (output == duty) return;

    if (duty > 0) {
      pwmWrite(lpwm, channelL, 0);
      pwmWrite(rpwm, channelR, duty);
    } else {
      pwmWrite(rpwm, channelR, 0);
      pwmWrite(lpwm, channelL, -duty);
    }

    digitalWrite(en, HIGH);
    output = duty;
    lastDirection = sign;
  }
};

Motor wheelL = {
  RPWM1, LPWM1, EN1, 0, 1,
  TRIM1, DIR1, 0, 0, 0
};

Motor wheelR = {
  RPWM2, LPWM2, EN2, 2, 3,
  TRIM2, DIR2, 0, 0, 0
};

bool pwmReady = false;
bool commandActive = false;

int targetL = 0;
int targetR = 0;

uint32_t lastCommandAt = 0;
uint32_t lastMoveAt = 0;     // last time a wheel was moving

char rxLine[64];
size_t rxLength = 0;
bool discardLine = false;

// ===== STOP (wheels only, relays are handled by updateMop) =====
void stopAll() {
  targetL = 0;
  targetR = 0;
  commandActive = false;

  if (pwmReady) {
    wheelL.stop();
    wheelR.stop();
  }

  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
}

// ===== MOP LOGIC =====
// Wheels moving      -> mop relays ON right away
// Wheels stopped     -> mop relays OFF after MOP_OFF_DELAY_MS (3 s)
void updateMop(uint32_t now) {
  bool wheelsMoving = (wheelL.output != 0 || wheelR.output != 0);

  if (wheelsMoving) {
    lastMoveAt = now;
    if (!mopOn) {
      mopOn = true;
      applyMop();
    }
  } else if (mopOn &&
             static_cast<uint32_t>(now - lastMoveAt) >= MOP_OFF_DELAY_MS) {
    mopOn = false;
    applyMop();
  }
}

// ===== SERIAL PARSER =====
// Compatible with Python commands:
// left_speed right_speed 0 0
//
// The last two fields are reserved and must be zero.
bool parseValues(const char *text, int values[4]) {
  const char *p = text;

  for (int i = 0; i < 4; ++i) {
    while (isspace(static_cast<unsigned char>(*p))) {
      ++p;
    }

    const char *start = p;

    if (*p == '+' || *p == '-') {
      ++p;
    }

    const char *digits = p;

    while (isdigit(static_cast<unsigned char>(*p))) {
      ++p;
    }

    if (p == digits || p - digits > 3) {
      return false;
    }

    if (*p && !isspace(static_cast<unsigned char>(*p))) {
      return false;
    }

    long value = strtol(start, nullptr, 10);

    if (value < -255 || value > 255) {
      return false;
    }

    values[i] = static_cast<int>(value);
  }

  while (isspace(static_cast<unsigned char>(*p))) {
    ++p;
  }

  return *p == '\0';
}

void handleLine(char *line) {
  while (isspace(static_cast<unsigned char>(*line))) {
    ++line;
  }

  size_t n = strlen(line);

  while (n &&
         isspace(static_cast<unsigned char>(line[n - 1]))) {
    line[--n] = '\0';
  }

  if (!n) return;

  if (strcmp(line, "STOP") == 0) {
    stopAll();
    Serial.println("OK STOP");
    return;
  }

  if (strcmp(line, "PING") == 0) {
    stopAll();
    Serial.println(
      pwmReady
        ? "PONG robot_wheels_v1"
        : "ERR PWM_INIT"
    );
    return;
  }

  if (strcmp(line, "STATUS") == 0) {
    Serial.printf(
      "STATUS target=%d,%d pwm=%d,%d ready=%d mop=%d light=%d water=%d build=%s\n",
      targetL, targetR,
      wheelL.output, wheelR.output,
      pwmReady, mopOn, lightOn, waterOn, FIRMWARE_BUILD
    );
    return;
  }

  int values[4];

  if (!parseValues(line, values)) {
    stopAll();
    Serial.println("ERR EXPECT_4_INTS_-255_TO_255");
    return;
  }

  if (values[2] != 0 || values[3] != 0) {
    stopAll();
    Serial.println("ERR RESERVED_FIELDS_MUST_BE_ZERO");
    return;
  }

  if (!pwmReady) {
    stopAll();
    Serial.println("ERR PWM_INIT");
    return;
  }

  targetL = values[0];
  targetR = values[1];

  lastCommandAt = millis();
  commandActive = (targetL != 0 || targetR != 0);

  if (!commandActive) {
    stopAll();
  }

  Serial.printf(
    "OK %d %d 0 0\n",
    values[0], values[1]
  );
}

void readSerial() {
  for (int budget = 0;
       budget < 64 && Serial.available();
       ++budget) {

    char c = static_cast<char>(Serial.read());

    if (c == '\n') {
      if (!discardLine) {
        rxLine[rxLength] = '\0';
        handleLine(rxLine);
      }

      rxLength = 0;
      discardLine = false;

    } else if (!discardLine) {
      unsigned char u = static_cast<unsigned char>(c);

      if ((u < 32 && c != '\t' && c != '\r') ||
          u > 126 ||
          rxLength >= sizeof(rxLine) - 1) {

        stopAll();
        discardLine = true;
        rxLength = 0;

        Serial.println("ERR BAD_OR_LONG_LINE");
      } else {
        rxLine[rxLength++] = c;
      }
    }
  }
}

// ===== COMMUNICATION TIMEOUT =====
void checkTimeout() {
  if (commandActive &&
      static_cast<uint32_t>(millis() - lastCommandAt)
        >= COMMAND_TIMEOUT_MS) {

    stopAll();
    Serial.println("STOP TIMEOUT");
  }
}

// ===== STARTUP =====
void setup() {
  // Relays: lights ON, mop OFF, water OFF
  relayInit(RELAY2, lightOn ? RELAY_ON : RELAY_OFF);
  relayInit(RELAY5, lightOn ? RELAY_ON : RELAY_OFF);
  relayInit(RELAY1, RELAY_OFF);
  relayInit(RELAY3, RELAY_OFF);
  relayInit(RELAY4, waterOn ? RELAY_ON : RELAY_OFF);

  pinMode(EN1, OUTPUT);
  digitalWrite(EN1, LOW);

  pinMode(EN2, OUTPUT);
  digitalWrite(EN2, LOW);

  Serial.begin(115200);

  bool leftOK = wheelL.begin();
  bool rightOK = wheelR.begin();

  pwmReady = leftOK && rightOK;
  stopAll();

  Serial.printf("INFO build=%s\n", FIRMWARE_BUILD);

  Serial.println(
    pwmReady
      ? "READY robot_wheels_v1"
      : "ERR PWM_INIT"
  );
}

// ===== MAIN LOOP =====
void loop() {
  checkTimeout();
  readSerial();
  checkTimeout();

  uint32_t now = millis();

  if (pwmReady && commandActive) {
    wheelL.update(targetL, now);
    wheelR.update(targetR, now);
  }

  updateMop(now);

  delay(1);
}
