#include <Arduino.h>
#include <esp_arduino_version.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// ===== YOUR WHEEL DRIVER PINS =====
const int RPWM1 = 25;
const int LPWM1 = 26;
const int EN1   = 14;

const int RPWM2 = 32;
const int LPWM2 = 33;
const int EN2   = 5;

const char FIRMWARE_BUILD[] = "wheels_only_r3";

const int PWM_FREQ = 20000;
const int PWM_BITS = 8;

const float TRIM1 = 1.0f;
const float TRIM2 = 1.0f;

// Change the affected direction to -1 if a wheel runs backward.
const int DIR1 = 1;
const int DIR2 = 1;

const uint32_t COMMAND_TIMEOUT_MS = 500;
const uint32_t REVERSE_PAUSE_MS = 100;

// ===== MOP RELAYS (ACTIVE LOW) =====
const int RELAY_MOP1_PIN = 13;   // relay 1
const int RELAY_MOP3_PIN = 4;    // relay 3

const int RELAY_ON  = LOW;
const int RELAY_OFF = HIGH;

// Minimum rest time before a relay is energised again after switching off.
const uint32_t MOP_MIN_OFF_MS = 1000;

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

// ===== MOP RELAY =====
struct Relay {
  int pin;
  bool requested;   // what the last MOP command asked for
  bool on;          // what the pin is actually driving
  bool everOn;
  uint32_t offAt;

  void begin() {
    digitalWrite(pin, RELAY_OFF);   // set the level before enabling the output
    pinMode(pin, OUTPUT);
    digitalWrite(pin, RELAY_OFF);
    requested = false;
    on = false;
    everOn = false;
    offAt = 0;
  }

  void off() {
    digitalWrite(pin, RELAY_OFF);
    if (on) {
      offAt = millis();
    }
    on = false;
  }

  // Relay runs only while it is requested AND the robot is under
  // active command (allowed).
  void update(uint32_t now, bool allowed) {
    if (!requested || !allowed) {
      if (on) off();
      return;
    }

    if (on) return;

    if (everOn &&
        static_cast<uint32_t>(now - offAt) < MOP_MIN_OFF_MS) {
      return;
    }

    digitalWrite(pin, RELAY_ON);
    on = true;
    everOn = true;
  }
};

Relay relayMop1 = { RELAY_MOP1_PIN, false, false, false, 0 };
Relay relayMop3 = { RELAY_MOP3_PIN, false, false, false, 0 };

// Cancel any mop request and switch both relays off immediately.
void mopStop() {
  relayMop1.requested = false;
  relayMop3.requested = false;
  relayMop1.off();
  relayMop3.off();
}

bool pwmReady = false;
bool commandActive = false;

int targetL = 0;
int targetR = 0;

uint32_t lastCommandAt = 0;

char rxLine[64];
size_t rxLength = 0;
bool discardLine = false;

// ===== STOP =====
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

  mopStop();
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
      "STATUS target=%d,%d pwm=%d,%d ready=%d build=%s\n",
      targetL, targetR,
      wheelL.output, wheelR.output,
      pwmReady, FIRMWARE_BUILD
    );
    return;
  }

  if (strcmp(line, "MOP?") == 0) {
    Serial.printf(
      "MOP req=%d,%d on=%d,%d\n",
      relayMop1.requested, relayMop3.requested,
      relayMop1.on, relayMop3.on
    );
    return;
  }

  // MOP <relay1> <relay3>   (0 = off, 1 = on), e.g. "MOP 1 1".
  // The relays only run while wheel commands are active, and any
  // stop/timeout/bad line cancels the request.
  if (strncmp(line, "MOP ", 4) == 0) {
    int a = -1, b = -1;
    char extra;

    if (sscanf(line + 4, "%d %d %c", &a, &b, &extra) != 2 ||
        (a != 0 && a != 1) || (b != 0 && b != 1)) {
      mopStop();
      Serial.println("ERR EXPECT_MOP_0_OR_1_0_OR_1");
      return;
    }

    if (!pwmReady) {
      mopStop();
      Serial.println("ERR PWM_INIT");
      return;
    }

    relayMop1.requested = (a == 1);
    relayMop3.requested = (b == 1);

    Serial.printf("OK MOP %d %d\n", a, b);
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
  // Relays first: drive them to the OFF level as early as possible.
  relayMop1.begin();
  relayMop3.begin();

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

  if (pwmReady && commandActive) {
    uint32_t now = millis();
    wheelL.update(targetL, now);
    wheelR.update(targetR, now);
  }

  uint32_t mopNow = millis();
  relayMop1.update(mopNow, commandActive);
  relayMop3.update(mopNow, commandActive);

  delay(1);
}
