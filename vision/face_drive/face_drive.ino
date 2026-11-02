// =============================================================================
// Exterminate - face_drive.ino
// Arduino Nano (ATmega328P) drive firmware for the face-tracking bench test.
//
// The Raspberry Pi sends line-based commands over USB serial (115200 baud):
//
//   M <left> <right>\n   set motor speeds, each -255..255 (clamped)
//   S\n                  stop both motors
//   P\n                  ping (replies "PONG"; also feeds the watchdog)
//
// Replies:
//   READY                once at boot, after the motors are confirmed stopped
//   OK                   after every valid M / S command
//   ERR <reason>         malformed / unknown command (does NOT feed watchdog)
//   STAT L=<l> R=<r> WD=<0|1>   status line roughly once per second
//   TIMEOUT              printed once when the watchdog stops the motors
//
// Safety:
//   * Motors are stopped at boot (STBY LOW, PWM 0).
//   * Watchdog: if no valid command for CMD_TIMEOUT_MS (300 ms), motors stop.
//   * Weapon ESC on D3 is held at a 1000 us disarmed pulse the whole time.
//     This sketch NEVER commands the weapon to spin.
//   * HotRC receiver pins A1-A3 are left as plain inputs (unused here).
// =============================================================================

#include <Servo.h>

// ---------------------------- Pin map ---------------------------------------
// TB6612FNG motor driver
const uint8_t PIN_STBY = 4;   // standby: LOW = driver disabled (motors coast)
const uint8_t PIN_PWMA = 5;   // left motor  PWM (Timer0, ~980 Hz)
const uint8_t PIN_AIN1 = 7;   // left motor  direction 1
const uint8_t PIN_AIN2 = 8;   // left motor  direction 2
const uint8_t PIN_PWMB = 6;   // right motor PWM (Timer0, ~980 Hz)
const uint8_t PIN_BIN1 = 9;   // right motor direction 1
const uint8_t PIN_BIN2 = 10;  // right motor direction 2

// Weapon ESC signal - disarmed only, never spun from this sketch
const uint8_t  PIN_WEAPON_ESC   = 3;
const uint16_t WEAPON_SAFE_US   = 1000;  // standard "throttle zero" pulse

// HotRC receiver channels (not used in this sketch, kept as inputs)
const uint8_t RX_PINS[3] = {A1, A2, A3};

// ---------------------------- Tunables --------------------------------------
// Flip a motor if it spins the wrong way: set to true and re-upload.
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = false;

const long          SERIAL_BAUD     = 115200;
const unsigned long CMD_TIMEOUT_MS  = 300;   // watchdog: stop if no valid cmd
const unsigned long STATUS_EVERY_MS = 1000;  // periodic STAT line
const int           SPEED_LIMIT     = 255;   // hard clamp for |speed|

// ---------------------------- State -----------------------------------------
Servo weaponEsc;

const uint8_t LINE_MAX = 32;
char    lineBuf[LINE_MAX];
uint8_t lineLen = 0;
bool    lineOverflow = false;

int  curLeft  = 0;
int  curRight = 0;
unsigned long lastCmdMs    = 0;
unsigned long lastStatusMs = 0;
bool watchdogTripped = true;   // start "tripped" so we are stopped until a cmd

// ---------------------------- Motor helpers ---------------------------------
static int clampSpeed(long v) {
  if (v >  SPEED_LIMIT) return  SPEED_LIMIT;
  if (v < -SPEED_LIMIT) return -SPEED_LIMIT;
  return (int)v;
}

// Drive one TB6612 channel. speed: -255..255. 0 = brake-free stop (coast).
static void driveChannel(uint8_t pwmPin, uint8_t in1, uint8_t in2,
                         int speed, bool invert) {
  if (invert) speed = -speed;
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

static void setMotors(int left, int right) {
  curLeft  = clampSpeed(left);
  curRight = clampSpeed(right);
  driveChannel(PIN_PWMA, PIN_AIN1, PIN_AIN2, curLeft,  INVERT_LEFT);
  driveChannel(PIN_PWMB, PIN_BIN1, PIN_BIN2, curRight, INVERT_RIGHT);
  // Only enable the driver when something should actually move.
  digitalWrite(PIN_STBY, (curLeft != 0 || curRight != 0) ? HIGH : LOW);
}

static void stopMotors() {
  setMotors(0, 0);  // also drops STBY LOW
}

// ---------------------------- Command parsing -------------------------------
// Parse a signed integer from *p, advancing p. Returns false if none found.
static bool parseInt(const char *&p, long &out) {
  while (*p == ' ' || *p == '\t') p++;
  char *end;
  out = strtol(p, &end, 10);
  if (end == p) return false;
  p = end;
  return true;
}

static void handleLine(char *line) {
  // Skip leading whitespace; ignore empty lines silently.
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  char cmd = line[0];
  if (cmd >= 'a' && cmd <= 'z') cmd -= 32;  // accept lowercase too
  const char *p = line + 1;

  if (cmd == 'M') {
    long l, r;
    if (!parseInt(p, l) || !parseInt(p, r)) {
      Serial.println(F("ERR bad M args"));
      return;
    }
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '\0') {
      Serial.println(F("ERR trailing junk"));
      return;
    }
    setMotors(clampSpeed(l), clampSpeed(r));
    lastCmdMs = millis();
    watchdogTripped = false;
    Serial.println(F("OK"));
  } else if (cmd == 'S') {
    stopMotors();
    lastCmdMs = millis();
    watchdogTripped = false;
    Serial.println(F("OK"));
  } else if (cmd == 'P') {
    lastCmdMs = millis();
    Serial.println(F("PONG"));
  } else {
    Serial.println(F("ERR unknown cmd"));
  }
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineOverflow) {
        Serial.println(F("ERR line too long"));
      } else {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;  // discard until newline
    }
  }
}

// ---------------------------- Setup / loop ----------------------------------
void setup() {
  // 1) Motors off before anything else.
  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, LOW);
  pinMode(PIN_PWMA, OUTPUT); analogWrite(PIN_PWMA, 0);
  pinMode(PIN_PWMB, OUTPUT); analogWrite(PIN_PWMB, 0);
  pinMode(PIN_AIN1, OUTPUT); digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT); digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT); digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT); digitalWrite(PIN_BIN2, LOW);

  // 2) Weapon ESC: steady 1000 us disarmed pulse. Never changed below.
  //    Note: Servo uses Timer1, which disables analogWrite on D9/D10 - fine,
  //    those are only used as digital direction pins here.
  weaponEsc.attach(PIN_WEAPON_ESC);
  weaponEsc.writeMicroseconds(WEAPON_SAFE_US);

  // 3) Receiver pins: plain inputs, unused.
  for (uint8_t i = 0; i < 3; i++) pinMode(RX_PINS[i], INPUT);

  Serial.begin(SERIAL_BAUD);
  stopMotors();
  lastCmdMs = millis();
  lastStatusMs = millis();
  Serial.println(F("READY"));
}

void loop() {
  pollSerial();

  unsigned long now = millis();

  // Watchdog: stop if the Pi goes quiet.
  if (!watchdogTripped && (now - lastCmdMs > CMD_TIMEOUT_MS)) {
    stopMotors();
    watchdogTripped = true;
    Serial.println(F("TIMEOUT"));
  }

  // Periodic status line.
  if (now - lastStatusMs >= STATUS_EVERY_MS) {
    lastStatusMs = now;
    Serial.print(F("STAT L="));  Serial.print(curLeft);
    Serial.print(F(" R="));      Serial.print(curRight);
    Serial.print(F(" WD="));     Serial.println(watchdogTripped ? 1 : 0);
  }
}
