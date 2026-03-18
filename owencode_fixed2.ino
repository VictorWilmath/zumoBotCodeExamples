#include <Zumo32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include <Zumo32U4Buzzer.h>

LSM6 imu;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors prox;
Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;

// Forward declarations
int checkRing(uint16_t raw[]);
void autoCalibrateRing();
void performRam();
void checkEvadeRing(int ringStatus);

const uint8_t SEARCH_THRESHOLD = 3;
const uint8_t COMMIT_THRESHOLD = 3;
const int ENCODER_STALL_THRESHOLD = 2;
const int pushSpeed = 380;
const int dodgeSpeed = 200;
int scanDirection = 1;
unsigned long scanTimer = 0;
unsigned long pivotTimer = 0;
const unsigned long arcTime = 900;
const unsigned long pivotTime = 220;
const unsigned long pivotInterval = 3000;
const float collisionThreshold = 20000;

// ---------------------------------------------------------------------------
// Proximity sensor readings — refreshed every loop via readProxSensors().
// Using the SAME four variables as the reference tracking code so there is
// one consistent source of truth throughout the entire sketch.
// ---------------------------------------------------------------------------
uint8_t frontLeft  = 0;
uint8_t frontRight = 0;
uint8_t sideLeft   = 0;
uint8_t sideRight  = 0;

// Aggregate helpers (used by checkEvadeRing / dodge to check "any target")
#define ANY_FRONT  (frontLeft > 0 || frontRight > 0)
#define ANY_LEFT   (sideLeft  > 0)
#define ANY_RIGHT  (sideRight > 0)
#define ANY_TARGET (ANY_FRONT || ANY_LEFT || ANY_RIGHT)

uint8_t ringConfirmCount = 0;
const uint8_t RING_CONFIRM_LIMIT = 3;

// Confirm consecutive front detections before committing to a ram
uint8_t frontConfirmCount = 0;
const uint8_t FRONT_COMMIT_LOOPS = 2;

uint16_t rawSensorValues[3];
uint16_t ringThresholds[3];

// Evade timing and FSM variables
const unsigned long EVADE_COOLDOWN = 1500;
unsigned long lastEvadeAt = 0;
unsigned long EVADE_BACK_MS    = 180;
unsigned long EVADE_TURN_MS    = 300;
unsigned long EVADE_FORWARD_MS = 350;
int EVADE_TURN_SPEED = 200;

#ifndef GEAR_RATIO
#define GEAR_RATIO 75
#endif

enum State { idle, search, dodge, ram, evadeRing };
State currentState = idle;

enum EvadeState { EVADE_INIT, EVADE_BACK, EVADE_TURN, EVADE_FORWARD };
EvadeState currentEvadeState = EVADE_INIT;
unsigned long evadeTimer = 0;
int evadeDirection = 0;

// ---------------------------------------------------------------------------
// readProxSensors() — mirrors the reference sketch exactly.
// Call this right before any decision that depends on proximity data so the
// readings are never stale. The old single prox.read() at the top of loop()
// was separated from the actual use of the data by up to ~20 ms of blocking
// (getDebouncedRingDirection samples 5× at 4 ms each), causing the side
// sensor branches to act on outdated values and miss targets.
// ---------------------------------------------------------------------------
void readProxSensors() {
  prox.read();
  frontLeft  = prox.countsFrontWithLeftLeds();
  frontRight = prox.countsFrontWithRightLeds();
  sideLeft   = prox.countsLeftWithLeftLeds();
  sideRight  = prox.countsRightWithRightLeds();
}

int checkRing(uint16_t raw[]) {
  const int DROP_DELTA = 220;
  const int HYST = 40;

  if (ringThresholds[0] == 0 && ringThresholds[1] == 0 && ringThresholds[2] == 0)
    return 99;

  if ((int)ringThresholds[0] - (int)raw[0] > DROP_DELTA + HYST) return -1; // left
  if ((int)ringThresholds[2] - (int)raw[2] > DROP_DELTA + HYST) return 1;  // right
  if ((int)ringThresholds[1] - (int)raw[1] > DROP_DELTA + HYST) return 0;  // center
  return 99;
}

bool collisionDetected() {
  imu.read();
  return abs(imu.a.x) > collisionThreshold;
}

void autoCalibrateRing() {
  const uint8_t samples = 25;
  uint32_t sum[3] = {0, 0, 0};
  uint16_t temp[3];
  for (uint8_t s = 0; s < samples; s++) {
    lineSensors.read(temp);
    for (int i = 0; i < 3; i++) sum[i] += temp[i];
    delay(15);
  }
  for (int i = 0; i < 3; i++) ringThresholds[i] = sum[i] / samples;
}

// Debounced ring check — only suppresses on proximity contact so a genuine
// edge still overrides. Uses ANY_TARGET which is always current because
// readProxSensors() is called before getDebouncedRingDirection() is reached.
int getDebouncedRingDirection(uint8_t samples = 5, uint8_t required = 3) {
  if (ANY_TARGET) return 99;

  uint8_t counts[3] = {0, 0, 0};
  uint16_t tmp[3];
  for (uint8_t i = 0; i < samples; ++i) {
    lineSensors.read(tmp);
    int d = checkRing(tmp);
    if      (d == -1) counts[0]++;
    else if (d ==  0) counts[1]++;
    else if (d ==  1) counts[2]++;
    delay(4);
  }
  if (counts[1] >= required) return 0;
  if (counts[0] >= required) return -1;
  if (counts[2] >= required) return 1;
  return 99;
}

bool safeDelay(unsigned long ms, bool allowForward = false) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    uint16_t sd_raw[3];
    lineSensors.read(sd_raw);
    int edge = checkRing(sd_raw);
    if (edge != 99) {
      motors.setSpeeds(0, 0);
      unsigned long revStart = millis();
      while (millis() - revStart < 400) {
        motors.setSpeeds(-380, -380);
        uint16_t chk[3];
        lineSensors.read(chk);
        if (checkRing(chk) == 99) break;
      }
      motors.setSpeeds(0, 0);
      evadeDirection = edge;
      currentEvadeState = EVADE_INIT;
      currentState = evadeRing;
      return true;
    }
    delay(5);
  }
  return false;
}

void performRam() {
  lcd.gotoXY(10, 0);
  lcd.print("RM");

  motors.setSpeeds(-200, -200);
  if (safeDelay(50)) return;

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  const long targetCounts = 600;
  const unsigned long maxMs = 1000;
  const int maxSpeed = pushSpeed;
  const float Kp = 1.4f;
  const int minForward = 150;
  const int maxCmd = 400;

  unsigned long start = millis();
  motors.setSpeeds(maxSpeed, maxSpeed);

  long startL = encoders.getCountsLeft();
  long startR = encoders.getCountsRight();

  while (millis() - start < maxMs) {
    uint16_t ramRaw[3];
    lineSensors.read(ramRaw);
    int ringDuringRam = checkRing(ramRaw);
    if (ringDuringRam != 99) {
      motors.setSpeeds(0, 0);
      evadeDirection = ringDuringRam;
      currentEvadeState = EVADE_INIT;
      currentState = evadeRing;
      return;
    }

    long l = abs(encoders.getCountsLeft()  - startL);
    long r = abs(encoders.getCountsRight() - startR);
    if ((l >= targetCounts) || (r >= targetCounts)) break;

    long err = l - r;
    float adj = Kp * err;
    int rawLeftCmd  = constrain(maxSpeed - (int)adj, minForward, maxCmd);
    int rawRightCmd = constrain(maxSpeed + (int)adj, minForward, maxCmd);
    motors.setSpeeds(rawLeftCmd, rawRightCmd);
    delay(5);
  }

  for (int burst = 0; burst < 2; ++burst) {
    uint16_t burstRaw[3];
    lineSensors.read(burstRaw);
    if (checkRing(burstRaw) != 99) { if (safeDelay(10)) return; }
    motors.setSpeeds(maxCmd, maxCmd);
    if (safeDelay(300)) return;
    motors.setSpeeds(0, 0);
    if (safeDelay(80)) return;
  }

  motors.setSpeeds(-220, -220);
  if (safeDelay(160)) return;
  motors.setSpeeds(0, 0);
  if (safeDelay(50)) return;

  if (random(0, 2) == 0) motors.setSpeeds(220, -220);
  else                    motors.setSpeeds(-220, 220);
  if (safeDelay(220 + random(0, 120))) return;

  motors.setSpeeds(0, 0);
  currentState = search;
}

void checkEvadeRing(int ringStatus) {
  if (millis() - lastEvadeAt < EVADE_COOLDOWN) return;
  if (ANY_TARGET) { ringConfirmCount = 0; return; }

  if (ringStatus != 99) {
    ringConfirmCount++;
    if (ringConfirmCount >= RING_CONFIRM_LIMIT) {
      ringConfirmCount = 0;
      evadeDirection = ringStatus;
      currentState = evadeRing;
      currentEvadeState = EVADE_INIT;
    }
  } else {
    ringConfirmCount = 0;
  }
}

void setup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  lcd.init();
  lcd.clear();
  buttonA.waitForPress();
  lineSensors.initThreeSensors();
  prox.initThreeSensors();
  delay(500);
  autoCalibrateRing();
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  pivotTimer = millis();
  scanTimer  = millis();
  if (GEAR_RATIO == 75) {
    EVADE_BACK_MS    = 220;
    EVADE_TURN_MS    = 700;
    EVADE_FORWARD_MS = 450;
    EVADE_TURN_SPEED = 200;
  } else {
    EVADE_BACK_MS    = 150;
    EVADE_TURN_MS    = 480;
    EVADE_FORWARD_MS = 360;
    EVADE_TURN_SPEED = 220;
  }
}

void loop() {
  // ── 1. Fresh sensor poll (line sensors + proximity together) ──────────────
  // readProxSensors() is called HERE so frontLeft/frontRight/sideLeft/sideRight
  // are always current before any decision is made. Previously, prox.read()
  // was called once at the top but getDebouncedRingDirection() then blocked
  // for ~20 ms before the side-sensor branches were ever reached, making those
  // readings stale and causing missed detections.
  readProxSensors();
  lineSensors.read(rawSensorValues);

  int ringStatus = checkRing(rawSensorValues);

  lcd.gotoXY(0, 0);
  lcd.print((int)currentState);
  lcd.print(" F"); lcd.print(frontLeft + frontRight);
  lcd.print(" R"); lcd.print(ringStatus);
  lcd.gotoXY(0, 1);
  lcd.print(rawSensorValues[0]); lcd.print(" ");
  lcd.print(rawSensorValues[1]); lcd.print(" ");
  lcd.print(rawSensorValues[2]);

  // ── 2. Ring evade trigger (outside evadeRing state only) ──────────────────
  if (currentState != evadeRing) {
    checkEvadeRing(ringStatus);
  }

  // ── 3. FSM ────────────────────────────────────────────────────────────────
  switch (currentState) {

    // ── evadeRing ────────────────────────────────────────────────────────────
    case evadeRing:
      switch (currentEvadeState) {
        case EVADE_INIT:
          lcd.gotoXY(10, 0); lcd.print("EV");
          motors.setSpeeds(-200, -200);
          evadeTimer = millis();
          currentEvadeState = EVADE_BACK;
          break;
        case EVADE_BACK:
          if (millis() - evadeTimer >= EVADE_BACK_MS) {
            if      (evadeDirection == -1) motors.setSpeeds( EVADE_TURN_SPEED, -EVADE_TURN_SPEED);
            else if (evadeDirection ==  1) motors.setSpeeds(-EVADE_TURN_SPEED,  EVADE_TURN_SPEED);
            else                           motors.setSpeeds( EVADE_TURN_SPEED, -EVADE_TURN_SPEED);
            evadeTimer = millis();
            currentEvadeState = EVADE_TURN;
          }
          break;
        case EVADE_TURN:
          if (millis() - evadeTimer >= EVADE_TURN_MS) {
            motors.setSpeeds(200, 200);
            evadeTimer = millis();
            currentEvadeState = EVADE_FORWARD;
          }
          break;
        case EVADE_FORWARD:
          if (millis() - evadeTimer >= EVADE_FORWARD_MS) {
            motors.setSpeeds(0, 0);
            lastEvadeAt = millis();
            currentState = search;
          }
          break;
      }
      break;

    // ── idle ─────────────────────────────────────────────────────────────────
    case idle:
      motors.setSpeeds(0, 0);
      currentState = search;
      break;

    // ── search ───────────────────────────────────────────────────────────────
    // Tracking logic is a direct port of the reference sketch, with ring-edge
    // safety guards added around each motor command.
    case search:
    {
      // Ring edge: debounced check first (highest priority)
      {
        int d = getDebouncedRingDirection();
        if (d != 99) {
          motors.setSpeeds(0, 0);
          evadeDirection = d; currentEvadeState = EVADE_INIT; currentState = evadeRing;
          break;
        }
      }

      uint8_t maxFront = max(frontLeft, frontRight);

      // ── FRONT tracking (reference code, verbatim logic) ──────────────────
      if (frontLeft > 0 || frontRight > 0)
      {
        int variableBaseSpeed = map(maxFront, 1, 6, 400, 100);

        int rawDiff       = (int)frontLeft - (int)frontRight;
        int error         = rawDiff * 5;
        int speedDiff     = error * 50;

        int leftSpeed  = constrain(variableBaseSpeed - speedDiff, -400, 400);
        int rightSpeed = constrain(variableBaseSpeed + speedDiff, -400, 400);

        {
          int d = getDebouncedRingDirection();
          if (d != 99) { motors.setSpeeds(0,0); evadeDirection=d; currentEvadeState=EVADE_INIT; currentState=evadeRing; break; }
        }

        motors.setSpeeds(leftSpeed, rightSpeed);

        if (maxFront >= COMMIT_THRESHOLD) {
          frontConfirmCount = 0;
          performRam();
        } else if (maxFront > 0) {
          if (++frontConfirmCount >= FRONT_COMMIT_LOOPS) {
            frontConfirmCount = 0;
            performRam();
          }
        } else {
          frontConfirmCount = 0;
        }
      }

      // ── SIDE tracking (reference code, verbatim logic) ───────────────────
      // sideLeft  → pivot LEFT  to face target: left motor back, right forward
      // sideRight → pivot RIGHT to face target: left motor forward, right back
      else if (sideLeft > 0)
      {
        {
          int d = getDebouncedRingDirection();
          if (d != 99) { motors.setSpeeds(0,0); evadeDirection=d; currentEvadeState=EVADE_INIT; currentState=evadeRing; break; }
        }
        motors.setSpeeds(-200, 200);
      }
      else if (sideRight > 0)
      {
        {
          int d = getDebouncedRingDirection();
          if (d != 99) { motors.setSpeeds(0,0); evadeDirection=d; currentEvadeState=EVADE_INIT; currentState=evadeRing; break; }
        }
        motors.setSpeeds(200, -200);
      }

      // ── SEARCH sweep ─────────────────────────────────────────────────────
      else
      {
        unsigned long now = millis();
        if (now - scanTimer > arcTime) { scanDirection *= -1; scanTimer = now; }

        {
          int d = getDebouncedRingDirection();
          if (d != 99) { motors.setSpeeds(0,0); evadeDirection=d; currentEvadeState=EVADE_INIT; currentState=evadeRing; break; }
        }

        motors.setSpeeds(scanDirection == 1 ? 260 : 190,
                         scanDirection == 1 ? 190 : 260);
      }
      break;
    }

    // ── dodge ────────────────────────────────────────────────────────────────
    case dodge:
    {
      if (sideLeft > sideRight) motors.setSpeeds( dodgeSpeed, -dodgeSpeed);
      else                      motors.setSpeeds(-dodgeSpeed,  dodgeSpeed);
      if (safeDelay(200)) break;
      motors.setSpeeds(200, 200);
      if (safeDelay(250)) break;
      motors.setSpeeds(0, 0);
      currentState = search;
      break;
    }

    // ── ram ──────────────────────────────────────────────────────────────────
    case ram:
      performRam();
      break;
  }
}
