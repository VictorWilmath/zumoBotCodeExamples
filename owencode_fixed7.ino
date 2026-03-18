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
int  checkRing(uint16_t raw[]);
void autoCalibrateRing();
void performRam();
void checkEvadeRing(int ringStatus);

// ── Tuning constants ─────────────────────────────────────────────────────────
const uint8_t  COMMIT_THRESHOLD   = 3;    // prox count to trigger immediate ram

// With the alignment gate in place, only 1 consecutive centred reading is
// needed before committing. The alignment check itself is the quality gate —
// FRONT_COMMIT_LOOPS now just guards against a single-sample noise spike.
const uint8_t  FRONT_COMMIT_LOOPS = 1;

// How centred the target must be before a ram is allowed.
// rawDiff = frontLeft - frontRight, range [-6, 6].
// 0 = perfectly centred. Allow ±1 so a single-count asymmetry still rams.
// Raise to 2 if the bot over-waits; lower to 0 to require perfect centering.
const uint8_t  ALIGN_THRESHOLD    = 1;
const int      pushSpeed          = 400;
const int      dodgeSpeed         = 200;

// Side pivot speed. Higher = faster turn to face a side target.
// Safe to raise now that the alignment gate prevents premature rams on overshoot.
const int      PIVOT_SPEED        = 350;

// Search sweep
int            scanDirection      = 1;
unsigned long  scanTimer          = 0;
const unsigned long arcTime       = 900;

const float    collisionThreshold = 20000;

// ── Proximity sensor readings ─────────────────────────────────────────────────
// Single-LED reads matching the reference sketch — see readProxSensors().
uint8_t frontLeft  = 0;
uint8_t frontRight = 0;
uint8_t sideLeft   = 0;
uint8_t sideRight  = 0;

#define ANY_FRONT  (frontLeft > 0 || frontRight > 0)
#define ANY_SIDE   (sideLeft  > 0 || sideRight  > 0)
#define ANY_TARGET (ANY_FRONT || ANY_SIDE)

uint8_t  ringConfirmCount  = 0;
const uint8_t RING_CONFIRM_LIMIT = 3;
uint8_t  frontConfirmCount = 0;

uint16_t rawSensorValues[3];
uint16_t ringThresholds[3];

// ── Evade FSM ─────────────────────────────────────────────────────────────────
const unsigned long EVADE_COOLDOWN = 1500;
unsigned long lastEvadeAt  = 0;
unsigned long EVADE_BACK_MS    = 180;
unsigned long EVADE_TURN_MS    = 300;
unsigned long EVADE_FORWARD_MS = 350;
int           EVADE_TURN_SPEED = 200;

#ifndef GEAR_RATIO
#define GEAR_RATIO 75
#endif

enum State      { idle, search, dodge, ram, evadeRing };
State currentState = idle;

enum EvadeState { EVADE_INIT, EVADE_BACK, EVADE_TURN, EVADE_FORWARD };
EvadeState currentEvadeState = EVADE_INIT;
unsigned long evadeTimer  = 0;
int           evadeDirection = 0;

// ── readProxSensors ───────────────────────────────────────────────────────────
// Matches the reference sketch exactly: each variable is a single-LED read.
//
// Why revert from summed LEDs:
//   - Summing both LEDs for sideLeft/sideRight introduced cross-talk noise from
//     the opposite LED bouncing off the floor, raising the noise floor and
//     producing phantom side detections.
//   - Summing both front LEDs doubled frontLeft/frontRight values, keeping
//     ANY_FRONT true longer than intended and blocking the side branch from
//     ever firing during front-to-side transitions.
//   - The reference steering formula (rawDiff * 5 * 50) is designed around
//     single-LED reads where max rawDiff = 6. Doubled values broke that math.
// Single-LED reads are what the Zumo library is characterised against.
void readProxSensors() {
  prox.read();
  frontLeft  = prox.countsFrontWithLeftLeds();
  frontRight = prox.countsFrontWithRightLeds();
  sideLeft   = prox.countsLeftWithLeftLeds();
  sideRight  = prox.countsRightWithRightLeds();
}

// ── checkRing ─────────────────────────────────────────────────────────────────
int checkRing(uint16_t raw[]) {
  // Sensor value map (approximate):
  //   Black mat (baseline) : ~1200-1300
  //   Starting lines       : ~1000        ← must NOT trigger
  //   White ring edge      : ~150         ← must trigger
  //
  // Old DROP_DELTA=220 + HYST=40 = 260 total, firing below ~990 — starting
  // lines sat right at that boundary and caused false positives.
  //
  // New DROP_DELTA=400 + HYST=50 = 450 total, firing below ~800.
  // That is well below the starting lines (~1000) and well above the white
  // ring (~150), so only a genuine ring edge triggers detection.
  const int DROP_DELTA = 400;
  const int HYST       = 50;
  if (ringThresholds[0] == 0 && ringThresholds[1] == 0 && ringThresholds[2] == 0)
    return 99;
  if ((int)ringThresholds[0] - (int)raw[0] > DROP_DELTA + HYST) return -1;
  if ((int)ringThresholds[2] - (int)raw[2] > DROP_DELTA + HYST) return  1;
  if ((int)ringThresholds[1] - (int)raw[1] > DROP_DELTA + HYST) return  0;
  return 99;
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

// ── getDebouncedRingDirection ─────────────────────────────────────────────────
// ONLY called when no target is visible — so the 20 ms blocking cost never
// delays a tracking or ram decision.
int getDebouncedRingDirection(uint8_t samples = 5, uint8_t required = 3) {
  // Caller must already have confirmed ANY_TARGET == false before calling this.
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
  if (counts[1] >= required) return  0;
  if (counts[0] >= required) return -1;
  if (counts[2] >= required) return  1;
  return 99;
}

// ── safeDelay ────────────────────────────────────────────────────────────────
bool safeDelay(unsigned long ms) {
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
      evadeDirection    = edge;
      currentEvadeState = EVADE_INIT;
      currentState      = evadeRing;
      return true;
    }
    delay(5);
  }
  return false;
}

// ── performRam ───────────────────────────────────────────────────────────────
// FIX: Removed the opening reverse tap — it was firing on every loop iteration
// where maxFront >= COMMIT_THRESHOLD, making the bot repeatedly reverse while
// the opponent was still right in front of it.
//
// FIX: Replaced the 2-burst-with-full-stop sequence with a single continuous
// sustained push loop. The previous full stops between bursts caused the visible
// "disengage" mid-fight.
//
// FIX: After the push, check for a live opponent before pivoting away. If the
// opponent is still detected, skip the random pivot and go straight back to
// search tracking instead of spinning off into empty space.
void performRam() {
  lcd.gotoXY(10, 0);
  lcd.print("RM");

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  const long         targetCounts = 700;
  const unsigned long maxMs       = 1200;
  const int          maxSpeed     = pushSpeed;
  const float        Kp           = 1.2f;
  const int          minForward   = 180;
  const int          maxCmd       = 400;

  unsigned long start = millis();
  long startL = encoders.getCountsLeft();
  long startR = encoders.getCountsRight();

  motors.setSpeeds(maxSpeed, maxSpeed);

  // ── Sustained push loop ───────────────────────────────────────────────────
  // Polls ring on every iteration. No stops, no pauses.
  // Straight-line encoder correction keeps the push square.
  while (millis() - start < maxMs) {
    uint16_t ramRaw[3];
    lineSensors.read(ramRaw);
    if (checkRing(ramRaw) != 99) {
      motors.setSpeeds(0, 0);
      evadeDirection    = checkRing(ramRaw);
      currentEvadeState = EVADE_INIT;
      currentState      = evadeRing;
      return;
    }

    long l = abs(encoders.getCountsLeft()  - startL);
    long r = abs(encoders.getCountsRight() - startR);
    if (l >= targetCounts || r >= targetCounts) break;

    float adj       = Kp * (float)(l - r);
    int leftCmd     = constrain(maxSpeed - (int)adj, minForward, maxCmd);
    int rightCmd    = constrain(maxSpeed + (int)adj, minForward, maxCmd);
    motors.setSpeeds(leftCmd, rightCmd);
    delay(4);
  }

  // ── Final sustained shove ─────────────────────────────────────────────────
  // One continuous burst — no stop in the middle.
  motors.setSpeeds(maxCmd, maxCmd);
  if (safeDelay(500)) return;

  // ── Back off ──────────────────────────────────────────────────────────────
  motors.setSpeeds(-220, -220);
  if (safeDelay(140)) return;
  motors.setSpeeds(0, 0);

  // ── FIX: check for opponent before pivoting ───────────────────────────────
  // Re-read sensors. If the opponent is still visible, return to search
  // immediately so the tracking loop re-acquires and rams again rather than
  // spinning away into empty space.
  readProxSensors();
  if (ANY_TARGET) {
    currentState = search;
    return;
  }

  // Opponent not visible — short random pivot to reposition, then search.
  if (random(0, 2) == 0) motors.setSpeeds(220, -220);
  else                    motors.setSpeeds(-220,  220);
  if (safeDelay(180 + random(0, 80))) return;

  motors.setSpeeds(0, 0);
  currentState = search;
}

// ── checkEvadeRing ────────────────────────────────────────────────────────────
void checkEvadeRing(int ringStatus) {
  if (millis() - lastEvadeAt < EVADE_COOLDOWN) return;
  if (ANY_TARGET) { ringConfirmCount = 0; return; }

  if (ringStatus != 99) {
    if (++ringConfirmCount >= RING_CONFIRM_LIMIT) {
      ringConfirmCount  = 0;
      evadeDirection    = ringStatus;
      currentState      = evadeRing;
      currentEvadeState = EVADE_INIT;
    }
  } else {
    ringConfirmCount = 0;
  }
}

// ── setup ─────────────────────────────────────────────────────────────────────
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
  scanTimer = millis();
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

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // 1. Always get fresh sensor data first.
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

  // 2. Ring evade trigger (skipped while already evading).
  if (currentState != evadeRing) checkEvadeRing(ringStatus);

  // 3. FSM.
  switch (currentState) {

    // ── evadeRing ─────────────────────────────────────────────────────────────
    case evadeRing:
      switch (currentEvadeState) {
        case EVADE_INIT:
          lcd.gotoXY(10, 0); lcd.print("EV");
          motors.setSpeeds(-200, -200);
          evadeTimer        = millis();
          currentEvadeState = EVADE_BACK;
          break;
        case EVADE_BACK:
          if (millis() - evadeTimer >= EVADE_BACK_MS) {
            if      (evadeDirection == -1) motors.setSpeeds( EVADE_TURN_SPEED, -EVADE_TURN_SPEED);
            else if (evadeDirection ==  1) motors.setSpeeds(-EVADE_TURN_SPEED,  EVADE_TURN_SPEED);
            else                           motors.setSpeeds( EVADE_TURN_SPEED, -EVADE_TURN_SPEED);
            evadeTimer        = millis();
            currentEvadeState = EVADE_TURN;
          }
          break;
        case EVADE_TURN:
          if (millis() - evadeTimer >= EVADE_TURN_MS) {
            motors.setSpeeds(200, 200);
            evadeTimer        = millis();
            currentEvadeState = EVADE_FORWARD;
          }
          break;
        case EVADE_FORWARD:
          if (millis() - evadeTimer >= EVADE_FORWARD_MS) {
            motors.setSpeeds(0, 0);
            lastEvadeAt  = millis();
            currentState = search;
          }
          break;
      }
      break;

    // ── idle ──────────────────────────────────────────────────────────────────
    case idle:
      motors.setSpeeds(0, 0);
      currentState = search;
      break;

    // ── search ────────────────────────────────────────────────────────────────
    case search:
    {
      // ── Ring edge safety (only when no target visible) ────────────────────
      // Fast path: single raw read first. Only spend the full 20 ms debounce
      // if the fast read already shows a potential edge — keeps loop latency
      // near zero when the bot is safely in the middle of the ring.
      if (!ANY_TARGET) {
        uint16_t quickRaw[3];
        lineSensors.read(quickRaw);
        if (checkRing(quickRaw) != 99) {
          // Potential edge — confirm with full debounce before committing.
          int d = getDebouncedRingDirection();
          if (d != 99) {
            motors.setSpeeds(0, 0);
            evadeDirection = d; currentEvadeState = EVADE_INIT; currentState = evadeRing;
            break;
          }
        }
      }

      uint8_t maxFront = max(frontLeft, frontRight);

      // ── FRONT tracking ────────────────────────────────────────────────────
      if (ANY_FRONT)
      {
        // Raised minimum base speed from 100 to 200: at close range the old
        // floor left the bot barely creeping forward while still steering,
        // stalling the approach. 200 keeps forward momentum at all distances.
        int variableBaseSpeed = map(maxFront, 1, 6, 400, 200);

        int rawDiff   = (int)frontLeft - (int)frontRight;
        int speedDiff = rawDiff * 5 * 50;

        int leftSpeed  = constrain(variableBaseSpeed - speedDiff, -400, 400);
        int rightSpeed = constrain(variableBaseSpeed + speedDiff, -400, 400);

        motors.setSpeeds(leftSpeed, rightSpeed);

        bool aligned = (abs(rawDiff) <= ALIGN_THRESHOLD);

        if (!aligned) {
          frontConfirmCount = 0;
        } else if (maxFront >= COMMIT_THRESHOLD) {
          frontConfirmCount = 0;
          performRam();
        } else {
          if (++frontConfirmCount >= FRONT_COMMIT_LOOPS) {
            frontConfirmCount = 0;
            performRam();
          }
        }
      }

      // ── SIDE tracking ────────────────────────────────────────────────────
      // Raised pivot speed from 200 to PIVOT_SPEED (350). The alignment gate
      // now prevents premature rams from overshoot, so fast pivots are safe.
      else if (sideLeft > 0 && sideRight > 0)
      {
        frontConfirmCount = 0;
        if (sideLeft >= sideRight) motors.setSpeeds(-PIVOT_SPEED,  PIVOT_SPEED);
        else                       motors.setSpeeds( PIVOT_SPEED, -PIVOT_SPEED);
      }
      else if (sideLeft > 0)
      {
        frontConfirmCount = 0;
        motors.setSpeeds(-PIVOT_SPEED, PIVOT_SPEED);
      }
      else if (sideRight > 0)
      {
        frontConfirmCount = 0;
        motors.setSpeeds(PIVOT_SPEED, -PIVOT_SPEED);
      }

      // ── SEARCH sweep ──────────────────────────────────────────────────────
      // FIX: also reset here so a count from a previous detection that ended
      // mid-loop doesn't carry over into the next detection event.
      else
      {
        frontConfirmCount = 0;
        unsigned long now = millis();
        if (now - scanTimer > arcTime) { scanDirection *= -1; scanTimer = now; }
        motors.setSpeeds(scanDirection == 1 ? 260 : 190,
                         scanDirection == 1 ? 190 : 260);
      }
      break;
    }

    // ── dodge ─────────────────────────────────────────────────────────────────
    case dodge:
    {
      if (sideLeft >= sideRight) motors.setSpeeds( dodgeSpeed, -dodgeSpeed);
      else                       motors.setSpeeds(-dodgeSpeed,  dodgeSpeed);
      if (safeDelay(200)) break;
      motors.setSpeeds(200, 200);
      if (safeDelay(250)) break;
      motors.setSpeeds(0, 0);
      currentState = search;
      break;
    }

    // ── ram ───────────────────────────────────────────────────────────────────
    case ram:
      performRam();
      break;
  }
}
