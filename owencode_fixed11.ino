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

// ── Gear-ratio-dependent variables ───────────────────────────────────────────
// 50:1 motors spin ~1.5× faster than 75:1 at the same PWM value.
// All values set in setup() based on GEAR_RATIO.

int PIVOT_SPEED = 350;

// Search sweep arc duration.
unsigned long ARC_TIME = 900;

// Post-ram forward arc: instead of backing up, the bot arcs around the
// opponent's side using differential speed. The inner wheel runs at ARC_INNER
// while the outer wheel runs at full maxCmd. This keeps the bot moving forward
// (never reversing) and re-attacks from a new angle.
// In a 30-inch ring there is no safe space to reverse — every mm of reverse
// is a free gift of push distance to the opponent.
int           ARC_INNER_SPEED  = 0;   // inner wheel speed during arc (set in setup)
unsigned long ARC_DURATION_MS  = 0;   // how long to hold the arc

// Emergency ring-edge reverse: minimum distance to clear the white ring sensor.
// Kept as short as physically possible — the 400ms cap in the old safeDelay
// was extremely dangerous in a 762mm ring where full-speed reverse covers the
// entire ring in ~1.9 seconds.
unsigned long EDGE_REVERSE_MS  = 0;

// Stall detection (50:1 only, short-window per-ram).
unsigned long STALL_WINDOW_MS    = 0;
long          STALL_COUNT_THRESH = 0;
unsigned long STALL_BREAK_MS     = 0;

// ── Stalemate detection (both gear ratios) ────────────────────────────────────
// A 'stalemate' is defined as: sustained front contact for 15+ seconds with
// net encoder displacement below STALEMATE_DISP_THRESH over that entire window.
// This is distinct from the short-window stall check (350ms) — the bots may
// be slowly drifting or rocking but making no real progress toward winning.
//
// Detection uses two signals:
//   1. Encoder net displacement: |currentPos - posAtContactStart| < threshold
//      over the full 15s window. Small drifts within the window are ignored
//      because we compare against the START position, not the last snapshot.
//   2. IMU sustained contact confirmation: average |imu.a.x| above a floor
//      confirms the two bots are actually pressing against each other, not
//      just that our bot is sitting idle during a slow search.
//
// On trigger: performStalemateBreak() executes a full disengage-and-reposition
// using the 50:1 speed advantage to sprint to the far side and re-attack with
// kinetic energy from a running start.
const unsigned long STALEMATE_WINDOW_MS    = 15000UL; // 15 seconds
const long          STALEMATE_DISP_THRESH  = 80;      // encoder counts — ~small drift OK
const long          STALEMATE_IMU_FLOOR    = 3000;    // sustained |a.x| floor for contact

unsigned long contactStartTime   = 0;   // millis() when front contact first detected
long          contactStartEncL   = 0;   // encoder snapshot at contact start
long          contactStartEncR   = 0;
bool          inSustainedContact = false;
long          imuAccumulator     = 0;   // running sum of |imu.a.x| samples
uint16_t      imuSampleCount     = 0;

// Forward declaration
void performStalemateBreak();

const float    collisionThreshold = 20000;

// Search sweep
int            scanDirection = 1;
unsigned long  scanTimer     = 0;

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
#define GEAR_RATIO 50
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
// Polls line sensors every 5ms. On edge detection: stops, reverses for
// EDGE_REVERSE_MS (just enough to clear the white ring), then hands off to
// evadeRing FSM. The old 400ms max reverse was lethal in a 762mm ring —
// at full speed the bot could cover nearly the entire ring diameter.
bool safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    uint16_t sd_raw[3];
    lineSensors.read(sd_raw);
    int edge = checkRing(sd_raw);
    if (edge != 99) {
      motors.setSpeeds(0, 0);
      // Reverse only long enough to physically clear the white ring line,
      // then stop and hand off to the FSM turn. EDGE_REVERSE_MS is set per
      // gear ratio in setup() — shorter for 50:1 (faster motors clear faster).
      unsigned long revStart = millis();
      while (millis() - revStart < EDGE_REVERSE_MS) {
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

// ── performStalemateBreak ─────────────────────────────────────────────────────
// Called when 15 seconds of sustained contact with no net progress is detected.
// Strategy: full disengage → sprint to far side of ring → 180° turn → running
// start re-attack. This exploits 50:1 speed to create real separation and hit
// with kinetic energy rather than static torque — breaking the force symmetry
// of the stalemate entirely.
void performStalemateBreak() {
  lcd.gotoXY(10, 0);
  lcd.print("SB");

  const int maxCmd = 400;

  // Reset stalemate tracking immediately.
  inSustainedContact = false;
  imuAccumulator     = 0;
  imuSampleCount     = 0;

  // Step 1: hard pivot ~180° away from the opponent.
  // Use EVADE_TURN_SPEED and EVADE_TURN_MS which are already tuned per gear ratio.
  if (random(0, 2) == 0) motors.setSpeeds( EVADE_TURN_SPEED, -EVADE_TURN_SPEED);
  else                   motors.setSpeeds(-EVADE_TURN_SPEED,  EVADE_TURN_SPEED);
  if (safeDelay(EVADE_TURN_MS)) return;

  // Step 2: full-speed sprint toward the far side of the ring.
  // At 50:1 speed ~400mm/s, 350ms covers ~140mm — enough to cross roughly
  // half the usable ring and create real separation from the opponent.
  motors.setSpeeds(maxCmd, maxCmd);
  if (safeDelay(350)) return;

  // Step 3: pivot 180° again to face back toward the opponent.
  if (random(0, 2) == 0) motors.setSpeeds( EVADE_TURN_SPEED, -EVADE_TURN_SPEED);
  else                   motors.setSpeeds(-EVADE_TURN_SPEED,  EVADE_TURN_SPEED);
  if (safeDelay(EVADE_TURN_MS)) return;

  // Step 4: running start — full speed toward the opponent.
  // The bot will re-enter search immediately; if the opponent is still centred
  // in the front sensor, performRam fires at full speed within one loop tick.
  motors.setSpeeds(maxCmd, maxCmd);
  if (safeDelay(100)) return;

  motors.setSpeeds(0, 0);
  currentState = search;
}

// ── performRam ────────────────────────────────────────────────────────────────
// Core strategy for the 30-inch ring with 10cm bots:
//   - NEVER reverse voluntarily. The ring is only ~762mm across. Every mm of
//     reverse is free push distance gifted to the opponent. Center-to-edge is
//     ~230mm; at opponent's push speed that's under 600ms of free travel.
//   - When stalled or needing to reposition: arc FORWARD around the opponent's
//     side using differential wheel speeds. This keeps pressure on, maintains
//     forward momentum, and attacks from a new angle.
//   - When being pushed to the edge: arc escape rather than pivot-in-place.
//     A stationary pivot gives the opponent a fixed target; an arc is harder
//     to track and maintains some separation from the edge.
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

  unsigned long start  = millis();
  long startL = encoders.getCountsLeft();
  long startR = encoders.getCountsRight();

  unsigned long stallWindowStart = millis();
  long stallSnapL = startL;
  long stallSnapR = startR;

  motors.setSpeeds(maxSpeed, maxSpeed);

  // ── Sustained push loop ───────────────────────────────────────────────────
  while (millis() - start < maxMs) {
    uint16_t ramRaw[3];
    lineSensors.read(ramRaw);
    int edgeDuringRam = checkRing(ramRaw);

    if (edgeDuringRam != 99) {
      motors.setSpeeds(0, 0);
      readProxSensors();

      if (ANY_FRONT) {
        int arcL = (edgeDuringRam == -1) ? ARC_INNER_SPEED : maxCmd;
        int arcR = (edgeDuringRam == -1) ? maxCmd           : ARC_INNER_SPEED;
        motors.setSpeeds(arcL, arcR);
        if (safeDelay(ARC_DURATION_MS)) return;
        motors.setSpeeds(0, 0);
        lastEvadeAt  = millis();
        currentState = search;
        return;
      }

      evadeDirection    = edgeDuringRam;
      currentEvadeState = EVADE_INIT;
      currentState      = evadeRing;
      return;
    }

    long curL = encoders.getCountsLeft();
    long curR = encoders.getCountsRight();
    long l    = abs(curL - startL);
    long r    = abs(curR - startR);
    if (l >= targetCounts || r >= targetCounts) break;

    float adj    = Kp * (float)(l - r);
    int leftCmd  = constrain(maxSpeed - (int)adj, minForward, maxCmd);
    int rightCmd = constrain(maxSpeed + (int)adj, minForward, maxCmd);
    motors.setSpeeds(leftCmd, rightCmd);

    // ── Stalemate detection ────────────────────────────────────────────────
    // Two-signal: net encoder displacement from contact start + IMU load.
    // We re-read sensors here only if not already called above (edge path
    // already returned, so we only reach here when edge == 99).
    readProxSensors();
    if (ANY_FRONT) {
      if (!inSustainedContact) {
        // First loop with front contact — snapshot position and time.
        inSustainedContact = true;
        contactStartTime   = millis();
        contactStartEncL   = curL;
        contactStartEncR   = curR;
        imuAccumulator     = 0;
        imuSampleCount     = 0;
      }
      // Accumulate IMU for sustained-contact confirmation.
      imu.read();
      imuAccumulator += (long)abs(imu.a.x);
      imuSampleCount++;

      unsigned long elapsed = millis() - contactStartTime;
      if (elapsed >= STALEMATE_WINDOW_MS) {
        long netDispL  = abs(curL - contactStartEncL);
        long netDispR  = abs(curR - contactStartEncR);
        long avgImu    = (imuSampleCount > 0)
                         ? (imuAccumulator / (long)imuSampleCount) : 0;
        bool noProgress    = (netDispL < STALEMATE_DISP_THRESH &&
                              netDispR < STALEMATE_DISP_THRESH);
        bool sustainedLoad = (avgImu > STALEMATE_IMU_FLOOR);

        if (noProgress && sustainedLoad) {
          // Genuine 15-second stalemate confirmed — break out.
          motors.setSpeeds(0, 0);
          performStalemateBreak();
          return;
        }
        // Bots are drifting significantly, or load is low — not a true
        // stalemate. Reset window and continue.
        contactStartTime = millis();
        contactStartEncL = curL;
        contactStartEncR = curR;
        imuAccumulator   = 0;
        imuSampleCount   = 0;
      }
    } else {
      // Lost front contact — reset stalemate tracking.
      inSustainedContact = false;
      imuAccumulator     = 0;
      imuSampleCount     = 0;
    }

    // ── Short-window stall detection (50:1 only) ───────────────────────────
    if (STALL_WINDOW_MS > 0 && millis() - stallWindowStart >= STALL_WINDOW_MS) {
      long progL = abs(curL - stallSnapL);
      long progR = abs(curR - stallSnapR);
      if (progL < STALL_COUNT_THRESH && progR < STALL_COUNT_THRESH) {
        int breakDir = (random(0, 2) == 0) ? 1 : -1;
        int arcL = (breakDir ==  1) ? maxCmd : ARC_INNER_SPEED;
        int arcR = (breakDir == -1) ? maxCmd : ARC_INNER_SPEED;
        motors.setSpeeds(arcL, arcR);
        if (safeDelay(STALL_BREAK_MS)) return;
        motors.setSpeeds(maxCmd, maxCmd);
        if (safeDelay(150)) return;
        motors.setSpeeds(0, 0);
        currentState = search;
        return;
      }
      stallWindowStart = millis();
      stallSnapL = curL;
      stallSnapR = curR;
    }

    delay(4);
  }

  // ── Final sustained shove ─────────────────────────────────────────────────
  motors.setSpeeds(maxCmd, maxCmd);
  if (safeDelay(500)) return;

  // ── Post-push reposition: forward arc, not reverse ────────────────────────
  // If opponent still visible, arc around them to re-attack from a new angle.
  // If opponent gone, short forward arc to reposition then search.
  // In either case: NO reverse. The ring is too small.
  readProxSensors();
  int arcDir = (random(0, 2) == 0) ? 1 : -1;
  int postArcL = (arcDir ==  1) ? maxCmd : ARC_INNER_SPEED;
  int postArcR = (arcDir == -1) ? maxCmd : ARC_INNER_SPEED;
  motors.setSpeeds(postArcL, postArcR);
  if (safeDelay(ARC_DURATION_MS)) return;

  motors.setSpeeds(0, 0);
  currentState = search;
}

// ── checkEvadeRing ────────────────────────────────────────────────────────────
// Called every loop when not already evading.
// Suppressed when opponent is pushing us from the front — in that case the
// standard evade (reverse → turn) would add our own reverse speed to the push,
// guaranteeing ejection. The being-pushed escape path lives in performRam's
// ring-edge detection instead, where it can execute a contact-breaking turn.
void checkEvadeRing(int ringStatus) {
  if (millis() - lastEvadeAt < EVADE_COOLDOWN) return;

  // If the opponent is in the front sensor AND we're at the ring edge,
  // we are being shoved. Don't start the standard evade — the ram loop's
  // ring check handles this with a contact-breaking turn. Just clear the
  // confirm counter so we don't accidentally trigger evade on the next loop.
  if (ANY_FRONT && ringStatus != 99) {
    ringConfirmCount = 0;
    return;
  }

  // No opponent visible — genuine self-reached edge, or side/rear approach.
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
    // ── 75:1 (slower motors) ─────────────────────────────────────────────────
    EVADE_BACK_MS      = 220;
    EVADE_TURN_MS      = 700;
    EVADE_FORWARD_MS   = 450;
    EVADE_TURN_SPEED   = 200;

    PIVOT_SPEED        = 350;
    ARC_TIME           = 900;
    EDGE_REVERSE_MS    = 200;   // enough to clear white ring at slower speed

    ARC_INNER_SPEED    = 120;   // wide arc for slower bot
    ARC_DURATION_MS    = 400;

    STALL_WINDOW_MS    = 0;
    STALL_COUNT_THRESH = 0;
    STALL_BREAK_MS     = 0;
  } else {
    // ── 50:1 (faster motors, ~1.5× speed at same PWM) ────────────────────────
    EVADE_BACK_MS      = 150;
    EVADE_TURN_MS      = 480;
    EVADE_FORWARD_MS   = 300;
    EVADE_TURN_SPEED   = 180;

    PIVOT_SPEED        = 300;
    ARC_TIME           = 600;

    // Emergency reverse capped at 80ms — 762mm ring means 400ms old cap
    // covered nearly the whole ring at 50:1 speed. 80ms clears the white line.
    EDGE_REVERSE_MS    = 80;

    // Forward arc repositioning: inner wheel at ~30% keeps both wheels moving
    // forward. 300ms sweeps ~90° around the opponent at 50:1 speed.
    ARC_INNER_SPEED    = 120;
    ARC_DURATION_MS    = 300;

    // Stall: < 15 encoder counts over 350ms at full power = pushing war.
    STALL_WINDOW_MS    = 350;
    STALL_COUNT_THRESH = 15;
    STALL_BREAK_MS     = 280;
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
      else
      {
        frontConfirmCount  = 0;
        // No target visible — genuine contact break. Reset stalemate tracking
        // so the 15s window starts fresh on the next engagement.
        inSustainedContact = false;
        imuAccumulator     = 0;
        imuSampleCount     = 0;
        unsigned long now = millis();
        if (now - scanTimer > ARC_TIME) { scanDirection *= -1; scanTimer = now; }
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
