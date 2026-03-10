#include <Arduino.h>
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

// ─────────────────────────────────────────────
//  Forward declarations
// ─────────────────────────────────────────────
int  checkRing(uint16_t raw[]);
void autoCalibrateRing();
void performRam();
void checkEvadeRing(int ringStatus);
void pollSensors();
void updateLCD(int ringStatus);

void handleEvadeRing();
void handleIdle();
void handleSearch(int ringStatus);
void handleDodge();
void handleRam();

void handleEvadeInit();
void handleEvadeBack();
void handleEvadeTurn();
void handleEvadeForward();

// ─────────────────────────────────────────────
//  Constants & globals
// ─────────────────────────────────────────────
const uint8_t SEARCH_THRESHOLD     = 3;
const uint8_t COMMIT_THRESHOLD     = 3;
const int     ENCODER_STALL_THRESHOLD = 2;
const int     pushSpeed             = 380;
const int     dodgeSpeed            = 200;

int           scanDirection         = 1;
unsigned long scanTimer             = 0;
unsigned long pivotTimer            = 0;
const unsigned long arcTime         = 900;
const unsigned long pivotTime       = 220;
const unsigned long pivotInterval   = 3000;
const float   collisionThreshold    = 20000;

uint8_t left = 0, front = 0, right = 0;
uint8_t ringConfirmCount            = 0;
const uint8_t RING_CONFIRM_LIMIT    = 3;

uint16_t rawSensorValues[3];
uint16_t ringThresholds[3];

// Evade timing
const unsigned long EVADE_COOLDOWN  = 1500;
unsigned long lastEvadeAt           = 0;
const unsigned long EVADE_BACK_MS   = 180;
const unsigned long EVADE_TURN_MS   = 300;
const unsigned long EVADE_FORWARD_MS = 350;

enum State { idle, search, dodge, ram, evadeRing };
State currentState = idle;

enum EvadeState { EVADE_INIT, EVADE_BACK, EVADE_TURN, EVADE_FORWARD };
EvadeState    currentEvadeState = EVADE_INIT;
unsigned long evadeTimer        = 0;
int           evadeDirection    = 0; // -1 left | 1 right | 0 center


// ─────────────────────────────────────────────
//  Auxiliary functions
// ─────────────────────────────────────────────

// Returns -1 (left edge), 0 (center edge), 1 (right edge), or 99 (on ring).
int checkRing(uint16_t raw[]) {
  const int DROP_DELTA = 220;
  const int HYST       = 40;

  if (ringThresholds[0] == 0 && ringThresholds[1] == 0 && ringThresholds[2] == 0)
    return 99;

  if ((int)ringThresholds[0] - (int)raw[0] > DROP_DELTA + HYST) return -1;
  if ((int)ringThresholds[2] - (int)raw[2] > DROP_DELTA + HYST) return  1;
  if ((int)ringThresholds[1] - (int)raw[1] > DROP_DELTA + HYST) return  0;

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

  for (int i = 0; i < 3; i++)
    ringThresholds[i] = sum[i] / samples;
}

// Polls all proximity and line sensors into globals.
void pollSensors() {
  prox.read();
  lineSensors.read(rawSensorValues);
  left  = prox.countsLeftWithLeftLeds()  + prox.countsLeftWithRightLeds();
  front = prox.countsFrontWithLeftLeds() + prox.countsFrontWithRightLeds();
  right = prox.countsRightWithLeftLeds() + prox.countsRightWithRightLeds();
}

void updateLCD(int ringStatus) {
  lcd.gotoXY(0, 0);
  lcd.print((int)currentState);
  lcd.print(" F"); lcd.print(front);
  lcd.print(" R"); lcd.print(ringStatus);

  lcd.gotoXY(0, 1);
  lcd.print(rawSensorValues[0]); lcd.print(" ");
  lcd.print(rawSensorValues[1]); lcd.print(" ");
  lcd.print(rawSensorValues[2]);
}

// Pure state setter — no delays, no motor commands.
void checkEvadeRing(int ringStatus) {
  if (millis() - lastEvadeAt < EVADE_COOLDOWN) return;

  if (ringStatus != 99) {
    ringConfirmCount++;
    if (ringConfirmCount >= RING_CONFIRM_LIMIT) {
      ringConfirmCount   = 0;
      evadeDirection     = ringStatus;
      currentState       = evadeRing;
      currentEvadeState  = EVADE_INIT;
    }
  } else {
    ringConfirmCount = 0;
  }
}

// Drives forward under PID and rams the opponent.
void performRam() {
  lcd.gotoXY(10, 0);
  lcd.print("RM");

  motors.setSpeeds(-200, -200);
  delay(50);

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  const long         targetCounts = 600;
  const unsigned long maxMs       = 1000;
  const int          maxSpeed     = pushSpeed;
  const float        Kp           = 1.0f;
  const int          minForward   = 150;
  const int          maxCmd       = 400;

  unsigned long start = millis();
  motors.setSpeeds(maxSpeed, maxSpeed);

  long startL = encoders.getCountsLeft();
  long startR = encoders.getCountsRight();

  while (millis() - start < maxMs) {
    delay(20);

    long l = abs(encoders.getCountsLeft()  - startL);
    long r = abs(encoders.getCountsRight() - startR);

    if (l >= targetCounts || r >= targetCounts) break;

    // Safety: check ring edge mid-ram
    uint16_t ramRaw[3];
    lineSensors.read(ramRaw);
    int ringDuringRam = checkRing(ramRaw);

    if (ringDuringRam != 99) {
      motors.setSpeeds(0, 0);
      delay(30);
      motors.setSpeeds(-200, -200);
      delay(120);
      if      (ringDuringRam == -1) motors.setSpeeds( 200, -200);
      else if (ringDuringRam ==  1) motors.setSpeeds(-200,  200);
      else                          motors.setSpeeds( 200, -200);
      delay(150);
      motors.setSpeeds(0, 0);
      currentState = search;
      return;
    }

    float adj        = Kp * (l - r);
    int rawLeftCmd   = constrain((int)(maxSpeed - adj), minForward, maxCmd);
    int rawRightCmd  = constrain((int)(maxSpeed + adj), minForward, maxCmd);
    motors.setSpeeds(rawLeftCmd, rawRightCmd);
  }

  motors.setSpeeds(maxSpeed, maxSpeed);
  delay(200);
  motors.setSpeeds(0, 0);
  delay(50);
  motors.setSpeeds(-150, -150);
  delay(100);
  motors.setSpeeds(0, 0);
  delay(50);

  if (random(0, 2) == 0) motors.setSpeeds( 180, -180);
  else                    motors.setSpeeds(-180,  180);
  delay(120);

  motors.setSpeeds(0, 0);
  currentState = search;
}


// ─────────────────────────────────────────────
//  Evade sub-state handlers
// ─────────────────────────────────────────────

void handleEvadeInit() {
  lcd.gotoXY(10, 0);
  lcd.print("EV");
  motors.setSpeeds(-200, -200);
  evadeTimer        = millis();
  currentEvadeState = EVADE_BACK;
}

void handleEvadeBack() {
  if (millis() - evadeTimer < EVADE_BACK_MS) return;

  if      (evadeDirection == -1) motors.setSpeeds( 200, -200);
  else if (evadeDirection ==  1) motors.setSpeeds(-200,  200);
  else                           motors.setSpeeds( 200, -200);

  evadeTimer        = millis();
  currentEvadeState = EVADE_TURN;
}

void handleEvadeTurn() {
  if (millis() - evadeTimer < EVADE_TURN_MS) return;

  motors.setSpeeds(200, 200);
  evadeTimer        = millis();
  currentEvadeState = EVADE_FORWARD;
}

void handleEvadeForward() {
  if (millis() - evadeTimer < EVADE_FORWARD_MS) return;

  motors.setSpeeds(0, 0);
  lastEvadeAt  = millis();
  currentState = search;
}


// ─────────────────────────────────────────────
//  Primary state handlers
// ─────────────────────────────────────────────

void handleEvadeRing() {
  switch (currentEvadeState) {
    case EVADE_INIT:    handleEvadeInit();    break;
    case EVADE_BACK:    handleEvadeBack();    break;
    case EVADE_TURN:    handleEvadeTurn();    break;
    case EVADE_FORWARD: handleEvadeForward(); break;
  }
}

void handleIdle() {
  motors.setSpeeds(0, 0);
  currentState = search;
}

void handleSearch(int ringStatus) {
  if (ringStatus != 99) return;

  if (front >= SEARCH_THRESHOLD) {
    motors.setSpeeds(160, 160);
    delay(120);
    pollSensors();

    if (front >= COMMIT_THRESHOLD) performRam();
    else                           motors.setSpeeds(0, 0);
    return;
  }

  if (left  >= SEARCH_THRESHOLD) { motors.setSpeeds(-120, 200); delay(120); }
  if (right >= SEARCH_THRESHOLD) { motors.setSpeeds( 200,-120); delay(120); }

  unsigned long now = millis();

  if (now - scanTimer > arcTime) {
    scanDirection *= -1;
    scanTimer = now;
  }

  if (now - pivotTimer > pivotInterval) {
    motors.setSpeeds(160 * scanDirection, -160 * scanDirection);
    if (now - pivotTimer > pivotInterval + pivotTime)
      pivotTimer = now;
    return;
  }

  if (scanDirection == 1) motors.setSpeeds(200, 160);
  else                    motors.setSpeeds(160, 200);
}

void handleDodge() {
  if (left > right) motors.setSpeeds( dodgeSpeed, -dodgeSpeed);
  else              motors.setSpeeds(-dodgeSpeed,  dodgeSpeed);
  delay(200);
  motors.setSpeeds(200, 200);
  delay(250);
  currentState = search;
}

void handleRam() {
  performRam();
}


// ─────────────────────────────────────────────
//  Setup & loop
// ─────────────────────────────────────────────

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
}

void loop() {
  // 1. Single sensor poll per cycle
  pollSensors();
  int ringStatus = checkRing(rawSensorValues);

  // 2. Update display
  updateLCD(ringStatus);

  // 3. Check for ring-edge trigger (suppressed while already evading)
  if (currentState != evadeRing)
    checkEvadeRing(ringStatus);

  // 4. FSM dispatch
  switch (currentState) {
    case evadeRing: handleEvadeRing();         break;
    case idle:      handleIdle();              break;
    case search:    handleSearch(ringStatus);  break;
    case dodge:     handleDodge();             break;
    case ram:       handleRam();               break;
  }
}
