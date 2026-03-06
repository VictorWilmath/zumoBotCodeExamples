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
const unsigned long stallTime = 200;
const float collisionThreshold = 20000;
unsigned long stallStart = 0;
uint8_t left = 0, front = 0, right = 0;
uint8_t ringConfirmCount = 0;
const uint8_t RING_CONFIRM_LIMIT = 3;

uint16_t rawSensorValues[3];
uint16_t blackRaw[3];
uint16_t whiteRaw[3];
uint16_t ringThresholds[3];

// Evade timing and lockout to avoid repeated/oscillating evade actions
const unsigned long EVADE_COOLDOWN = 1500; // ms
unsigned long lastEvadeAt = 0;
const unsigned long EVADE_BACK_MS = 180;
const unsigned long EVADE_TURN_MS = 300;
const unsigned long EVADE_FORWARD_MS = 350;

enum State { idle, search, dodge, ram, evadeRing };
State currentState = idle;

void pushBehavior(uint8_t frontSignal) {
  static long lastLeft = 0, lastRight = 0;
  motors.setSpeeds(pushSpeed, pushSpeed);
  long currentLeft = encoders.getCountsLeft();
  long currentRight = encoders.getCountsRight();
  long deltaLeft = abs(currentLeft - lastLeft);
  long deltaRight = abs(currentRight - lastRight);
  if (frontSignal > COMMIT_THRESHOLD) {
    if (deltaLeft < ENCODER_STALL_THRESHOLD && deltaRight < ENCODER_STALL_THRESHOLD) {
      if (stallStart == 0) stallStart = millis();
      if (millis() - stallStart > stallTime) {
        motors.setSpeeds(-200, -200);
        delay(150);
        motors.setSpeeds(pushSpeed, pushSpeed);
        stallStart = 0;
      }
    } else {
      stallStart = 0;
    }
  }
  lastLeft = currentLeft;
  lastRight = currentRight;
}

int checkRing()
{
  static uint16_t baseline[3];
  static bool baselineInitialized = false;

  uint16_t raw[3];
  lineSensors.read(raw);

  const int DROP_DELTA = 300;   // Increase robustness
  const int HYST = 40;

  // Initialize baseline once
  if (!baselineInitialized)
  {
    for (int i = 0; i < 3; i++)
      baseline[i] = raw[i];

    baselineInitialized = true;
    return 99;
  }

  // Update baseline slowly while on black
  for (int i = 0; i < 3; i++)
  {
    if (raw[i] > baseline[i])  
      baseline[i] = raw[i];    // Track highest black level
  }

  if (baseline[0] - raw[0] > DROP_DELTA + HYST) return -1;
  if (baseline[2] - raw[2] > DROP_DELTA + HYST) return 1;
  if (baseline[1] - raw[1] > DROP_DELTA + HYST) return 0;

  return 99;
}

bool collisionDetected() {
  imu.read();
  return abs(imu.a.x) > collisionThreshold;
}

void autoCalibrateRing()
{
  const uint8_t samples = 25;
  uint32_t sum[3] = {0,0,0};
  uint16_t temp[3];

  for (uint8_t s = 0; s < samples; s++)
  {
    lineSensors.read(temp);
    for (int i = 0; i < 3; i++)
      sum[i] += temp[i];
    delay(15);
  }

  for (int i = 0; i < 3; i++)
  {
    ringThresholds[i] = sum[i] / samples;  // true black baseline
  }
}

 // Performs the full ramming sequence (extracted from the ram case)
 void performRam()
 {
   lcd.gotoXY(10, 0);
   lcd.print("RM");

   // Back up briefly before the charge (shorter to retain momentum)
   motors.setSpeeds(-200, -200);
   delay(50);

   // Reset encoder counts to measure forward travel reliably
   encoders.getCountsAndResetLeft();
   encoders.getCountsAndResetRight();

   const long targetCounts = 600; // counts to travel while ramming (tune)
   const unsigned long maxMs = 1000; // safety timeout
   const int maxSpeed = pushSpeed;
   const float Kp = 1.0f; // proportional gain (tune)
   int leftCmd = maxSpeed;
   int rightCmd = maxSpeed;
   unsigned long start = millis();
   motors.setSpeeds(leftCmd, rightCmd);

   // Minimum forward throttle to avoid reversing one wheel during correction
   const int minForward = 150;
   const int maxCmd = 400;

   // Capture starting encoder counts so we use deltas (robust to sign)
   long startL = encoders.getCountsLeft();
   long startR = encoders.getCountsRight();

   while (millis() - start < maxMs) {
     delay(20);

     long lDelta = encoders.getCountsLeft() - startL;
     long rDelta = encoders.getCountsRight() - startR;

     long l = abs(lDelta);
     long r = abs(rDelta);

     // If we've moved enough, break
     if ((l >= targetCounts) || (r >= targetCounts)) break;

     long err = l - r; // positive => left ahead
     float adj = Kp * err;

     int rawLeftCmd = maxSpeed - (int)adj;
     int rawRightCmd = maxSpeed + (int)adj;

     // Clamp into [minForward, maxCmd] to avoid sign flips
     if (rawLeftCmd > maxCmd) rawLeftCmd = maxCmd;
     if (rawLeftCmd < minForward) rawLeftCmd = minForward;
     if (rawRightCmd > maxCmd) rawRightCmd = maxCmd;
     if (rawRightCmd < minForward) rawRightCmd = minForward;

     leftCmd = rawLeftCmd;
     rightCmd = rawRightCmd;

     motors.setSpeeds(leftCmd, rightCmd);
   }

   // Short full push to finish the contact
   motors.setSpeeds(maxSpeed, maxSpeed);
   delay(200);

   // Stop and back off slightly
   motors.setSpeeds(0, 0);
   delay(50);
   motors.setSpeeds(-150, -150);
   delay(100);
   motors.setSpeeds(0, 0);
   delay(50);

   // Small turn-away to disengage
   if (random(0,2) == 0) motors.setSpeeds(180, -180);
   else motors.setSpeeds(-180, 180);
   delay(120);

   motors.setSpeeds(0,0);
   currentState = search;
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
  scanTimer = millis();
}

void loop() {
  prox.read();
  lineSensors.read(rawSensorValues);
  left  = prox.countsLeftWithLeftLeds() + prox.countsLeftWithRightLeds();
  front = prox.countsFrontWithLeftLeds() + prox.countsFrontWithRightLeds();
  right = prox.countsRightWithLeftLeds()  + prox.countsRightWithRightLeds();

  int ringStatus = checkRing();

  // LCD line 0: state, front, ring
  lcd.gotoXY(0, 0);
  lcd.print((int)currentState);
  lcd.print(" F");
  lcd.print(front);
  lcd.print(" R");
  lcd.print(ringStatus);

  // LCD line 1: raw sensor values
  lcd.gotoXY(0, 1);
  lcd.print(rawSensorValues[0]);
  lcd.print(" ");
  lcd.print(rawSensorValues[1]);
  lcd.print(" ");
  lcd.print(rawSensorValues[2]);

  // Ring escape overrides everything
  static uint8_t ringConfirm = 0;
  const uint8_t RING_LIMIT = 3;

  if (ringStatus != 99)
  {
    ringConfirm++;
    if (ringConfirm >= RING_LIMIT)
    {
      ringConfirm = 0;
      handleEvadeRing(); // run evade action immediately
    }
    }
  else
    {
      ringConfirm = 0;
    }

  switch (currentState) {
    case idle:
      motors.setSpeeds(0, 0);
      currentState = search;
      break;

    case search:
      if (ringStatus != 99) break;
      if (front >= SEARCH_THRESHOLD) {
      // short approach to ensure we actually move into contact before ramming
      motors.setSpeeds(160, 160);   // gentle forward approach
      delay(120);

      // refresh sensors after the approach
      prox.read();
      lineSensors.read(rawSensorValues);
      left  = prox.countsLeftWithLeftLeds() + prox.countsLeftWithRightLeds();
      front = prox.countsFrontWithLeftLeds() + prox.countsFrontWithRightLeds();
      right = prox.countsRightWithLeftLeds()  + prox.countsRightWithRightLeds();

      if (front >= COMMIT_THRESHOLD) {
        performRam(); // execute ram immediately
      } else {
        motors.setSpeeds(0, 0);
      }
      break;
      } 
      if (left >= SEARCH_THRESHOLD) {
      motors.setSpeeds(-120, 200);  // turn toward target
      delay(120);
      // then perform the same short forward approach and re-sample (use same snippet above)
      }
      if (right >= SEARCH_THRESHOLD) {
      motors.setSpeeds(200, -120);  // turn toward target
      delay(120);
      // then perform the same short forward approach and re-sample (use same snippet above)
      }
      // Scanning motion
      unsigned long now = millis();
      if (now - scanTimer > arcTime) {
        scanDirection *= -1;
        scanTimer = now;
      }
      if (now - pivotTimer > pivotInterval) {
        motors.setSpeeds(160 * scanDirection, -160 * scanDirection);
        if (now - pivotTimer > pivotInterval + pivotTime)
          pivotTimer = now;
        break;
      }
      if (scanDirection == 1)
        motors.setSpeeds(200, 160);
      else
        motors.setSpeeds(160, 200);
      break;

    case dodge:
      if (left > right)
        motors.setSpeeds(dodgeSpeed, -dodgeSpeed);
      else
        motors.setSpeeds(-dodgeSpeed, dodgeSpeed);
      delay(200);
      motors.setSpeeds(200, 200);
      delay(250);
      currentState = search;
      break;
    case ram: {
      performRam();
      break;
    }
    case evadeRing:
      int dir = checkRing();
      motors.setSpeeds(-200, -200);
      delay(150);
      if (dir == -1)
        motors.setSpeeds(200, -200);
      else if (dir == 1)
        motors.setSpeeds(-200, 200);
      else
        motors.setSpeeds(200, -200);
      delay(120);
      currentState = search;
      break;
  }
  delay(20);
}

// Immediate handler for ring-evade behavior. Call directly when ring is confirmed
void handleEvadeRing()
{
  int dir = checkRing();
  // small LCD indicator so we can see the handler running
  lcd.gotoXY(10, 0);
  lcd.print("EV");
  motors.setSpeeds(-200, -200);
  delay(150);
  if (dir == -1)
    motors.setSpeeds(200, -200);
  else if (dir == 1)
    motors.setSpeeds(-200, 200);
  else
    motors.setSpeeds(200, -200);
  delay(120);
  currentState = search;
}

