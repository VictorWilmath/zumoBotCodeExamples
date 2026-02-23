#include <Zumo32U4.h>
#include <Wire.h>
#include <LSM6.h>

LSM6 imu;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors prox;
Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4Encoders encoders;

const uint8_t SEARCH_THRESHOLD = 3;
const uint8_t COMMIT_THRESHOLD = 3;
const int ENCODER_STALL_THRESHOLD = 2;
uint8_t detectCount = 0;
const int pushSpeed = 300;
const int dodgeSpeed = 200;
int scanDirection = 1;
unsigned long scanTimer = 0;
unsigned long pivotTimer = 0;
const unsigned long arcTime = 900;
const unsigned long pivotTime = 220;
const unsigned long pivotInterval = 3000;
unsigned long lastScanSwitch = 0;
const unsigned long scanSwitchInterval = 1500;
const unsigned long stallTime = 200; // (ms)
const uint16_t ringThreshold = 600;
const float collisionThreshold = 20000;
unsigned long stallStart = 0;
unsigned long searchPhaseTimer = 0;
bool scanning = true;
uint8_t left = 0;
uint8_t front = 0;
uint8_t right = 0; 


uint16_t sensorValues[3];

void pushBehavior(uint8_t frontSignal){
  static long lastLeft = 0;
  static long lastRight = 0;
  motors.setSpeeds(pushSpeed, pushSpeed);
  long currentLeft = encoders.getCountsLeft();
  long currentRight = encoders.getCountsRight();
  long deltaLeft = abs(currentLeft - lastLeft);
  long deltaRight = abs(currentRight - lastRight);
  if(frontSignal > COMMIT_THRESHOLD){
    if(deltaLeft < ENCODER_STALL_THRESHOLD && deltaRight < ENCODER_STALL_THRESHOLD){
      if(stallStart == 0){
        stallStart = millis();
      }
      if(millis() - stallStart > stallTime){
        motors.setSpeeds(-200, -200);
        delay(150);
        motors.setSpeeds(pushSpeed, pushSpeed);
        stallStart = 0;
      }
    }
    else{
      stallStart = 0;
    }
  }
  lastLeft = currentLeft;
  lastRight = currentRight;
}

int checkRing() {
  lineSensors.readCalibrated(sensorValues);
  bool leftRing   = sensorValues[0] > ringThreshold;
  bool centerRing = sensorValues[1] > ringThreshold;
  bool rightRing  = sensorValues[2] > ringThreshold;
  if (leftRing)   return -1;   
  if (rightRing)  return 1;    
  if (centerRing) return 0;    
  return 99;                   
}

bool collisionDetected() {
  imu.read();
  int ax = imu.a.x;
  return abs(ax) > collisionThreshold;
}

enum DetectState{
  noDetect,
  frontDetect,
  sideDetect
};

enum State{
  idle,
  search, 
  dodge,
  attack,
  ram,
  evadeRing
};

State currentState = idle;

void setup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  lcd.init();
  lcd.clear();
  buttonA.waitForPress();
  lineSensors.initThreeSensors();
  for(uint8_t i = 0; i < 40; i++){
    lineSensors.calibrate();
    delay(20);
  }
  delay(300);
  prox.initThreeSensors();
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  delay(500);
  pivotTimer = millis();
  scanTimer = millis();
}

void loop() {
  prox.read();
  checkRing();
  lineSensors.readCalibrated(sensorValues);
  left  = prox.countsLeftWithLeftLeds() + prox.countsLeftWithRightLeds();
  front = prox.countsFrontWithLeftLeds() + prox.countsFrontWithRightLeds();
  right = prox.countsRightWithLeftLeds()  + prox.countsRightWithRightLeds();

  int ringStatus = checkRing();
  if(ringStatus != 99){
    currentState = evadeRing;
  }
  
  switch(currentState){
    case idle:
      motors.setSpeeds(0, 0);
      currentState = search;
      break;
    case search:
      if(ringStatus != 99){
        return;
      }
      if(front >= SEARCH_THRESHOLD){
        currentState = attack;
        break;
      }
      if(left >= SEARCH_THRESHOLD){
        motors.setSpeeds(-120, 200);
        currentState = attack;
        break;
      }
      if(right >= SEARCH_THRESHOLD){
        motors.setSpeeds(200, -120);
        currentState = attack;
        break;
      }
      unsigned long now = millis();
      if(now - scanTimer > arcTime){
        scanDirection *= -1;
        scanTimer = now;
      }
      if(now - pivotTimer > pivotInterval){
        motors.setSpeeds(160 * scanDirection, -160 * scanDirection);
        if(now - pivotTimer > pivotInterval + pivotTime){
          pivotTimer = now;
        }
        break;
      }
      int baseSpeed = 200;
      int curve = 40;
      if(scanDirection == 1){
        motors.setSpeeds(baseSpeed, baseSpeed - curve);
      }
      else{
        motors.setSpeeds(baseSpeed - curve, baseSpeed);
      }
      break;
    case dodge:
      if(left > right){
        motors.setSpeeds(dodgeSpeed, -dodgeSpeed);
      }
      else{
        motors.setSpeeds(-dodgeSpeed, dodgeSpeed);
      }
      delay(200);
      motors.setSpeeds(200, 200);
      delay(250);
      currentState = search;
      break;
    case attack:
      if(front > COMMIT_THRESHOLD){
        motors.setSpeeds(250, 250);
        if(collisionDetected()){
          currentState = ram;
        }
      }
      else if(left > COMMIT_THRESHOLD || right > COMMIT_THRESHOLD){
        currentState = dodge;
      }
      else{
        currentState = search;
      }
      break;
    case ram:
      motors.setSpeeds(-200, -200);
      delay(120);
      bool angleRight = random(0, 2) == 0;
      int baseLeft;
      int baseRight;
      if(angleRight){
        baseLeft = 255;
        baseRight = 200;
      }
      else{
        baseLeft = 200;
        baseRight = 255;
      }
      for(int speed = 120; speed <= 400; speed += 40){
        float scale = speed / 255.0;
        int leftSpeed = baseLeft * scale;
        int rightSpeed = baseRight * scale;
        motors.setSpeeds(leftSpeed, rightSpeed);
        delay(30);
      }
      currentState = attack;
      break;
    case evadeRing:
      int dir = checkRing();
      motors.setSpeeds(-200, -200);
      delay(150);
      if(dir == -1){
        motors.setSpeeds(-200, 200);
        delay(120);
      }
      else if(dir == 1){
        motors.setSpeeds(-200, 200);
        delay(120);
      }
      else{
        motors.setSpeeds(200, -200);
        delay(120);
      }
      currentState = search;
      break;
  }
  delay(20);
}
    