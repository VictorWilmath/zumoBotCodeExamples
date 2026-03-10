

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;

void setup() {
  proxSensors.initThreeSensors();
  buttonA.waitForButton();
  delay(1000);
}

void loop() {
  proxSensors.read();

  uint8_t frontLeft = proxSensors.countsFrontWithLeftLeds();
  uint8_t frontRight = proxSensors.countsFrontWithRightLeds();
  uint8_t sideLeft = proxSensors.countsLeftWithLeftLeds();
  uint8_t sideRight = proxSensors.countsRightWithRightLeds();

  if (frontLeft > 0 || frontRight > 0) {
    uint8_t maxReading = max(frontLeft, frontRight);
    int variableBaseSpeed = map(maxReading, 1, 6, 400, 100);

    int rawDiff = (int)frontLeft - (int)frontRight;
    int error = rawDiff * 5;
    int speedDifference = error * 50;

    int leftSpeed = variableBaseSpeed - speedDifference;
    int rightSpeed = variableBaseSpeed + speedDifference;

    leftSpeed = constrain(leftSpeed, -400, 400);
    rightSpeed = constrain(rightSpeed, -400, 400);

    motors.setSpeeds(leftSpeed, rightSpeed);
  } 
  else if (sideLeft > 0) {
    motors.setSpeeds(-200, 200); 
  }
  else if (sideRight > 0) {
    motors.setSpeeds(200, -200);
  }
  else {
    motors.setSpeeds(0, 0);
  }
}