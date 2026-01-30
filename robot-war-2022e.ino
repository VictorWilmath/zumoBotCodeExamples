//  Mikael Forsberg, 2022, https://github.com/lizell/sumo

/** This example uses the Zumo's line sensors to detect the black
border around a sumo ring.  When the border is detected, it
backs up and turns. */

#include <Wire.h>
#include <Zumo32U4.h>

// This might need to be tuned for different lighting conditions,
// surfaces, etc.
#define QTR_THRESHOLD     250  // microseconds, default 400

// These might need to be tuned for different motor types.
#define REVERSE_SPEED     400  // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     400
#define FULL_SPEED        400
#define REVERSE_DURATION  500  // ms
#define TURN_DURATION     400  // ms

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

// A sensors reading must be greater than or equal to this
// threshold in order for the program to consider that sensor as
// seeing an object.
const uint8_t sensorThreshold = 2;
const uint8_t chargeSensorThreshold = 5;

// The maximum speed to drive the motors while turning.  400 is
// full speed.
const uint16_t turnSpeedMax = 400;

// The minimum speed to drive the motors while turning.  400 is
// full speed.
const uint16_t turnSpeedMin = 100;

// The amount to decrease the motor speed by during each cycle
// when an object is seen.
const uint16_t deceleration = 10;

// The amount to increase the speed by during each cycle when an
// object is not seen.
const uint16_t acceleration = 10;


#define LEFT 0
#define RIGHT 1

// Stores the last indication from the sensors about what
// direction to turn to face the object.  When no object is seen,
// this variable helps us make a good guess about which direction
// to turn.
bool senseDir = RIGHT;

// True if the robot is turning left (counter-clockwise).
bool turningLeft = false;

// True if the robot is turning right (clockwise).
bool turningRight = false;

// If the robot is turning, this is the speed it will use.
uint16_t turnSpeed = turnSpeedMax;

// The time, in milliseconds, when an object was last seen.
uint16_t lastTimeObjectSeen = 0;

// The time, in milliseconds, we have been searching for an object
uint16_t searchTime = 0;

// milliseconds before giving up and going for a new location
uint16_t searchTimeThreshold = 1000;

// Are we charging?
bool isCharging = false;

// Never stop
bool isNeverStop = false;

String line1 = "";
String line2 = "";
bool buttonPressed = false;

void waitForButtonAndCountDown()
{
  ledYellow(1);
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("Program");
  lcd.gotoXY(0, 1);
  lcd.print("A, B, C");

  do {
    if (buttonA.isPressed()) {
      buttonPressed = true;
    } else if (buttonB.isPressed()) {
      buttonPressed = true;
      isNeverStop = true;
      searchTimeThreshold = 250;
    } else if (buttonC.isPressed()) {
      buttonPressed = true;
      isNeverStop = true;
      searchTimeThreshold = 250;

      // Spin around 180° before start
      lcd.clear();
      lcd.gotoXY(0, 0);
      lcd.print("Reverse!");
      motors.setSpeeds(FULL_SPEED, -FULL_SPEED);
      delay(TURN_DURATION);
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    }
  } while (!buttonPressed);

  ledYellow(0);
  lcd.clear();

  // Play audible countdown.
  for (int i = 0; i < 3; i++)
  {
    delay(100);
    buzzer.playNote(NOTE_G(2+i), 50, 15);
  }
  delay(100);
  buzzer.playNote(NOTE_G(5), 100, 15);
}

void turnRight()
{
  motors.setSpeeds(turnSpeed, -turnSpeed);
  turningLeft = false;
  turningRight = true;
  line1 = "Right";
  line2 = "";
}

void turnLeft()
{
  motors.setSpeeds(-turnSpeed, turnSpeed);
  turningLeft = true;
  turningRight = false;
  line1 = "Left";
  line2 = "";
}

void stop()
{
  motors.setSpeeds(0, 0);
  turningLeft = false;
  turningRight = false;
  line1 = "Stop";
  line2 = "";
}

void forward()
{
  motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  turningLeft = false;
  turningRight = false;
  line1 = "Forward";
  line2 = "";
}

void charge() {
  charge(false);
}

void charge(bool blindly)
{
  motors.setSpeeds(FULL_SPEED, FULL_SPEED);
  turningLeft = false;
  turningRight = false;
  isCharging = true;
  line1 = "Charge";
  if (blindly) {
    buzzer.playNote(NOTE_C(6), 500, 15);
    line2 = "Blindly";
  } else {
    buzzer.playNote(NOTE_G(4), 500, 15);
  }
}

void setup()
{
  proxSensors.initFrontSensor();
  lineSensors.initThreeSensors();

  waitForButtonAndCountDown();
}

void loop()
{
  // Read the front proximity sensor and gets its left value (the
  // amount of reflectance detected while using the left LEDs)
  // and right value.
  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

  // Determine if an object is visible or not.
  bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;

  lineSensors.read(lineSensorValues);

  if (lineSensorValues[0] > QTR_THRESHOLD) //flip aligator sign if you want to detect white edges on a black board instead of black border on a white board.
  {
    // If leftmost sensor detects line, reverse and turn to the
    // right.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    isCharging = false;
    searchTime = 0;
    senseDir = random(2) == 1 ? RIGHT : LEFT;
    line1 = "Reverse";
    line2 = "";
  }
  else if (lineSensorValues[NUM_SENSORS - 1] > QTR_THRESHOLD) //flip alligator sign if you want to detect white edges on a black board instead of black border on a white board.
  {
    // If rightmost sensor detects line, reverse and turn to the
    // left.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    isCharging = false;
    searchTime = 0;
    senseDir = random(2) == 1 ? RIGHT : LEFT;
    line1 = "Reverse";
    line2 = "";
  }
  else
  {
    if (!isCharging) {
      if (objectSeen)
      {
        // An object is visible, so we will start decelerating in
        // order to help the robot find the object without
        // overshooting or oscillating.
        turnSpeed -= deceleration;
      }
      else
      {
        // An object is not visible, so we will accelerate in order
        // to help find the object sooner.
        turnSpeed += acceleration;
      }

      // Constrain the turn speed so it is between turnSpeedMin and
      // turnSpeedMax.
      turnSpeed = constrain(turnSpeed, turnSpeedMin, turnSpeedMax);

      if (objectSeen)
      {
        // An object seen.
        ledYellow(1);

        lastTimeObjectSeen = millis();

        bool lastTurnRight = turnRight;

        if (leftValue < rightValue)
        {
          // The right value is greater, so the object is probably
          // closer to the robot's right LEDs, which means the robot
          // is not facing it directly.  Turn to the right to try to
          // make it more even.
          turnRight();
          senseDir = RIGHT;
        }
        else if (leftValue > rightValue)
        {
          // The left value is greater, so turn to the left.
          turnLeft();
          senseDir = LEFT;
        }
        else
        {
          // The values are equal
          if (leftValue >= chargeSensorThreshold) {
            charge();
          } else {
            forward();
          }
        }
      }
      else if (searchTime != 0 && (millis() - searchTime) > (searchTimeThreshold + random(-250, 250))) {
        charge(true);
        searchTime = 0;
      }
      else
      {
        // No object is seen, so just keep turning in a circle
        if (searchTime == 0) {
          searchTime = millis();
        }
        ledYellow(0);
        buzzer.playNote(NOTE_A(5), 100, 15);
        if (isNeverStop) {
          if (random(2) == 1) {
            random(2) == 1 ? turnRight() : turnLeft();
          } else {
            forward();
          }
        } else {
          turnRight();
        }
        line1 = "Search";
        line2 = "";
      }
    }
  }

  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print(line1);
  lcd.gotoXY(0, 1);
  lcd.print(line2);
}
