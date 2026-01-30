// https://github.com/Bays246, 2023

/*
 * LCD Display is 8x2.
 * Paused is not a state, it is in the loop function.
 * The yellow led is designated as the pause light, when it is on all code execution is paused.
 * Utilize delayTimeMillis for delays, otherwise button presses don't register well.
 * Zumo bot utilizes ATmega32U4 AVR microcontroller.
 * There is no way out of the main states besides resetting.
 * Yes I know that switch takes up a lot of memory, but I don't care and it didn't end up mattering.
 * 
 * Zumo 1 needs left motor flipped.
 * Zumo 2 needs both motors flipped.
 * 
 * Zumo 2 has the new motors.
 */

// ----------------------------------------------DEFINITIONS-----------------------------------------------------------------------

#include <Wire.h>
#include <Zumo32U4.h>

// Interface with the hardware attached to the Zumo bot.
Zumo32U4LCD display; // Interfaces with the display. Alternate class is Zumo32U4OLED.
Zumo32U4ButtonA buttonA; // Interfaces with button A on the bot.
Zumo32U4ButtonB buttonB; // Interfaces with button B on the bot.
Zumo32U4ButtonC buttonC; // Interfaces with button C on the bot.
Zumo32U4Buzzer buzzer; // Interfaces with buzzer located behind the screen.
Zumo32U4Motors motors; // Interfaces with both motors on the bot.
Zumo32U4LineSensors lineSensors; // Interfaces with available line sensors.
Zumo32U4ProximitySensors proxSensors; // Interfaces available proximity sensors.
Zumo32U4IMU imu; // Interfaces with the gyroscope/magnetometer/accelerometer unit.

// Sensor Sensitivity.
#define LINE_THRESHOLD 1000 // Reference number for what is considered a white line.
#define PROX_THRESHOLD 4 // Out of 6 brightness levels, how many needs to trigger the sensor.
#define ACCEL_THRESHOLD 900 // Reference for movement.
#define IMU_SAMPLES 5 // Number of samples taken before averaging.

// Motor Speeds. Max speed is 400 due to Zumo bot itself (The class "Zumo32U4Motors" only allows up to 400).
#define SPEED_MAX 400 // FULL POWER.
#define SPEED_HIGH 300 // High speed, woah.
#define SPEED_MED 200 // Used for wide turns.
#define SPEED_LOW 100 // Low speed, wooooaaaahhhh.
#define SPEED_STOP 0 // Please do not change this from zero.

// Misc
#define COUNTDOWN_SECONDS 5 // Number of seconds you have to set the robot down in sumo mode.
#define LINE_EVADE_TURN_TIME 250 // Number in milliseconds to turn when evading the line on the arena edge.
#define LINE_EVADE_REVERSE_TIME 200 // Number in milliseconds to back up when evading the line on the arena edge.
#define MOVEMENT_CHANGE_TIME 100 // Number of milliseconds to wait before recalibrating the Accel unit after changing motor speeds.

// Robot States.
// The order here determins the order of the menu (Keep stateMainMenu as the first one).
enum mainStates {
  stateMainMenu,
  stateSumo,
  stateMotorTest,
  stateSensorTest,
  stateBatteryTest,
  stateMusicTest,
};

// Universal Variables.
mainStates mainState = stateMainMenu; // What is the Zumo bot supposed to be doing?
String topScreenBuffer; // Used when building strings for the display.
String bottomScreenBuffer; // Same as topScreenBuffer.
bool generalFlag = false; // General purpose bool for any reason.
bool displayFlag = true; // When true, the display needs to be updated.
bool motorFlag = false; // When true, the motor speed needs updated.
bool movingFlag = false; // When true, at least one motor is active.
bool aButtonPressed; // The a button is the leftmost on the Zumo bot.
bool bButtonPressed; // The b button is the center button on the Zumo bot.
bool cButtonPressed; // The c button is the rightmost on the Zumo bot.
int leftMotorBuffer; // Used when using logic to determine motor speed.
int rightMotorBuffer; // Same as leftMotorBuffer.
int imuOriginal[3]; // Used to compare new imu values to previous ones.
int imuValues[3][IMU_SAMPLES]; // Used to smooth out spikes in imu sensors.
int imuCompare[3]; // Used to compare new imu values to previous ones.
byte imuIndex; // Used to overwrite specific values in imuReadings.
unsigned int lineSensorValues[3]; // Used to store result from "lineSensors.read(lineSensorValues);"
unsigned long delayTimeMillis; // Used for delays inside states.

// Seperate file for each main state.
#include "SharedFunctions.h"
#include "MainMenu.h"
#include "MotorTest.h"
#include "BatteryTest.h"
#include "SensorTest.h"
#include "MusicTest.h"
#include "Sumo.h"

// ----------------------------------------------PRECONFIG-------------------------------------------------------------------------

void setup() {
  // Initilize motors.
  motors.flipLeftMotor(true); // I needed to flip the direction of motors.
  motors.flipRightMotor(false); // True Zumo 2, false Zumo 1.
  
  // Initilize basic sensors.
  lineSensors.initThreeSensors(); // Using 3 line sensors.
  proxSensors.initThreeSensors(); // Using 3 prox sensors.

  // Initilize IMU.
  Wire.begin();
  imu.init();
  imu.enableDefault();
}

// ----------------------------------------------RUNNING LOOP----------------------------------------------------------------------

void loop() {
  // Button Capture.
  aButtonPressed = buttonA.getSingleDebouncedPress();
  bButtonPressed = buttonB.getSingleDebouncedPress();
  cButtonPressed = buttonC.getSingleDebouncedPress();

  // Actions depend on current state.
  switch(mainState){
    case stateMainMenu:{
      mainMenu();
      break;
    }
    case stateSumo:{
      sumo();
      break;
    }
    case stateMusicTest:{
      musicTest();
      break;
    }
    case stateBatteryTest:{
      batteryTest();
      break;
    }
    case stateMotorTest:{
      motorTest();
      break;
    }
    case stateSensorTest:{
      sensorTest();
      break;
    }
  }
  
  // Pausing
  if(cButtonPressed){
    // Save current time.
    unsigned long pauseTimeMillis = millis();

    // Save current movement. (Zumo's setLeftSpeed uses OCR1B and setRightSpeed uses OCR1A, DIR_L 16, DIR_R 15)
    // Why I know where to go for motor speed: https://pololu.github.io/zumo-32u4-arduino-library/_zumo32_u4_motors_8cpp_source.html
    // Read on FastGPIO here: https://github.com/pololu/fastgpio-arduino
    int leftMotorSpeed = OCR1B;
    int rightMotorSpeed = OCR1A;
    bool leftMotorDir = FastGPIO::Pin<16>::isInputHigh();
    bool rightMotorDir = FastGPIO::Pin<15>::isInputHigh();
    
    // Make sure button is not still pressed (exits prematurely if this is not here).
    buttonC.waitForRelease();

    // Stop moving.
    motors.setSpeeds(SPEED_STOP, SPEED_STOP);

    // Pause light.
    ledYellow(true);

    // You must press the button to continue...
    buttonC.waitForButton();

    // Pause light.
    ledYellow(false);

    // Resume moving.
    motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
    FastGPIO::Pin<16>::setOutput(leftMotorDir);
    FastGPIO::Pin<15>::setOutput(rightMotorDir);

    // Restore time (by adding time spent here to the original time).
    // This is done so that delays in other parts of the code don't lose their place.
    delayTimeMillis += millis() - pauseTimeMillis;
  }
}
