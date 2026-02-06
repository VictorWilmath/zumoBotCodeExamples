#include <Wire.h>
#include <Zumo32U4.h>


// Instantiate the objects for the hardware we will use in the FSM
Zumo32U4LCD display;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4IMU imu;

// 1. Define the States
enum ZumoState {
  CALIBRATION,
  WAIT_FOR_START,
  SEARCH,     // HUNT
  ATTACK,     // TRACK
  CONFLICT,   // PUSH
  SURVIVAL    // BORDER
};
// Global Variables
float lastError = 0;
unsigned long pushStartTime = 0;

// Initialize State
ZumoState currentState = CALIBRATION;

void setup() {
  // Hardware initialization (sensors, motors, button)
  // Serial.begin(9600);
Wire.begin(); // Required for the IMU to talk to the Arduino
  
  // Initialize the IMU
  if (!imu.init()) {
    // If IMU fails, halt everything and flash the LED to warn the user
    while(1) {
      ledRed(1);
      delay(100);
      ledRed(0);
      delay(100);
    }
  }
  
  imu.enableDefault(); // Turn on accel and gyro
  
}

void loop() {
    int leftValue = 0; 
    int rightValue = 0;
  // --- HIGH PRIORITY INTERRUPT: SURVIVAL ---
  // If we are active (not calibrating or waiting) and sensors see the line:
  // Immediately override current state to SURVIVAL.
  if (currentState != CALIBRATION && currentState != WAIT_FOR_START) {
    if (isBorderDetected()) {
      currentState = SURVIVAL;
    }
  }

  // --- FSM SWITCH LOGIC ---
  switch (currentState) {
    
    // STATE 1: CALIBRATION
    // Executed once on boot.
    case CALIBRATION:
      // --- 1. Initialize Sensor Objects ---
      // We use all 5 line sensors for maximum safety
      // lineSensors.initFiveSensors(); 
      // We use the front and side proximity sensors for hunting

      proxSensors.initThreeSensors(); 

      break;

      // --- 2. User Feedback ---
      display.clear();
      display.print("Calib..Press A");

      buttonA.waitForButton();
      // Delay to give you time to move your hand away before it starts spinning
      delay(1000); 

      // --- 3. The Calibration Loop ---
      // We rotate left and right so sensors "see" the floor and the white line.
      // 80 iterations * 20ms = ~1.6 seconds of calibration
      for(int i = 0; i < 80; i++) 
      {
        // Spin Left for first phase, then Right
        if (i < 20 || i >= 60) {
          motors.setSpeeds(200, -200); 
        } else {
          motors.setSpeeds(-200, 200); 
        }

        // IMPORTANT: This function records the contrast values
       lineSensors.calibrate(); // Calibrate breaking switch?
        
        // Keep the loop timing consistent
        delay(20);
      }

      // --- 4. Cleanup & Transition ---
      motors.setSpeeds(0, 0); // Stop moving
      
      display.clear();
      display.print("Ready!");
      
      // Calibration complete, move to the Sumo Start Delay
      currentState = WAIT_FOR_START;
      break;
    // STATE 2: WAIT_FOR_START
    // Handling the 5-second delay rule (Sumo regulation).
    case WAIT_FOR_START:
      // --- 1. Waiting for Input ---
      display.clear();
      display.print("Press A");

      // This function pauses everything until Button A is pressed and released.
      buttonA.waitForButton();

      // --- 2. The 5-Second Countdown ---
      display.clear();
      
      // Loop 5 times (for 5 seconds)
      for (int i = 0; i < 5; i++) {
        display.gotoXY(0, 0);
        display.print("Start in");
        display.gotoXY(0, 1);
        display.print(5 - i); // Print 5, 4, 3, 2, 1
        
        // Beep to indicate a second passed
        buzzer.playNote(NOTE_G(4), 200, 15); 
        
        delay(1000); // Wait 1 full second
      }

      // --- 3. Go! ---
      buzzer.playNote(NOTE_G(5), 500, 15); // Higher pitch "Go" beep
      display.clear();
      display.print("HUNTING");
      
      // Transition: Time is up, enter the arena!
      currentState = SEARCH;
      break;

    // STATE 3: SEARCH (HUNT)
    // Active pattern to locate the opponent (Spin, Scan, or Arc).
    case SEARCH:
      // --- 1. Read Proximity Sensors ---
      proxSensors.read();
      
      // We look at the front-left and front-right sensors
      leftValue = proxSensors.countsFrontWithLeftLeds();
      rightValue = proxSensors.countsFrontWithRightLeds();

      // --- 2. Check for Enemy ---
      // If either sensor sees something (value > 0), we found them!
      if (leftValue > 0 || rightValue > 0) {
        // Stop spinning immediately
        motors.setSpeeds(0, 0);
        display.print("In here");
        // Transition: Engage the target
        currentState = ATTACK;
        break; 
      }

      // --- 3. Execute Search Pattern ---
      // If no enemy is seen, we spin to scan the area.
      // Speed 200 is fast enough to scan, slow enough to not miss detection.
      motors.setSpeeds(200, -200); 
      
      // Note: We do NOT break or change state here; we stay in SEARCH
      // until the sensors trigger the 'if' block above.
      break;

    // STATE 4: ATTACK (TRACK)
    // PID-controlled approach to the target.
    case ATTACK:
      // --- 1. Read Proximity Sensors ---
      proxSensors.read();
      leftValue = proxSensors.countsFrontWithLeftLeds();
      rightValue = proxSensors.countsFrontWithRightLeds();

      // --- 2. Check Transitions ---
      
      // LOST: If both are zero, we lost them. Go back to hunting.
      if (leftValue == 0 && rightValue == 0) {

        currentState = SEARCH;
        break;
      }
      
      // CONTACT: If readings are very high (close), we have made contact.
      // (Max reading is usually 6). Switch to full power pushing.
      if (leftValue >= 6 || rightValue >= 6) {
        currentState = CONFLICT;
        break;
      }
      // Inside ATTACK case:
      if (leftValue >= 6 || rightValue >= 6) {
        pushStartTime = millis(); // <--- ADD THIS LINE
        currentState = CONFLICT;
        break;
      }
      // --- 3. PID Calculations ---
      
      // The "Error" is how far off-center the opponent is.
      // Positive error = opponent is to the left.
      // Negative error = opponent is to the right.
      int error = leftValue - rightValue;

      // "Derivative" is how fast the error is changing (dampens oscillation)
      int derivative = error - lastError;
      lastError = error;

      // Kp (Proportional Gain): How hard to turn based on current error
      // Kd (Derivative Gain): How much to resist changing direction too fast
      // Tweak these numbers! (Kp=4, Kd=1 are good starting points for Zumo)
      int turnSpeed = (error * 4) + (derivative * 1);

      // --- 4. Motor Output ---
      // Base speed is 200 (out of 400). 
      // We add/subtract turnSpeed to steer towards the target.
      int leftMotorSpeed = 200 + turnSpeed;
      int rightMotorSpeed = 200 - turnSpeed;

      motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
      break;

    // STATE 5: CONFLICT (PUSH)
    // Full-power engagement upon contact.
    case CONFLICT:
      // --- 1. Full Power Engagement ---
      // Zumo motors max out at 400. We go all in.
      motors.setSpeeds(400, 400);

      // --- 2. Check for "Stall" / Stalemate ---
      // If we have been pushing for > 3 seconds, we are likely stuck.
      // We need to break the lock to save motors and try a new angle.
      if (millis() - pushStartTime > 3000) {
        // A. Stop momentarily
        motors.setSpeeds(0, 0);
        delay(100);

        // B. The "Judo" Move: Back up quickly
        motors.setSpeeds(-400, -400);
        delay(200); // Short burst backwards

        // C. Turn sharply to change angle
        motors.setSpeeds(400, -400);
        delay(250); 

        // D. Reset to Hunting
        currentState = SEARCH;
        break;
      }

      // --- 3. Check for Enemy Loss ---
      // If the opponent slips off our front sensors, we stop pushing blindly.
      proxSensors.read();
      if (proxSensors.countsFrontWithLeftLeds() < 2 && 
          proxSensors.countsFrontWithRightLeds() < 2) {
        currentState = SEARCH;
      }
      break;

    // STATE 6: SURVIVAL (BORDER)
    // Highest priority interrupt to avoid ring-out.
    case SURVIVAL:
      // --- 1. Visual Feedback ---
      display.clear();
      display.print("BORDER!");
      // Panic beep
      buzzer.playNote(NOTE_C(6), 100, 15);

      // --- 2. The Escape Maneuver ---
      
      // Step A: Full Reverse to stop momentum
      // We back up for 300ms. This brings the sensors back over black.
      motors.setSpeeds(-400, -400);
      delay(300);

      // Step B: Spin Turn
      // Ideally, we want to turn roughly 130-180 degrees.
      // Speed 400 for 400ms usually achieves a sharp turn.
      motors.setSpeeds(400, -400);
      delay(400);

      // --- 3. Resume Hunting ---
      // We assume we are safe now.
      // We do NOT check sensors here because we just blindly backed up.
      // Checking immediately might trigger a false positive if we are still near the edge.
      currentState = SEARCH;
      break;
  }
}

// --- PLACEHOLDER HELPER FUNCTIONS ---
// (You would fill these with your specific sensor reading/motor logic)

bool isBorderDetected() {
  // Array to hold sensor values (0 to 1000)
  unsigned int sensors[5];
  lineSensors.read(sensors);
  
  // "White" usually reads as a low value (e.g., < 200) on Zumo 32U4.
  // "Black" usually reads high (e.g., > 600).
  // We check the outer sensors (0 and 4) first as they hit the line first.
  if (sensors[0] < 200 || sensors[4] < 200) {
    return true;
  }
  return false;
}

bool isSafeFromBorder() {
  // Return true if line sensors read black surface
  return true;
}

bool enemySeen() {
  // Return true if proximity sensors detect object
  return false; 
}

bool contactMade() {
  // Return true if accelerometer/current-spike indicates impact
  return false; 
}

void runCalibration() { /* ... */ }
void waitForButtonAndDelay() { /* ... */ }
void executeSearchPattern() { /* ... */ }
void executePIDApproach() { /* ... */ }
void fullPowerPush() { /* ... */ }
void executeEscapeManeuver() { /* ... */ }