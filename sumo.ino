//https://gist.github.com/garrows/19b713809295e277ba8a
//2014

#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

#define LED 13
#define DISTANCE_SENSOR A1

// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  500 // microseconds

// these might need to be tuned for different motor types
#define REVERSE_SPEED     400 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define ATTACK_SPEED      400
#define SEARCH_SPEED      170

#define REVERSE_DURATION  200 // loops
#define TURN_DURATION     300 // loops


ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];

ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

void waitForButtonAndCountDown()
{
    digitalWrite(LED, HIGH);
    button.waitForButton();
    digitalWrite(LED, LOW);

    // play audible countdown
    for (int i = 0; i < 3; i++)
    {
        delay(1000);
        buzzer.playNote(NOTE_G(3), 200, 15);
    }
    delay(1000);
    buzzer.playNote(NOTE_G(4), 500, 15);
    delay(1000);
}

enum State { TurnLeft, TurnRight, Reverse, Attack, Search };
State state = Search;
State nextState = Search;
int stateCount = 0;
int stateCountLimit = 0;


void setup()
{
    // uncomment if necessary to correct motor directions
    //motors.flipLeftMotor(true);
    //motors.flipRightMotor(true);

    pinMode(LED, HIGH);

    waitForButtonAndCountDown();
}

void loop()
{
    int distance = analogRead(DISTANCE_SENSOR);
    sensors.read(sensor_values);


    switch(state) {
        case Search:
            motors.setSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
        break;
        case TurnLeft:
            if (++stateCount < stateCountLimit) {
                motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
            } else {
                /*state = Search;*/
            }
        break;
        case TurnRight:
            if (++stateCount < stateCountLimit) {
                motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
            } else {
                /*state = Search;*/
            }
        break;
        case Reverse:
            if (++stateCount < stateCountLimit) {
                motors.setSpeeds(-TURN_SPEED, -TURN_SPEED);
                buzzer.playNote(NOTE_C(3), 200, 15);
            } else {
                state = nextState;
                nextState = Search;
                stateCount = 0;
                stateCountLimit = TURN_DURATION;
            }
        break;
        case Attack:
            if (++stateCount < stateCountLimit) {
                buzzer.playNote(NOTE_A(3), 200, 15);
                motors.setSpeeds(ATTACK_SPEED, ATTACK_SPEED);
            } else {
                state = Search;
                nextState = Search;
            }

        break;
    }



    if (button.isPressed())
    {
        // if button is pressed, stop and wait for another press to go again
        motors.setSpeeds(0, 0);
        button.waitForRelease();
        waitForButtonAndCountDown();
    }

    if (sensor_values[0] < QTR_THRESHOLD)
    {
        stateCount = 0;

        nextState = TurnLeft;
        state = Reverse;
        stateCount = 0;
        stateCountLimit = REVERSE_DURATION;
    }
    else if (sensor_values[5] < QTR_THRESHOLD)
    {
        stateCount = 0;

        nextState = TurnRight;
        state = Reverse;
        stateCount = 0;
        stateCountLimit = REVERSE_DURATION;
    }
    else if(distance > 230)
    {
        state = Attack;
        stateCount = 0;
        stateCountLimit = 500;
    }
}
