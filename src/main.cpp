#include <Arduino.h>
#include "gripper.h"

#define motor_LB 6 // left back
#define motor_LF 11 // left forward
#define motor_RB 5 // right back
#define motor_RF 9 // right forward

#define RPM_1 3
#define RPM_2 2

const int READ_RPM_INTERVAL = 1000;
const unsigned long WHEEL_A_INTERVAL = 8000;
const unsigned long WHEEL_B_INTERVAL = 8000;

unsigned long startTime = 0;
unsigned long startTimeA = 0;
unsigned long startTimeB = 0;
unsigned long startTurningTime = 0;

float RPM_A = 0;
float RPM_B = 0;
volatile unsigned long RPM_counterA = 0;
volatile unsigned long RPM_counterB = 0;
volatile unsigned long pulse_counterA = 0;
volatile unsigned long pulse_counterB = 0;

const float WHEEL_DIAMETER = 6; // in cm 7
const float WHEEL_WIDTH = 9; // 9 13
const float ROTATE_FACTOR_RIGHT = 3.2;      // tune: bigger = turns more, smaller = turns less
const float ROTATE_FACTOR_LEFT  = 4.625;
const int PULSES_PER_REVOLUTION = 20; // number of pulses per revolution of the wheel
const int MIN_SPEED_FORWARD = 160; // minimum speed to prevent stalling
const int START_SPEED_FORWARD = 190;
const int MIN_SPEED_BACKWARD = 180; // minimum speed to prevent stalling
const int START_SPEED_BACKWARD = 210; // speed to start the wheels at
const float TRIM_FORWARD = 0.87; // .86, increasing will make it go right
const float TRIM_BACKWARD = 0.90; // increasing will make it go right
const float CORRECTION_GAIN = 0;
const float MAX_CORRECTION = 15.0; // maximum correction to apply to the speed of the faster wheel
const int CONTROL_INTERVAL = 20; // interval in milliseconds to check and adjust the speed of the wheels
const int RAMP_PULSES = 15; // number of pulses to ramp up the speed of the wheels
const int DRIVE_SPEED_FORWARD = 190; // speed to drive the wheels at
const int DRIVE_SPEED_BACKWARD = 210;
const int ROTATE_SPEED = 220; // speed to turn the wheels at
const int TURN_INNER_SPEED = 160;
const int TURN_OUTER_SPEED = 220;
const float TURN_CORRECTION = 3.0;
const float MAXIMUM_CORRECTION = 40.0;
const float SLOW_DOWN = 6.0;
const float TURN_FACTOR_LEFT = 1.0;
const float TURN_FACTOR_RIGHT = 1.0;
const float TURN_RIGHT = 33.7;
const float TURN_LEFT = 33.5;


const unsigned long TURN_TIMEOUT = 8000;
// arcTurn tuning
const int TURN_PWM_STALL = 130;          // PWM where the wheel roughly stops (speed ~ PWM - this). tune
const int ARC_CATCHUP_PWM = 180;         // one wheel finishing alone after the other reached its target
const unsigned long ARC_MS_PER_PULSE = 60; // timeout budget per outer pulse (60 ms = ~16 pulses/s minimum)
const long ARC_MAX_CATCHUP = 12;         // max pulses one wheel may finish alone; more = something is wrong
// which encoder sits on the left wheel. arcTurn turning the WRONG way = flip this
const bool ENCODER_A_IS_LEFT = true;

bool done = false;

void RPM_Calc();
void wheelCounterA();
void wheelCounterB();
void resetCounters();
void stopBothMotors();
void setMotors(int leftSpeed, int rightSpeed, bool forward);
void driveStraight(bool forward, float distance);
void rotate(bool right, float angle);
void arcTurn(bool right, float angle, float radius);
void wheelCheck();
void waitWithGripper(unsigned long ms);

// const int MAX_SPEED = 0;
// read the wheels rpm, and correct the faster one to the slower one
void setup() {
  pinMode(GRIPPER_PIN, OUTPUT);
  pinMode(motor_LB, OUTPUT);
  pinMode(motor_LF, OUTPUT);
  pinMode(motor_RB, OUTPUT);
  pinMode(motor_RF, OUTPUT);
  pinMode(RPM_1, INPUT_PULLUP);
  pinMode(RPM_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RPM_1), wheelCounterA, RISING);
  attachInterrupt(digitalPinToInterrupt(RPM_2), wheelCounterB, RISING);

  servoLet();
  Serial.begin(9600);
  Serial.println("Bluetooth connected!");
  startTime = millis();
  startTurningTime = millis();
  //delay(2000);
}

const float SQUARE_SIDE = 150.0;
const unsigned long STEP_PAUSE = 1500;
void loop() {
  if (!done) {
    servoGrab();
    waitWithGripper(5000);
    servoLet();
    waitWithGripper(5000);

     // hold the robot in the air for this, remove once mapping is confirmed
    delay(2000);  // time to put the robot down
    /*
    // 1. forward, then back the same distance
    driveStraight(true, SQUARE_SIDE);
    delay(STEP_PAUSE);
    driveStraight(false, SQUARE_SIDE);
    delay(STEP_PAUSE);
    
    // 2. rotate left 90
    rotate(false, 90);
    delay(STEP_PAUSE);

    
    // 3. forward to the bottom-left corner
    driveStraight(true, SQUARE_SIDE);
    delay(STEP_PAUSE);
    
    // 4. rotate right 90 -> facing up
    rotate(true, 90);
    delay(STEP_PAUSE);
    */
    
    // 5. right arc to the top-right corner (radius = side of the square)
    arcTurn(true, 90, SQUARE_SIDE);
    delay(STEP_PAUSE);
    /*
    // 6. rotate right 90 -> facing down
    rotate(true, 90);
    delay(STEP_PAUSE);

    // 7. straight back to the start
    driveStraight(true, SQUARE_SIDE);
    */
    done = true;
  } else {
    stopBothMotors();
  }
}

void waitWithGripper(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    gripperUpdate();
  }
}

void RPM_Calc() {
  if (millis() - startTime > READ_RPM_INTERVAL) {
    noInterrupts();

    unsigned long temp1 = RPM_counterA;
    unsigned long temp2 = RPM_counterB;

    RPM_counterA = 0;
    RPM_counterB = 0;

    interrupts();

    RPM_A = (temp1 / 20.0) * (60000.0 / READ_RPM_INTERVAL);
    RPM_B = (temp2 / 20.0) * (60000.0 / READ_RPM_INTERVAL);
    
    /*
    Serial.print("RPM 1 ");
    Serial.print(RPM_A);
    Serial.print(" RPM 2 ");
    Serial.println(RPM_B);
    */
    startTime = millis();
  }
}

void resetCounters() {
  noInterrupts();
  pulse_counterA = 0;
  pulse_counterB = 0;
  interrupts();
}

void stopBothMotors() {
  analogWrite(motor_LF, 0);
  analogWrite(motor_RF, 0);
  analogWrite(motor_LB, 0);
  analogWrite(motor_RB, 0);
}

void setMotors(int leftSpeed, int rightSpeed, bool forward) {
  if(forward) {
    analogWrite(motor_LB, 0);
    analogWrite(motor_RB, 0);
    analogWrite(motor_LF, leftSpeed);
    analogWrite(motor_RF, rightSpeed);
  } else {
    analogWrite(motor_LF, 0);
    analogWrite(motor_RF, 0);
    analogWrite(motor_LB, leftSpeed);
    analogWrite(motor_RB, rightSpeed);
  }
}

void driveStraight(bool forward, float distance){
  resetCounters();
  float circumference = PI * WHEEL_DIAMETER;
  long targetPulses = (distance / circumference) * PULSES_PER_REVOLUTION;
  float trim = forward ? TRIM_FORWARD : TRIM_BACKWARD;
  unsigned long lastControlTime = millis();

  while(true){
    if(millis() - lastControlTime >= CONTROL_INTERVAL){
      lastControlTime = millis();
      noInterrupts();

      long pulsesA = pulse_counterA;
      long pulsesB = pulse_counterB;

      interrupts();

      long average = (pulsesA + pulsesB) / 2;
      if(average >= targetPulses){
        stopBothMotors();
        return;
      }
      float base_forward = DRIVE_SPEED_FORWARD;
      float base_backward = DRIVE_SPEED_BACKWARD;
      if(forward){
        if(average < RAMP_PULSES){
          base_forward = START_SPEED_FORWARD + (DRIVE_SPEED_FORWARD - START_SPEED_FORWARD) * (average / (float)RAMP_PULSES);
        }

        long remaining = targetPulses - average;

        if (remaining < RAMP_PULSES) {
          base_forward = MIN_SPEED_FORWARD + (DRIVE_SPEED_FORWARD - MIN_SPEED_FORWARD) * (remaining / (float)RAMP_PULSES);
        }
      }else{
        if(average < RAMP_PULSES){
          base_backward = START_SPEED_BACKWARD + (DRIVE_SPEED_BACKWARD - START_SPEED_BACKWARD) * (average / (float)RAMP_PULSES);
        }

        long remaining = targetPulses - average;

        if (remaining < RAMP_PULSES) {
          base_backward = MIN_SPEED_BACKWARD + (DRIVE_SPEED_BACKWARD - MIN_SPEED_BACKWARD) * (remaining / (float)RAMP_PULSES);
        }
      }

      long diff = pulsesA - pulsesB;
      float correction = diff * CORRECTION_GAIN;
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
      
      int leftSpeed;
      int rightSpeed;
      if (forward) {
        leftSpeed = base_forward * trim - correction;
        rightSpeed = base_forward + correction;
      } else {
        leftSpeed = base_backward * trim - correction;
        rightSpeed = base_backward + correction;
      }
      
      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);
      setMotors(leftSpeed, rightSpeed, forward);
    }
  }
}
// motor PWM puts short noise spikes on the encoder lines (right motor -> encoder A).
// a real slot pulse stays HIGH for milliseconds, a spike for a few microseconds.
const int GLITCH_SAMPLES = 5;
const int GLITCH_SAMPLE_US = 10; // 5 x 10 us = must stay HIGH ~50 us

bool stableHigh(uint8_t pin) {
  for (int i = 0; i < GLITCH_SAMPLES; i++) {
    if (digitalRead(pin) == LOW) {
      return false;
    }
    delayMicroseconds(GLITCH_SAMPLE_US);
  }
  return true;
}

void wheelCounterA() {
  if(!stableHigh(RPM_1)) {
    return;
  }
  if (micros() - startTimeA > WHEEL_A_INTERVAL) {
    startTimeA = micros();
    pulse_counterA++;
    RPM_counterA++;
  }
}

void wheelCounterB() {
  if(!stableHigh(RPM_2)) {
    return;
  }
  if (micros() - startTimeB > WHEEL_B_INTERVAL) {
    startTimeB = micros();
    pulse_counterB++;
    RPM_counterB++;
  }
}

void rotate(bool right, float angle) {
  resetCounters();

  float factor = right ? ROTATE_FACTOR_RIGHT : ROTATE_FACTOR_LEFT;
  long target = (WHEEL_WIDTH * angle / 360) / WHEEL_DIAMETER * PULSES_PER_REVOLUTION * factor + 0.5;

  if(right){  
    analogWrite(motor_LB, 0);
    analogWrite(motor_RB, ROTATE_SPEED + 15);
    analogWrite(motor_LF, ROTATE_SPEED);
    analogWrite(motor_RF, 0);
    
  } else {
    analogWrite(motor_LB, ROTATE_SPEED);
    analogWrite(motor_RB, 0);
    analogWrite(motor_LF, 0);
    analogWrite(motor_RF, ROTATE_SPEED + 15);
  }

  unsigned long start = millis();
  bool leftDone = false;
  bool rightDone = false;

  while (!(leftDone && rightDone)) {
    noInterrupts();
    long a = pulse_counterA;
    long b = pulse_counterB;
    interrupts();

    if(!leftDone && a >= target) {
      analogWrite(motor_LF, 0);
      analogWrite(motor_LB, 0);
      leftDone = true;
    }
    if(!rightDone && b >= target) {
      analogWrite(motor_RF, 0);
      analogWrite(motor_RB, 0);
      rightDone = true;
    }

    if (millis()- start > TURN_TIMEOUT) {
      Serial.println("turn timeout");
      break;
    }
  }
  stopBothMotors();
}

// spins each motor alone and prints which encoder counted.
// watch WHICH physical wheel spins first (should be the left one)
void wheelCheck() {
  for (int i = 0; i < 2; i++) {
    bool leftMotor = (i == 0);
    resetCounters();
    setMotors(leftMotor ? 200 : 0, leftMotor ? 0 : 200, true);
    delay(700);
    stopBothMotors();
    delay(300);
    noInterrupts();
    long a = pulse_counterA;
    long b = pulse_counterB;
    interrupts();
    Serial.print(leftMotor ? "LEFT motor pins  -> A: " : "RIGHT motor pins -> A: ");
    Serial.print(a);
    Serial.print("  B: ");
    Serial.println(b);
    delay(500);
  }
}

void arcTurn(bool right, float angle, float radius) {
  resetCounters();

  if (angle <= 0) {
    return;
  }

  float factor = right ? TURN_FACTOR_RIGHT : TURN_FACTOR_LEFT;

  // angle in rad
  float angleRad = angle * PI / 180.0;
  float width = right ? TURN_RIGHT : TURN_LEFT;

  // must use the same width as the geometry below, otherwise innerDist goes negative
  if (radius < width / 2.0) {
    Serial.println("Radius too small");
    return;
  }


  //distance wheel travels
  float outerDist = angleRad * (radius + width / 2.0);
  float innerDist = angleRad * (radius - width / 2.0);

  // distance - pulses
  float circumference = PI * WHEEL_DIAMETER;
  long outerTarget = (outerDist / circumference) * PULSES_PER_REVOLUTION * factor + 0.5;
  long innerTarget = (innerDist / circumference) * PULSES_PER_REVOLUTION * factor + 0.5;

  float ratio = innerDist / outerDist;

  unsigned long lastControlTime = millis();
  unsigned long turnStart = millis();
  // timeout scales with the arc length (a 150 cm quarter arc is ~280 pulses)
  unsigned long timeout = max(TURN_TIMEOUT, (unsigned long)outerTarget * ARC_MS_PER_PULSE);

  bool outerDone = false;
  bool innerDone = (ratio < 0.05); // radius == W/2 -> inner wheel stands still

  while(true) {
    if(millis() - lastControlTime < CONTROL_INTERVAL) {
      continue;
    }

    lastControlTime = millis();

    noInterrupts();
    long pulsesA = pulse_counterA;
    long pulsesB = pulse_counterB;
    interrupts();

    long leftCount  = ENCODER_A_IS_LEFT ? pulsesA : pulsesB;
    long rightCount = ENCODER_A_IS_LEFT ? pulsesB : pulsesA;
    long outer = right ? leftCount : rightCount;
    long inner = right ? rightCount : leftCount;

    // each wheel stops on ITS OWN target -> final heading is right even if the
    // wheels drifted apart during the arc (same idea as rotate())
    if (!outerDone && outer >= outerTarget) outerDone = true;
    if (!innerDone && inner >= innerTarget) innerDone = true;
    if (outerDone && innerDone) {
      break;
    }

    if (millis() - turnStart >= timeout) {
      Serial.println("arc timeout");
      break;
    }

    int outerPWM = 0;
    int innerPWM = 0;

    if (outerDone || innerDone) {
      // one wheel finished, the other one catches up alone (only a few pulses!)
      long gap = outerDone ? (innerTarget - inner) : (outerTarget - outer);
      if (gap > ARC_MAX_CATCHUP) {
        Serial.print("arc: wheels too far apart (");
        Serial.print(gap);
        Serial.println(" pulses) - check ENCODER_A_IS_LEFT");
        break;
      }
      if (!outerDone) outerPWM = ARC_CATCHUP_PWM;
      if (!innerDone) innerPWM = ARC_CATCHUP_PWM;
    } else {
      float outerSpeed = TURN_OUTER_SPEED;
      long remaining = outerTarget - outer;

      if (remaining < SLOW_DOWN) {
        outerSpeed = TURN_INNER_SPEED + (TURN_OUTER_SPEED - TURN_INNER_SPEED) * (remaining / SLOW_DOWN);
      }

      // wheel speed is ~proportional to (PWM - stall), not to PWM.
      // old version: inner = 220 * 0.8 = 176 PWM, which really turns at only ~half the outer speed
      float innerSpeed = TURN_PWM_STALL + (outerSpeed - TURN_PWM_STALL) * ratio;

      float expectedInner = outer * ratio;
      float error = expectedInner - inner;
      float correction = error * TURN_CORRECTION;
      correction = constrain(correction, -MAXIMUM_CORRECTION, MAXIMUM_CORRECTION);

      innerSpeed += correction;
      outerSpeed -= correction;

      innerPWM = constrain(innerSpeed, TURN_INNER_SPEED, 255);
      outerPWM = constrain(outerSpeed, TURN_INNER_SPEED, 255);
    }

    int leftPWM  = right ? outerPWM : innerPWM;
    int rightPWM = right ? innerPWM : outerPWM;
    // left motor is stronger, same trim as driveStraight
    leftPWM = leftPWM * TRIM_FORWARD;

    setMotors(leftPWM, rightPWM, true);
  }
    
  stopBothMotors();

  // wait for coasting, then read final counts
  delay(300);
  noInterrupts();
  long finalA = pulse_counterA;
  long finalB = pulse_counterB;
  interrupts();
  long finalLeft  = ENCODER_A_IS_LEFT ? finalA : finalB;
  long finalRight = ENCODER_A_IS_LEFT ? finalB : finalA;
  long finalOuter = right ? finalLeft : finalRight;
  long finalInner = right ? finalRight : finalLeft;
  // debug
  
  Serial.print("Outer: ");
  Serial.print(finalOuter);
  Serial.print(" / ");
  Serial.print(outerTarget);
  Serial.print("  Inner: ");
  Serial.print(finalInner);
  Serial.print(" / ");
  Serial.print(innerTarget);
  Serial.print("  Ratio: ");
  Serial.println(ratio);
}