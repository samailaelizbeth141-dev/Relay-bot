#include <Arduino.h>

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

float RPM_A = 0;
float RPM_B = 0;
volatile unsigned long RPM_counterA = 0;
volatile unsigned long RPM_counterB = 0;
volatile unsigned long pulse_counterA = 0;
volatile unsigned long pulse_counterB = 0;

const float WHEEL_DIAMETER = 6; // in cm
const int PULSES_PER_REVOLUTION = 20; // number of pulses per revolution of the wheel
const int MIN_SPEED = 130; // minimum speed to prevent stalling
const int START_SPEED = 150; // speed to start the wheels at
const float TRIM_FORWARD = 0.86;
const float TRIM_BACKWARD = 0.86;
const float CORRECTION_GAIN = 0.0;
const float MAX_CORRECTION = 15.0; // maximum correction to apply to the speed of the faster wheel
const int CONTROL_INTERVAL = 20; // interval in milliseconds to check and adjust the speed of the wheels
const int RAMP_PULSES = 15; // number of pulses to ramp up the speed of the wheels
const int DRIVE_SPEED = 190; // speed to drive the wheels at

bool done = false;

void RPM_Calc();
void wheelCounterA();
void wheelCounterB();
void resetCounters();
void stopBothMotors();
void setMotors(int leftSpeed, int rightSpeed, bool forward);
void driveStraight(bool forward, float distance);

// const int MAX_SPEED = 0;
// read the wheels rpm, and correct the faster one to the slower one
void setup() {
  pinMode(motor_LB, OUTPUT);
  pinMode(motor_LF, OUTPUT);
  pinMode(motor_RB, OUTPUT);
  pinMode(motor_RF, OUTPUT);
  pinMode(RPM_1, INPUT_PULLUP);
  pinMode(RPM_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RPM_1), wheelCounterA, RISING);
  attachInterrupt(digitalPinToInterrupt(RPM_2), wheelCounterB, RISING);

  Serial.begin(9600);
  Serial.println("Bluetooth connected!");
  startTime = millis();
  delay(2000);
}

void loop() {
  if(!done){
    driveStraight(true, 100);
    delay(2000);
    driveStraight(false, 100);
    delay(2000);
    done = true;
  }else{
    stopBothMotors();
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
      float base = DRIVE_SPEED;
      if(average < RAMP_PULSES){
        base = START_SPEED + (DRIVE_SPEED - START_SPEED) * (average / (float)RAMP_PULSES);
      }

      long remaining = targetPulses - average;

      if (remaining < RAMP_PULSES) {
        base = MIN_SPEED + (DRIVE_SPEED - MIN_SPEED) * (remaining / (float)RAMP_PULSES);
      }

      long diff = pulsesA - pulsesB;
      float correction = diff * CORRECTION_GAIN;
      correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
      int leftSpeed = base * trim - correction;
      int rightSpeed = base + correction;
      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);
      setMotors(leftSpeed, rightSpeed, forward);
    }
  }
}
void wheelCounterA() {
  if(digitalRead(RPM_1) == LOW) {
    return;
  }
  if (micros() - startTimeA > WHEEL_A_INTERVAL) {
    startTimeA = micros();
    pulse_counterA++;
    RPM_counterA++;
  }
}

void wheelCounterB() {
  if(digitalRead(RPM_2) == LOW) {
    return;
  }
  if (micros() - startTimeB > WHEEL_B_INTERVAL) {
    startTimeB = micros();
    pulse_counterB++;
    RPM_counterB++;
  }
}