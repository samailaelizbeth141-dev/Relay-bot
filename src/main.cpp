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

int speedA = 0;
int speedB = 0;

float RPM_A = 0;
float RPM_B = 0;
volatile unsigned long RPM_counterA = 0;
volatile unsigned long RPM_counterB = 0;
volatile unsigned long pulse_counterA = 0;
volatile unsigned long pulse_counterB = 0;

const int WHEEL_DIAMETER = 6; // in cm
const int PULSES_PER_REVOLUTION = 20; // number of pulses per revolution of the wheel
const int MIN_SPEED = 130; // minimum speed to prevent stalling
const float TRIM_FORWARD = 0.86;
const float TRIM_BACKWARD = 0.86;
const float CORRECTION_GAIN = 0.0;
const float MAX_CORRECTION = 15.0; // maximum correction to apply to the speed of the faster wheel
const int CONTROL_INTERVAL = 20; // interval in milliseconds to check and adjust the speed of the wheels
const int RAMP_PULSES = 15; // number of pulses to ramp up the speed of the wheels

void RPM_Calc();
void goBothForward(int speed);
void wheelCounterA();
void wheelCounterB();
void resetCounters();
void stopBothMotors();

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
}

void loop() {
  goBothForward(190);
  RPM_Calc();
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

void goBothForward(int speed) {
  int baseLeft = speed * LEFT_TRIM;
  noInterrupts();
  long temp1 = pulse_counterA;
  long temp2 = pulse_counterB;

  interrupts();

  long diff = 0;
  if(temp1 > temp2) {
    diff = temp1 - temp2;
    int correction = diff * CORRECTION_GAIN;
    speedA = baseLeft - correction;
    speedB = speed + correction;
  }else{
    diff = temp2 - temp1;
    int correction = diff * CORRECTION_GAIN;
    speedA = baseLeft + correction;
    speedB = speed - correction;
  }
  if (millis() - startTime > 200) {
    Serial.print("Pulse A: ");
    Serial.print(temp1);
    Serial.print(" Pulse B: ");
    Serial.print(temp2);
    Serial.print(" Diff: ");
    Serial.println(diff);
    startTime = millis();
  }
  speedA = constrain(speedA, 130, speed);
  speedB = constrain(speedB, 130, speed);
  analogWrite(motor_LF, speedA);
  analogWrite(motor_RF, speedB);
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