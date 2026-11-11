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
const float CORRECTION_GAIN = 0.0;

void RPM_Calc();
void goBothForward(int speed);
void wheelCounterA();
void wheelCounterB();


// const int MAX_SPEED = 0;
// read the wheels rpm, and correct the faster one to the slower one
void setup() {
  pinMode(motor_LB, OUTPUT);
  pinMode(motor_LF, OUTPUT);
  pinMode(motor_RB, OUTPUT);
  pinMode(motor_RF, OUTPUT);
  pinMode(RPM_1, INPUT);
  pinMode(RPM_2, INPUT);

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
    Serial.print("RPM 1 ");
    Serial.print(RPM_A);
    Serial.print(" RPM 2 ");
    Serial.println(RPM_B);
    startTime = millis();
  }
}

void goBothForward(int speed) {
  noInterrupts();
  long temp1 = pulse_counterA;
  long temp2 = pulse_counterB;

  interrupts();

  long diff = 0;
  if(temp1 > temp2) {
    diff = temp1 - temp2;
    int correction = diff * CORRECTION_GAIN;
    speedA = speed - correction;
    speedB = speed + correction;
  }else if(temp2 > temp1) {
    diff = temp2 - temp1;
    int correction = diff * CORRECTION_GAIN;
    speedA = speed + correction;
    speedB = speed - correction;
  }
  /*if (millis() - startTime > 200) {
    Serial.println(diff);
    startTime = millis();
  }*/
  speedA = constrain(speedA, 130, speed);
  speedB = constrain(speedB, 130, speed);
  analogWrite(motor_LF, speedA);
  analogWrite(motor_RF, speedB);
}

void wheelCounterA() {
  if (micros() - startTimeA > WHEEL_A_INTERVAL) {
    startTimeA = micros();
    pulse_counterA++;
    RPM_counterA++;
  }
}

void wheelCounterB() {
  if (micros() - startTimeB > WHEEL_B_INTERVAL) {
    startTimeB = micros();
    pulse_counterB++;
    RPM_counterB++;
  }
}