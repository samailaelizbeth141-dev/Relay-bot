#include <Arduino.h>

#define motor_LB 6 // left back
#define motor_LF 11 // left forward
#define motor_RB 5 // right back
#define motor_RF 9 // right forward

#define RPM_1 3
#define RPM_2 2

const int READ_RPM_INTERVAL = 1000;

unsigned long startTime = 0;

volatile int RPM_A = 0;
volatile int RPM_B = 0;
unsigned long RPM_counterA = 0;
unsigned long RPM_counterB = 0;


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
  goBothForward(170);
  RPM_Calc();
}

void RPM_Calc() {
  if (millis() - startTime > READ_RPM_INTERVAL) {
    

    Serial.println("RPM 1 ");
    Serial.println(digitalRead(RPM_1));
    Serial.println("RPM 2 ");
    Serial.println(digitalRead(RPM_2));
    startTime = millis();
  }
}

void goBothForward(int speed) {
  analogWrite(motor_LB, 0);
  analogWrite(motor_LF, speed);
  analogWrite(motor_RB, 0);
  analogWrite(motor_RF, speed);
}

void wheelCounterA() {
  RPM_counterA++;
}

void wheelCounterB() {
  RPM_counterB++;
}