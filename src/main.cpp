#include <Arduino.h>

#define motor_LB 10 // left back
#define motor_LF 11 // left forward
#define motor_RB 5 // right back
#define motor_RF 6 // right forward

void setup() {
  pinMode(motor_LB, OUTPUT);
  pinMode(motor_LF, OUTPUT);
  pinMode(motor_RB, OUTPUT);
  pinMode(motor_RF, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  analogWrite(motor_LB, 0);
  analogWrite(motor_LF, 255);
}