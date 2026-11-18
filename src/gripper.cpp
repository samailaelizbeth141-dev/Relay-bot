#include "gripper.h"

const int GRIPPER_OPEN = 130;
const int GRIPPER_CLOSE = 69;

int gripperPulse = 0;
unsigned long lastGripperPulse = 0;

void servoGrab() {
  gripperPulse = map(GRIPPER_CLOSE, 0, 180, 544, 2400);
}

void servoLet() {
  gripperPulse = map(GRIPPER_OPEN, 0, 180, 544, 2400);
}

void gripperUpdate() {
  if (gripperPulse == 0) return;

  if (micros() - lastGripperPulse >= 20000) {
    lastGripperPulse = micros();
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(gripperPulse);
    digitalWrite(GRIPPER_PIN, LOW);
  }
}