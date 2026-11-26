#include "Scanner.h"
#include "ServoCtrl.h"
#include "Sonar.h"
#include "Maze.h"

#define SONAR_SERVO_PIN 8

static const uint8_t look_left = 0;
static const uint8_t look_forward = 90;
static const uint8_t look_right = 180;
const int wallCheckWaitTime = 50;

uint8_t scanAngles[4] = {look_forward, look_left, look_forward, look_right};
int scannedDist[4] = {};

void servoSetup() {
    SonarPinSetup();
    ServoPinSetup(SONAR_SERVO_PIN);

    ServoAngleSet(SONAR_SERVO_PIN, look_forward);
}

void scanWalls() {
    for (int i = 0; i < sizeof(scanAngles) / sizeof(scanAngles[0]); i++) {
        ServoAngleSet(SONAR_SERVO_PIN,scanAngles[i]);
        delay(50);
        scannedDist[i] = SonarCheckNow();
    }    
}