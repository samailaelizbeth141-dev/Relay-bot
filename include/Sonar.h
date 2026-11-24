#pragma once
#include <Arduino.h>

const unsigned int SONAR_NOTHING = 400;

void SonarPinSetup();
unsigned int SonarCheck();
unsigned int SonarCheckNow();