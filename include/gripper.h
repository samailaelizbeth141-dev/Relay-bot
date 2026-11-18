#pragma once
#include <Arduino.h>

const int GRIPPER_PIN = 4;

void servoGrab();                   // zár (megfog), majd megvárja, hogy odaérjen
void servoLet();                    // nyit (elenged), majd megvárja, hogy odaérjen
void gripperUpdate();