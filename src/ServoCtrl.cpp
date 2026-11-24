#include "ServoCtrl.h"

static const unsigned int SERVO_MIN_US = 1000; // impuls for 0 degres
static const unsigned int SERVO_MAX_US = 2000; // impuls for 180 degres
static const unsigned int SERVO_PULSES = 20;

void ServoPinSetup(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void ServoAngleSet(uint8_t pin, uint8_t angle)
{
    angle = constrain(angle, 0, 180);
    unsigned int pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

    for (unsigned int i = 0; i < SERVO_PULSES; i++)
    {
        digitalWrite(pin, HIGH);
        delayMicroseconds(pulseUs);
        digitalWrite(pin, LOW);
        delay(18);
    }
}