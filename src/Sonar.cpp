#include "Sonar.h"

static const uint8_t pinTrig = 13;
static const uint8_t pinEcho = 12;
static const unsigned long SONAR_TIMEOUT = 8000;
static const unsigned long SONAR_INTERVAL = 60;
static unsigned long timeSonar = 0;
static unsigned int lastDistance;

static unsigned int SonarDistanceCM()
{
    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);

    unsigned long echoTime = pulseIn(pinEcho, HIGH, SONAR_TIMEOUT);
    if (echoTime == 0)
    {
        return SONAR_NOTHING;
    }

    return echoTime / 58;
}

void SonarPinSetup()
{
    pinMode(pinTrig, OUTPUT);
    pinMode(pinEcho, INPUT);
    digitalWrite(pinTrig, LOW);
    lastDistance = SonarDistanceCM();
    timeSonar = millis();
}

unsigned int SonarCheck()
{
    if (millis() - timeSonar >= SONAR_INTERVAL)
    {
        timeSonar = millis();
        lastDistance = SonarDistanceCM();
        return lastDistance;
    }
    return lastDistance;
}

unsigned int SonarCheckNow()
{
    lastDistance = SonarDistanceCM();
    timeSonar = millis();
    return lastDistance;
}