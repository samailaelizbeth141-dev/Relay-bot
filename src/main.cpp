#include <Arduino.h>


int counter = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);   // D1 (TX) -> HR: USB-re és Bluetoothra is megy
  Serial.println("RelayBot BT teszt");
}

void loop() {
  Serial.print("Counter: ");
  Serial.println(counter++);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(1000);
}