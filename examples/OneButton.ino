#include <Arduino.h>

#define LED_PIN PB4 //PA15 黄灯  PB4 蓝灯

void setup() {
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}