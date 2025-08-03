#include "Arduino.h"
#include "Wheels.h"

Wheel::Wheel(byte controlPinA, byte controlPinB, byte speedPin) {
    _controlPinA = controlPinA;
    _controlPinB = controlPinB;
    _speedPin = speedPin;

    pinMode(controlPinA, OUTPUT);
    pinMode(controlPinB, OUTPUT);
    pinMode(speedPin, OUTPUT);
}

void Wheel::setSpeed(int speed) {
    analogWrite(_speedPin, speed);
}

void Wheel::forward() {
    digitalWrite(_controlPinA, HIGH);
    digitalWrite(_controlPinB, LOW);
}

void Wheel::backward() {
    digitalWrite(_controlPinA, LOW);
    digitalWrite(_controlPinB, HIGH);
}

void Wheel::stop() {
    analogWrite(_speedPin, 0);
}

void Wheel::brake() {
    digitalWrite(_controlPinA, LOW);
    digitalWrite(_controlPinA, LOW);
}