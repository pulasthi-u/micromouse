#ifndef DRIVE_H
#define DRIVE_H

#include "Arduino.h"
#include "Wheels.h"

class Drive
{
public:
    Drive();
    Drive(Wheel &left, Wheel &right, volatile long &leftCount, volatile long &rightCount);
    void forward(int distance);
    void backward(int distance);
    void left();
    void right();
    void stop();

    Wheel &leftWheel;
    Wheel &rightWheel;
    int distance;
    volatile long &encoder_count_left;
    volatile long &encoder_count_right;
};

#endif