#include "Drive.h"
#include "Arduino.h"
#include "Wheels.h"

Drive::Drive(Wheel &left, Wheel &right, volatile long &leftCount, volatile long &rightCount)
    : leftWheel(left), rightWheel(right), encoder_count_left(leftCount), encoder_count_right(rightCount)
{
}

void Drive::forward(int distance)
{
    while (encoder_count_left < distance || encoder_count_right < distance)
    {
        leftWheel.forward();
        rightWheel.forward();
    }
}

void Drive::backward(int distance)
{
    while (encoder_count_left < distance || encoder_count_right < distance)
    {
        leftWheel.backward();
        rightWheel.backward();
    }
}

void Drive::left()
{
    leftWheel.backward();
    rightWheel.forward();
}

void Drive::right()
{
    leftWheel.forward();
    rightWheel.backward();
}

void Drive::stop()
{
    leftWheel.brake();
    rightWheel.brake();
}
