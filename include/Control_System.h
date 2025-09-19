#pragma once

#include <Servo.h>
#include <Arduino.h>

class ControlSystem
{
private:
    const int tolerance = 20;

    byte _currentPosition = 90;
    bool _inHysterisMode = false;
    Servo _pvServo;

public:
    ControlSystem() {}
    ~ControlSystem() {}

    void begin(uint8_t pin)
    {
        _pvServo.attach(pin);
        _pvServo.write(_currentPosition);
    }

    void runControl(float west, float east)
    {
        float err = west - east;
        const int startMovingThreshold = 10; // e.g. start moving if err > 10
        const int stopMovingThreshold = 30;  // e.g. stop moving once err > 30

        if (_inHysterisMode)
        {
            if (abs(err) > stopMovingThreshold)
            {
                _inHysterisMode = false; // Break out of hysteresis and start moving again.
            }
            return;
        }
        else
        {
            if (err > startMovingThreshold)
            {
                _currentPosition++;
            }
            else if (err < -startMovingThreshold)
            {
                _currentPosition--;
            }
            else
            {
                _inHysterisMode = true;
            }
        }

        _currentPosition = constrain(_currentPosition, 0, 180);
        _pvServo.write(_currentPosition);
    }
};