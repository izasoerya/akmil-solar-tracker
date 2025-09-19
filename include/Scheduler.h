#pragma once

#include <Arduino.h>

struct Scheduler
{
    unsigned long _prevMillis = 0;
    int16_t _timeDelay;
    void (*_task)();

    void runTask()
    {
        _prevMillis = millis();
        _task();
    }

    Scheduler(int16_t timeDelay, void (*task)()) : _timeDelay(timeDelay), _task(task) {}
};