#pragma once

// define a class for MotorDC with a fsm to control the motor 
// (forward, reverse, stop, brake)
// fwdPin can only be used on pins in timer5PWM.cpp
// (44, 45, 46)

enum class MotorDC_fsm
{
    STOP = 0,
    FORWARD = 1,
    REVERSE = 2,
    HOLDING = 3
};

class MotorDC
{
public:
    MotorDC(int fwdPin, int revPin, int sensePin, bool normallyOpen);
    void moveToLimit(uint8_t speed, uint8_t holdSpeed);
    void stop();
    void run();

    unsigned long start_time;
    MotorDC_fsm state;

private:
    int _fwdPin;
    int _revPin;
    int _sensePin;
    bool _normallyOpen;
    bool _prevWasForward;
    uint8_t _holdSpeed;
    void forward(uint8_t speed);
    void reverse(uint8_t speed);
    void hold();
    bool isLimitReached();
};