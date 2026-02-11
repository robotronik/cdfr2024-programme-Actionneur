#pragma once

// define a class for MotorDC with a fsm to control the motor 
// (forward, reverse, stop, brake)
// fwdPin can only be used on pins in timer5PWM.cpp
// (44, 45, 46)

enum class MotorDC_fsm
{
    STOP = 0,
    FORWARD = 1,
    REVERSE = 2
};

class MotorDC
{
public:
    MotorDC(int fwdPin, int revPin);
    void moveSpeed(uint8_t speed, bool is_forward);
    void stop();
    void run();

    MotorDC_fsm state;

private:
    int _fwdPin;
    int _revPin;

    void forward(uint8_t speed);
    void reverse(uint8_t speed);
};