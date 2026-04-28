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
    void setRampRate(uint8_t rate);  // Set ramp rate (speed increment per run() call)

    MotorDC_fsm state;

private:
    int _fwdPin;
    int _revPin;
    uint8_t _targetSpeed;   // Target speed to reach
    uint8_t _currentSpeed;  // Current speed (ramped)
    uint8_t _rampRate;      // Speed increment per run() call
    bool _isForward;        // Direction flag

    void forward(uint8_t speed);
    void reverse(uint8_t speed);
};