#include "timer5PWM.h"
#include "MotorDC.h"
#include <Arduino.h>

extern const int* sensor_pins;

MotorDC::MotorDC(int fwdPin, int revPin)
{
    _fwdPin = fwdPin;
    _revPin = revPin;
    pinMode(_fwdPin, OUTPUT);
    pinMode(_revPin, OUTPUT);
    state = MotorDC_fsm::STOP;
    
    // Initialize soft start parameters
    _targetSpeed = 0;
    _currentSpeed = 0;
    _rampRate = 1;  // Default: 1 speed unit per run() call (~51ms per 255 units)
    _isForward = true;
}

void MotorDC::moveSpeed(uint8_t speed, bool is_forward)
{
    _targetSpeed = speed;
    _isForward = is_forward;
}

void MotorDC::stop()
{
    _targetSpeed = 0;
}

void MotorDC::run(){
    // Ramp current speed towards target speed
    if (_currentSpeed < _targetSpeed) {
        // Ramp up
        _currentSpeed = (_currentSpeed + _rampRate <= _targetSpeed) 
                        ? _currentSpeed + _rampRate 
                        : _targetSpeed;
    } else if (_currentSpeed > _targetSpeed) {
        // Ramp down
        _currentSpeed = (_currentSpeed - _rampRate >= _targetSpeed) 
                        ? _currentSpeed - _rampRate 
                        : _targetSpeed;
    }
    
    // Apply ramped speed to motor
    if (_currentSpeed > 0) {
        if (_isForward) {
            setPWM(_fwdPin, _currentSpeed);
            setPWM(_revPin, 0);
            state = MotorDC_fsm::FORWARD;
        } else {
            setPWM(_fwdPin, 0);
            setPWM(_revPin, _currentSpeed);
            state = MotorDC_fsm::REVERSE;
        }
    } else {
        setPWM(_fwdPin, 0);
        setPWM(_revPin, 0);
        state = MotorDC_fsm::STOP;
    }
}

void MotorDC::setRampRate(uint8_t rate)
{
    _rampRate = rate;
}
void MotorDC::forward(uint8_t speed)
{
    setPWM(_fwdPin, speed);
    setPWM(_revPin, 0);
    state = MotorDC_fsm::FORWARD;
}

void MotorDC::reverse(uint8_t speed)
{
    setPWM(_fwdPin, 0);
    setPWM(_revPin, speed);
    state = MotorDC_fsm::REVERSE;
}