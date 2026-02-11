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
}

void MotorDC::moveSpeed(uint8_t speed, bool is_forward)
{
    if (is_forward)
        forward(speed);
    else
        reverse(speed);
}

void MotorDC::stop()
{
    setPWM(_fwdPin, 0);
    setPWM(_revPin, 0);
    state = MotorDC_fsm::STOP;
}

void MotorDC::run(){
    if (state == MotorDC_fsm::STOP)
        return;
    return;
    
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