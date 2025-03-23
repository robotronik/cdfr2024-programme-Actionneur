#include "timer5PWM.h"
#include "MotorDC.h"
#include <Arduino.h>

MotorDC::MotorDC(int fwdPin, int revPin, int sensePin, bool normallyOpen)
{
    _fwdPin = fwdPin;
    _revPin = revPin;
    _sensePin = sensePin;
    _normallyOpen = normallyOpen;
    pinMode(_fwdPin, OUTPUT);
    pinMode(_revPin, OUTPUT);
    pinMode(_sensePin, INPUT);
    state = MotorDC_fsm::STOP;
}

void MotorDC::moveToLimit(uint8_t speed, uint8_t holdSpeed)
{
    if (state != MotorDC_fsm::STOP)
        return;
    _holdSpeed = holdSpeed;
    start_time = millis();
    if (_prevWasForward)
        reverse(speed);
    else
        forward(speed);
}

void MotorDC::stop()
{
    setPWM(_fwdPin, 0);
    digitalWrite(_revPin, LOW);
    state = MotorDC_fsm::STOP;
}

void MotorDC::run(){
    switch(state){
        case MotorDC_fsm::FORWARD:
            if (isLimitReached())
                hold();
            break;
        case MotorDC_fsm::REVERSE:
            if (isLimitReached())
                hold();
            break;
        default:
            break;
    }
}

bool MotorDC::isLimitReached()
{
    if (millis() - start_time < 200)
        return false;
    return digitalRead(_sensePin) == (_normallyOpen ? HIGH : LOW);
}

void MotorDC::forward(uint8_t speed)
{
    setPWM(_fwdPin, speed);
    digitalWrite(_revPin, LOW);
    state = MotorDC_fsm::FORWARD;
    _prevWasForward = true;
}

void MotorDC::reverse(uint8_t speed)
{
    setPWM(_fwdPin, 0xFF - speed);
    digitalWrite(_revPin, HIGH);
    state = MotorDC_fsm::REVERSE;
    _prevWasForward = false;
}

void MotorDC::hold()
{
    if (_prevWasForward){
        setPWM(_fwdPin, _holdSpeed);
        digitalWrite(_revPin, LOW);
    }
    else{
        setPWM(_fwdPin, 0xFF - _holdSpeed);
        digitalWrite(_revPin, HIGH);
    }
    state = MotorDC_fsm::HOLDING;
}