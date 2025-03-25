#include "timer5PWM.h"
#include "MotorDC.h"
#include <Arduino.h>

MotorDC::MotorDC(int fwdPin, int revPin, int sensePin, bool normallyOpen, int rampTimeMs)
{
    _fwdPin = fwdPin;
    _revPin = revPin;
    _sensePin = sensePin;
    _normallyOpen = normallyOpen;
    _rampTimeMs = rampTimeMs;
    pinMode(_fwdPin, OUTPUT);
    pinMode(_revPin, OUTPUT);
    pinMode(_sensePin, INPUT);
    state = MotorDC_fsm::STOP;
    _prevWasForward = true;
}

void MotorDC::moveToLimit(uint8_t speed, uint8_t holdSpeed)
{
    _holdSpeed = holdSpeed;
    _speed = speed;
    start_time = millis();
    if (_prevWasForward)
        reverse(0);
    else
        forward(0);
}

void MotorDC::stop()
{
    setPWM(_fwdPin, 0);
    digitalWrite(_revPin, LOW);
    state = MotorDC_fsm::STOP;
}

void MotorDC::run(){
    if (state == MotorDC_fsm::STOP)
        return;
    if (state == MotorDC_fsm::HOLDING)
        return;
    if (isLimitReached()){
        hold();
        return;
    }

    unsigned long elapsed = millis() - start_time;
    unsigned long rampTime = (unsigned long)(_speed) * _rampTimeMs / 255;
    uint8_t speed = _speed;
    if (elapsed < rampTime)
        speed = (uint8_t)(255 * elapsed / _rampTimeMs);
    
    if (state == MotorDC_fsm::FORWARD)
        forward(speed);
    else
        reverse(speed);
    
}

bool MotorDC::isLimitReached()
{
    if (millis() - start_time < 800)
        return false;
    return digitalRead(_sensePin) != _normallyOpen;
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
    } else {
        setPWM(_fwdPin, 0xFF - _holdSpeed);
        digitalWrite(_revPin, HIGH);
    }
    state = MotorDC_fsm::HOLDING;
}