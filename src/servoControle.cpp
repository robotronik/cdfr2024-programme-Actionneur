#include "servoControl.h"

servoControl::servoControl(/* args */)
{
}

void servoControl::setMinValue(int min){
    minVal = min;
}
void servoControl::setMaxValue(int max){
    maxVal = max;
}
void servoControl::setMinMaxValue(int min, int max){
    minVal = min;
    maxVal = max;
}

void servoControl::write(int val){
    if(val<minVal){
        val = minVal;
    }
    else if(val>maxVal){
        val = maxVal;
    }
    servo.write(val);
}

uint8_t servoControl::attach(int pin){
    return servo.attach(pin);
}

servoControl::~servoControl()
{
}
