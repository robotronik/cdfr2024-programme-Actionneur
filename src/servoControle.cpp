#include "servoControl.h"

servoControl::servoControl(bool move_slow)
{
    this->move_slow = move_slow;
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

void servoControl::setParamater(    
    double vitesseMaxAv,
    double accelerationMaxAv,
    double decelerationMaxAv,
    double vitesseMaxAr,
    double accelerationMaxAr,
    double decelerationMaxAr)
{
    posControl.vitesseMaxAv         = vitesseMaxAv;
    posControl.accelerationMaxAv    = accelerationMaxAv; 
    posControl.decelerationMaxAv    = decelerationMaxAv;
    posControl.vitesseMaxAr         = vitesseMaxAr;
    posControl.accelerationMaxAr    = accelerationMaxAr; 
    posControl.decelerationMaxAr    = decelerationMaxAr;
}

void servoControl::write(int val){
    if(val<minVal){
        val = minVal;
    }
    else if(val>maxVal){
        val = maxVal;
    }
    if (!move_slow){
        servo.write(val);
    } else {
        move_start_time = millis(); // To capture moving time
        is_moving = true;
        stop_angle = val;
    }
}

void servoControl::run(void){
    if (!move_slow || !is_moving){
        return;
    }
    unsigned long progress = millis() - move_start_time;
    if (progress <= move_time) {
        long angle = map(progress, 0, move_start_time, start_angle, stop_angle);
        myServo.write(angle); 
    } else {
        is_moving = false;
        start_angle = stop_angle; // update angle and stop moving
    }
}

uint8_t servoControl::attach(int pin){
    return servo.attach(pin);
}

servoControl::~servoControl()
{
}
