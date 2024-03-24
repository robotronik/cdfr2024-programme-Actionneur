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
    servo.write(val);
    //Bug To Fix
    //posControl.setConsigne(val);
}

void servoControl::run(void){
    //Bug To Fix
    // int i = posControl.getPostion();
    // servo.write(i);
}

uint8_t servoControl::attach(int pin){
    return servo.attach(pin);
}

servoControl::~servoControl()
{
}
