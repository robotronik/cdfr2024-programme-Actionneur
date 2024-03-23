#include <Arduino.h>
#include "positionControl.h"
#include <Servo.h>


class servoControl
{
private:
    /* data */
    Servo servo;
    positionControl posControl;
    int minVal;
    int maxVal;
public:
    servoControl(/* args */);
    void setMinValue(int min);
    void setMaxValue(int max);
    void setMinMaxValue(int min, int max);
    void write(int val);
    uint8_t attach(int pin);
    ~servoControl();
};

