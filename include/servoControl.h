#include <Arduino.h>
#include "positionControl.h"
#include <Servo.h>


class servoControl
{
private:
    /* data */
    Servo servo;
    positionControl posControl;
    int minVal = 0;
    int maxVal = 180;
public:
    servoControl(/* args */);
    void setMinValue(int min);
    void setMaxValue(int max);
    void setMinMaxValue(int min, int max);

    void setParamater(
        double vitesseMaxAv,
        double accelerationMaxAv,
        double decelerationMaxAv,
        double vitesseMaxAr,
        double accelerationMaxAr,
        double decelerationMaxAr
    );
    
    void write(int val);
    void run(void);
    uint8_t attach(int pin);
    ~servoControl();
};

