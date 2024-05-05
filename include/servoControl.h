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

    // Variables to handle slow movement
    bool move_slow = false;
    unsigned long move_time = 100; // time of servo movement in milli seconds
    unsigned long move_start_time;
    bool is_moving = false;
    int start_angle = 0;
    int stop_angle;
public:
    servoControl(bool move_slow = false);
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

