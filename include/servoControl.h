#include <Arduino.h>
#include "positionControl.h"
#include <Servo.h>

class servoControl
{
private:
    /* data */
    Servo servo;
    int minVal = 0;
    int maxVal = 180;

    // Variables to handle slow movement
    uint32_t move_time = 300; // time of servo movement in milli seconds
    unsigned long move_start_time;
    int start_angle = -1;
    int target_angle;
    bool is_slow_moving = false;
public:
    servoControl();    
    void target(int val, uint16_t speed);
    void run();
    uint8_t attach(int pin, int min, int max);
    int current_angle = -1;
    ~servoControl();

private:
    void write(int val);
};

