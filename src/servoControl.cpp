#include "servoControl.h"

#define MIN_PULSE 500
#define MAX_PULSE 2500

servoControl::servoControl()
{
}

// Speed in deg/s
void servoControl::target(int val, uint16_t speed)
{
    // Go to val in ms
    if (val < minVal)
        val = minVal;
    else if (val > maxVal)
        val = maxVal;
    if (speed == 0 || current_angle == -1)
    {
        write(val);
    }
    else
    {
        move_start_time = millis();
        target_angle = val;
        start_angle = current_angle;
        move_time = (uint32_t)(abs(target_angle - start_angle)) * 1000 / speed;
        is_slow_moving = true;
        run();
    }
}

void servoControl::write(int val)
{
    current_angle = val;
    int pulse = map(val, 0, 180, MIN_PULSE, MAX_PULSE);
    servo.writeMicroseconds(pulse);
    // servo.write(val);
    
    // Serial.println(val);
}

void servoControl::run(void)
{
    if (!is_slow_moving)
        return;

    unsigned long progress = millis() - move_start_time;
    if (progress < move_time)
    {
        int angle = map(progress, 0, move_time, start_angle, target_angle);
        if (angle != current_angle)
            write(angle);
    }
    else
    {
        // Done moving
        is_slow_moving = false;
        write(target_angle);
    }
}

uint8_t servoControl::attach(int pin, int min, int max)
{
    minVal = min;
    maxVal = max;
    return servo.attach(pin, min, max);
}

servoControl::~servoControl()
{
}
