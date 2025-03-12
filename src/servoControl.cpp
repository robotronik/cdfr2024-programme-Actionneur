#include "servoControl.h"

#define MIN_PULSE 55
#define MAX_PULSE 284

servoControl::servoControl()
{
}
void servoControl::setMinMaxValue(int min, int max){
    minVal = min;
    maxVal = max;
}

// Speed in deg/s
void servoControl::target(int val, uint16_t speed){
    // Go to val in ms
    if(val < minVal){
        val = minVal;
    }
    else if(val > maxVal){
        val = maxVal;
    }
    if (speed == 0 || current_angle == -1){
        write(val);
    }
    else{
        move_start_time = millis();
        target_angle = val;
        start_angle = current_angle;
        move_time = abs(target_angle - start_angle) * 1000 / speed;
        is_slow_moving = true;
        run();
    }
}

void servoControl::write(int val){
    if(val<minVal){
        val = minVal;
    }
    else if(val>maxVal){
        val = maxVal;
    }
    current_angle = val;
    //int pulse = map(val, 0, 180, MIN_PULSE, MAX_PULSE);
    //servo.writeMicroseconds(pulse);
    servo.write(val);
}

void servoControl::run(void){
    if (!is_slow_moving){
        return;
    }
    Serial.println("Running");
    unsigned long progress = millis() - move_start_time;
    if (progress < move_time) {
        int angle = map(progress, 0, move_time, start_angle, target_angle);
        write(angle);
        Serial.print(angle);
    } else {
        // Done moving
        is_slow_moving = false;
        write(target_angle);
        Serial.print(target_angle);
    }
}

uint8_t servoControl::attach(int pin){
    return servo.attach(pin);
}

servoControl::~servoControl()
{
}
