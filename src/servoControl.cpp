#include "servoControl.h"

#define MIN_PULSE 500
#define MAX_PULSE 2500

Adafruit_PWMServoDriver servoControl::pca = Adafruit_PWMServoDriver();
bool servoControl::pcaInitialized = false;
uint8_t servoControl::pcaAddress = 0x40;
uint16_t servoControl::pcaFrequency = 50;

servoControl::servoControl()
{
    channel = 0;
    start_angle = -1;
    is_slow_moving = false;
    move_time = 300;
    current_angle = -1;
}

bool servoControl::beginPCA9685(uint8_t address, uint16_t frequency)
{
    pcaAddress = address;
    pcaFrequency = frequency;
    pca = Adafruit_PWMServoDriver(pcaAddress);
    if (!pca.begin())
    {
        pcaInitialized = false;
        return false;
    }

    pca.setPWMFreq(pcaFrequency);
    delay(10);
    pcaInitialized = true;
    return true;
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
        is_slow_moving = false;
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
    if (!pcaInitialized)
        return;

    current_angle = val;
    int pulse_us = map(val, 0, 180, MIN_PULSE, MAX_PULSE);
    uint16_t pulse_ticks = (uint32_t)pulse_us * pcaFrequency * 4096UL / 1000000UL;
    pca.setPWM(channel, 0, pulse_ticks);
#ifdef SERIAL_DEBUG
    Serial.print("Servo channel #");
    Serial.print(channel);
    Serial.print(" set ");
    Serial.println(val);
#endif
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

uint8_t servoControl::attach(int pca_channel, int min, int max)
{
    minVal = min;
    maxVal = max;
    start_angle = -1;
    is_slow_moving = false;
    move_time = 300;
    current_angle = -1;
    channel = (uint8_t)pca_channel;
    return (pcaInitialized && channel < 16) ? 1 : 0;
}

servoControl::~servoControl()
{
}
