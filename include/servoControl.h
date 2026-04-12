#include <Arduino.h>
#include "positionControl.h"
#include <Adafruit_PWMServoDriver.h>

class servoControl
{
private:
    uint8_t channel;
    int minVal = 0;
    int maxVal = 180;

    // Variables to handle slow movement
    uint32_t move_time;
    unsigned long move_start_time;
    int start_angle;
    int target_angle;
    bool is_slow_moving;

    static Adafruit_PWMServoDriver pca;
    static bool pcaInitialized;
    static uint8_t pcaAddress;
    static uint16_t pcaFrequency;

public:
    servoControl();    
    void target(int val, uint16_t speed);
    void run();
    uint8_t attach(int channel, int min, int max);
    static bool beginPCA9685(uint8_t address = 0x40, uint16_t frequency = 50);
    int current_angle;
    ~servoControl();

private:
    void write(int val);
};

