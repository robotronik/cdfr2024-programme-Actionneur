#pragma once 

#include <math.h>
#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include "clock.h"
#endif


#ifdef ARDUINO
    #define GET_TIME_MS micros
#else
    #define GET_TIME_MS get_uptime_ms
#endif


class positionControl{
public:
    /* data */
    double position;
    double consigne;
    double vitesse;

    double vitesseMaxAv = 45;
    double accelerationMaxAv = 25; 
    double decelerationMaxAv = 25;
    double vitesseMaxAr = 45;
    double accelerationMaxAr = 25; 
    double decelerationMaxAr = 25;
    double deltaTemps = 0;

    uint32_t PreviousTime;

    bool stopStatus;

public:
    positionControl(double positionDepart  = 0.0);
    void reset(double initialValue);
    void stop(void);
    void setPosition(double initialValue);
    void setConsigne(double setConsigne);
    double getPostion();
    bool getMove();
    int getBrakingDistance();
    ~positionControl();
private:
    void calculVitesse();
};
