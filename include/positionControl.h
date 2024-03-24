#pragma once 

#include <math.h>
#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include "clock.h"
#endif


#ifdef ARDUINO
    #define GET_TIME_MS millis
#else
    #define GET_TIME_MS get_uptime_ms
#endif


class positionControl{

public :
    double vitesseMaxAv = -1;
    double accelerationMaxAv = -1; 
    double decelerationMaxAv = -1;
    double vitesseMaxAr = -1;
    double accelerationMaxAr = -1; 
    double decelerationMaxAr = -1;
private :
    /* data */
    double position;
    double consigne;
    double vitesse;
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
