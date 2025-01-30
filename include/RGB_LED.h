#pragma once

#define MODE_SOLID 0
#define MODE_BLINK 1
#define MODE_RAINBOW 2

class RGB_LED{
public :
    RGB_LED(int pinR, int pinG, int pinB);
    ~RGB_LED();
    void run();
    void recieveData(uint8_t* data);
private :
    unsigned char R, G, B;
    unsigned char R_S, G_S, B_S;
    unsigned char mode;
    int pinR, pinG, pinB;
    void setColor(unsigned char R, unsigned char G, unsigned char B);
    void setMode(unsigned char mode);
    void solid();
    void blink();
    void rainbow();
    void setLED(bool R, bool G, bool B);
};
