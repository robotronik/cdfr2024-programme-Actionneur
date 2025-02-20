#include "RGB_LED.h"
#include "utils.h"
#include <Arduino.h>

RGB_LED::RGB_LED(int pinR, int pinG, int pinB){
    pinMode(pinR, OUTPUT);
    pinMode(pinG, OUTPUT);
    pinMode(pinB, OUTPUT);
    this->pinR = pinR;
    this->pinG = pinG;
    this->pinB = pinB;
    setColor(0,0,0);
}

RGB_LED::~RGB_LED(){
    digitalWrite(pinR, LOW);
    digitalWrite(pinG, LOW);
    digitalWrite(pinB, LOW);
}

void RGB_LED::recieveData(unsigned char* data){
    uint8_t _mode = ReadUInt8(&data);
    setMode(_mode);
    uint8_t _R, _G, _B;
    switch (_mode)
    {
    case MODE_SOLID:
        _R = ReadUInt8(&data);
        _G = ReadUInt8(&data);
        _B = ReadUInt8(&data);
        setColor(_R, _G, _B);
        break;
    case MODE_BLINK:
        _R = ReadUInt8(&data);
        _G = ReadUInt8(&data);
        _B = ReadUInt8(&data);
        setColor(_R, _G, _B);
        break;
    case MODE_RAINBOW:
        break;      
    default:
        break;
    }
}

void RGB_LED::run(){
    static unsigned char counter = 0;
    setLED(counter < R, counter < G, counter < B);
    counter++;
    switch(mode){
        case MODE_SOLID:
            solid();
            break;
        case MODE_BLINK:
            blink();
            break;
        case MODE_RAINBOW:
            if (counter % 128 == 0)
                rainbow();
            break;
        default:
            mode = MODE_SOLID;
            break;
    }
}

void RGB_LED::setColor(unsigned char R, unsigned char G, unsigned char B){
    this->R_S = R;
    this->G_S = G;
    this->B_S = B;
}

void RGB_LED::setLED(bool R, bool G, bool B){
    digitalWrite(pinR, R);
    digitalWrite(pinG, G);
    digitalWrite(pinB, B);
}

void RGB_LED::setMode(unsigned char mode){
    this->mode = mode;
    if (mode == MODE_RAINBOW){
        R = 255; G = 0; B = 0;
    }
}

void RGB_LED::solid(){
    R = R_S; G = G_S; B = B_S;
}

void RGB_LED::blink(){
    if (millis() % 500 < 300){
        R = 0; G = 0; B = 0;
    }else{
        R = R_S; G = G_S; B = B_S;
    }
}

void RGB_LED::rainbow(){
    static int state = 0;
    switch(state){
        case 0: // Red -> Yellow
            if (G < 255) {
                (G)++;
            } else {
                state = 1;
            }
            break;
            
        case 1: // Yellow -> Green
            if (R > 0) {
                (R)--;
            } else {
                state = 2;
            }
            break;
            
        case 2: // Green -> Cyan
            if (B < 255) {
                (B)++;
            } else {
                state = 3;
            }
            break;
            
        case 3: // Cyan -> Blue
            if (G > 0) {
                (G)--;
            } else {
                state = 4;
            }
            break;
            
        case 4: // Blue -> Magenta
            if (R < 255) {
                (R)++;
            } else {
                state = 5;
            }
            break;
            
        case 5: // Magenta -> Red
            if (B > 0) {
                (B)--;
            } else {
                state = 0;
            }
            break;
    }
}