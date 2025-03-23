#include "timer5PWM.h"
#include <Arduino.h>


// Configure Timer5 for Fast PWM Mode, Non-inverting, No prescaler (PWM Pin 44, 45 and 46)
void configTMR5() {
    TCCR5A = (1 << COM5A1) | (1 << COM5B1) | (1 << COM5C1) | (1 << WGM51);
    TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS50);
    ICR5 = 799; // 20kHz (16MHz / 800 = 20kHz)
    pinMode(44, OUTPUT);
    pinMode(45, OUTPUT);
    pinMode(46, OUTPUT);
  }

  void setPWM(int pin, uint8_t val) {
    switch (pin) {
      case 44:
        setPWM_P44(val);
        break;
      case 45:
        setPWM_P45(val);
        break;
      case 46:
        setPWM_P46(val);
        break;
    }
  }
  
  // Set PWM for pin 44 (0-255)
  void setPWM_P44(uint8_t val) {
    OCR5C = map(val, 0, 255, 0, ICR5);
    pinMode(44, val == 0 ? INPUT : OUTPUT);
  }
  
  // Repeat for pins 45 and 46
  void setPWM_P45(uint8_t val) {
    OCR5B = map(val, 0, 255, 0, ICR5);
    pinMode(45, val == 0 ? INPUT : OUTPUT);
  }
  
  void setPWM_P46(uint8_t val) {
    OCR5A = map(val, 0, 255, 0, ICR5);
    pinMode(46, val == 0 ? INPUT : OUTPUT);
  }