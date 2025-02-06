#pragma once

#include "Arduino.h"

#define BUFFERONRECEIVESIZE 32

//Pin Servo moteur
#define PIN_SERVOMOTEUR_1 7
#define PIN_SERVOMOTEUR_2 6
#define PIN_SERVOMOTEUR_3 9
#define PIN_SERVOMOTEUR_4 8
#define PIN_SERVOMOTEUR_5 11
#define PIN_SERVOMOTEUR_6 10
#define PIN_SERVOMOTEUR_7 13

//Pin Moteur pas a pas VALEUR HAUTE 2200 avec Microstep 1/8
#define PIN_STEPPER_SLEEP 22
#define PIN_STEPPER_RESET 23
#define PIN_STEPPER_STEP_1 2
#define PIN_STEPPER_DIR_1 42
#define PIN_STEPPER_ENABLE_1 47

#define PIN_STEPPER_STEP_2 3
#define PIN_STEPPER_DIR_2 43
#define PIN_STEPPER_ENABLE_2 40

#define PIN_STEPPER_STEP_3 4
#define PIN_STEPPER_DIR_3 28
#define PIN_STEPPER_ENABLE_3 30

#define PIN_STEPPER_STEP_4 5
#define PIN_STEPPER_DIR_4 29
#define PIN_STEPPER_ENABLE_4 31

//Pin Actionneur
#define PIN_ACTIONNEUR_1 44

// DC Motor Pins
#define PIN_MOTEURDC_REVERSE_1 45
#define PIN_MOTEURDC_FORWARD_1 12

// Sensor Pins
#define PIN_SENSOR_1 32
#define PIN_SENSOR_2 33
#define PIN_SENSOR_3 34
#define PIN_SENSOR_4 35
#define PIN_SENSOR_5 36
#define PIN_SENSOR_6 37
#define PIN_SENSOR_7 38
#define PIN_SENSOR_8 39

// LED Pins
#define PIN_LED_1_R 66 //A12
#define PIN_LED_1_G 65 //A11
#define PIN_LED_1_B 64 //A10

// PWM Lidar
#define PIN_PWM_LIDAR 46

//configuration Stepper
#define DEFAULT_MAX_SPEED 5000  // Vitesse maximale (en pas par seconde)
#define DEFAULT_MAX_ACCEL 8000 // Accélération maximale (en pas par seconde carré)