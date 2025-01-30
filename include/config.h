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
#define PIN_STEPPER_STEP_1 3
#define PIN_STEPPER_DIR_1 25
#define PIN_STEPPER_ENABLE_1 24

#define PIN_STEPPER_STEP_2 2
#define PIN_STEPPER_DIR_2 27
#define PIN_STEPPER_ENABLE_2 28

#define PIN_STEPPER_STEP_3 5
#define PIN_STEPPER_DIR_3 29
#define PIN_STEPPER_ENABLE_3 26

//Pin Actionneur
#define PIN_ACTIONNEUR_1 4
#define PIN_ACTIONNEUR_2 42
#define PIN_ACTIONNEUR_3 43


// DC Motor Pins
#define PIN_MOTEURDC_REVERSE_1 45
#define PIN_MOTEURDC_FORWARD_1 46

#define PIN_MOTEURDC_REVERSE_2 44
#define PIN_MOTEURDC_FORWARD_2 52

// Sensor Pins
#define PIN_SENSOR_1 32
#define PIN_SENSOR_2 34
#define PIN_SENSOR_3 35
#define PIN_SENSOR_4 36
#define PIN_SENSOR_5 37

// LED Pins
#define PIN_LED_1_R 39
#define PIN_LED_1_G 38
#define PIN_LED_1_B 33

// PWM Lidar
#define PIN_PWM_LIDAR 12

//configuration Stepper
#define DEFAULT_MAX_SPEED 5000  // Vitesse maximale (en pas par seconde)
#define DEFAULT_MAX_ACCEL 8000 // Accélération maximale (en pas par seconde carré)