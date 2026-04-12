#pragma once

#include "Arduino.h"

#define BUFFERONRECEIVESIZE 32

// Pin that enables the 5V power supply for the servos
#define PIN_SERVOS_POWER 14

// PCA9685 configuration
#define PCA9685_I2C_ADDRESS 0x40
#define PCA9685_PWM_FREQUENCY 50
#define PCA9685_SERVO_CHANNEL_1 0
#define PCA9685_SERVO_CHANNEL_2 1
#define PCA9685_SERVO_CHANNEL_3 2
#define PCA9685_SERVO_CHANNEL_4 3
#define PCA9685_SERVO_CHANNEL_5 4
#define PCA9685_SERVO_CHANNEL_6 5
#define PCA9685_SERVO_CHANNEL_7 6

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

// DC Motor Pins
#define PIN_MOTEURDC_REVERSE_1 45
#define PIN_MOTEURDC_FORWARD_1 44

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
#define PIN_LED_1_B 66 //A12
#define PIN_LED_1_G 65 //A11
#define PIN_LED_1_R 64 //A10

// PWM Lidar
#define PIN_PWM_LIDAR 46

//configuration Stepper
#define DEFAULT_MAX_SPEED 5000  // Vitesse maximale (en pas par seconde)
#define DEFAULT_MAX_ACCEL 8000 // Accélération maximale (en pas par seconde carré)