#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"
#include "conversionArray.h"
#include "servoControl.h"

// TODO: move these defines later
#define SERVO_MAX_VALUE 180
#define SERVO_MIN_VALUE 0
#define SERVO_COUNT 7
#define STEPPER_COUNT 3

/* TODO: Remove
AccelStepper stepper1(AccelStepper::DRIVER, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1);
AccelStepper stepper2(AccelStepper::DRIVER, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2);
AccelStepper stepper3(AccelStepper::DRIVER, PIN_STEPPER_STEP_3, PIN_STEPPER_DIR_3);
*/

// idk if this is ok
AccelStepper steppers[STEPPER_COUNT] = { 
  {AccelStepper::DRIVER, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1};
  {AccelStepper::DRIVER, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2};
  {AccelStepper::DRIVER, PIN_STEPPER_STEP_3, PIN_STEPPER_DIR_3};
}

/* TODO: Remove
servoControl servo1;
servoControl servo2(true); // ??
servoControl servo3;
servoControl servo4;
servoControl servo5;
servoControl servo6;
servoControl servo7;
*/

servoControl *servos[SERVO_COUNT];
for(int i=0; i<SERVO_COUNT; i++) {
  servos[i] = new servoControl; 
}


uint8_t onReceiveData[BUFFERONRECEIVESIZE];
uint8_t onRequestData[BUFFERONREQUESTSIZE];
int lenghtOnRequest;

void receiveEvent(int numBytes);
void requestEvent();

void setup() {
  Serial.begin(115200);
  Serial.println("start 2");

  initServo(servos[0], PIN_SERVOMOTEUR_1, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servos[1], PIN_SERVOMOTEUR_2, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servos[2], PIN_SERVOMOTEUR_3, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servos[3], PIN_SERVOMOTEUR_4, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servos[4], PIN_SERVOMOTEUR_5, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servos[5], PIN_SERVOMOTEUR_6, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servos[6], PIN_SERVOMOTEUR_7, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);

  initPin(PIN_STEPPER_SLEEP, false);
  initPin(PIN_STEPPER_RESET, false);
  delay(1); // ??
  initStepper(steppers[0], PIN_STEPPER_ENABLE_1, DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL);
  initStepper(steppers[1], PIN_STEPPER_ENABLE_1, DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL);
  initStepper(steppers[2], PIN_STEPPER_ENABLE_1, DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL);

  initPin(PIN_ACTIONNEUR_1, false);
  initPin(PIN_ACTIONNEUR_2, false);
  initPin(PIN_ACTIONNEUR_3, false);

  initPin(PIN_MOTEURDC_REVERSE_1, true);
  initPin(PIN_MOTEURDC_FORWARD_1, true);
  initPin(PIN_MOTEURDC_REVERSE_2, true);
  initPin(PIN_MOTEURDC_FORWARD_2, true);

  // sensor pins are from 32 to 39 in order, we could make a loop here if the pinmodes were the same 
  // what's up with sensors 4 & 5??
  pinMode(PIN_CAPTEUR_1, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_2, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_3, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_4, OUTPUT);
  pinMode(PIN_CAPTEUR_5, OUTPUT);
  pinMode(PIN_CAPTEUR_6, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_7, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_8, INPUT_PULLUP);
  digitalWrite(PIN_CAPTEUR_4, LOW);
  digitalWrite(PIN_CAPTEUR_5, LOW);

  Wire.begin(100);
  Wire.setTimeout(1000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  // current_time = millis();
}

void loop() {
  for(int servoNb=0; servoNb<SERVO_COUNT; servoNb++) {
    servos[servoNb].run();
  }
  for(int stepperNb=0; stepperNb<STEPPER_COUNT; stepperNb++) {
    steppers[stepperNb].run();
  }
}

void receiveEvent(int numBytes) {
  int i = 0;
  while (Wire.available() > 0) {
    if(i<BUFFERONRECEIVESIZE){
      onReceiveData[i] = Wire.read();
      i++;
    }
    else{
      Wire.read();
    }
   
  }

  // still kinda ugly
  int number;  
  int position = 0; 
  arrayToParameter(onReceiveData,BUFFERONRECEIVESIZE,"1%d",&number);
  arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);

  if(number > STEPPER_COUNT + 10) { // servos can take ids from 1 to 10, and steppers 10 to 20. above we do other stuff
    switch(number) { // old code here still, dunno if it can be simplified
      case 21:
        digitalWrite(PIN_STEPPER_ENABLE_1,LOW);
        break;
  
      case 22:
        digitalWrite(PIN_STEPPER_ENABLE_1,HIGH);
        break;
  
      case 23:
        digitalWrite(PIN_STEPPER_ENABLE_2,LOW);
        break;
  
      case 24:
        digitalWrite(PIN_STEPPER_ENABLE_2,HIGH);
        break;
  
      case 25:
        digitalWrite(PIN_STEPPER_ENABLE_3,LOW);
        break;
  
      case 26:
        digitalWrite(PIN_STEPPER_ENABLE_3,HIGH);
        break;

      case 31:
        digitalWrite(PIN_CAPTEUR_4,HIGH);
        break;

      case 32:
        digitalWrite(PIN_CAPTEUR_4,LOW);
        break;

      case 33:
        digitalWrite(PIN_CAPTEUR_5,HIGH);
        break;

      case 34:
        digitalWrite(PIN_CAPTEUR_5,LOW);
        break;

      default: break;
    }
  }
  else if(number > SERVO_COUNT) 
    steppers[number - 1].write(position);
  else 
    servos[number - 1].write(position);

}

void requestEvent(){ // i have no idea what this really does yet
  switch (onReceiveData[0])
  {
  case 100 :
    parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_1));
    lenghtOnRequest = 2;
    break;

  case 101 :
    parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_2));
    lenghtOnRequest = 2;
    break;

  case 102 :
    parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_3));
    lenghtOnRequest = 2;
    break;

  case 103 :
    //parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_4));
    lenghtOnRequest = 2;
    break;

  case 104 :
    //parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_5));
    lenghtOnRequest = 2;
    break;

  case 105 :
    parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_6));
    lenghtOnRequest = 2;
    break;

  case 106 :
    parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_7));
    lenghtOnRequest = 2;
    break;

  case 107 :
    parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_8));
    lenghtOnRequest = 2;
    break;
  
  default:
    break;
  }
  Wire.write(onRequestData, lenghtOnRequest);
}

void initServo(servoControl servo, int pin, int min, int max, int writeVal) {
  servo.attach(pin);
  servo.setMinMax(min, max);
  servo.write(writeVal); // not sure wat it do (initialize servo position?)
  return;
}

void initStepper(AccelStepper stepper, int pin, int maxSpeed, int accel) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  return;
}

void initPin(int pin, bool low) {
  pinMode(pin, OUTPUT);
  if(low)
    digitalWrite(pin, LOW);
  else
    digitalWrite(pin, HIGH);
  return;
}