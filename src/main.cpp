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

AccelStepper stepper1(AccelStepper::DRIVER, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1);
AccelStepper stepper2(AccelStepper::DRIVER, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2);
AccelStepper stepper3(AccelStepper::DRIVER, PIN_STEPPER_STEP_3, PIN_STEPPER_DIR_3);

servoControl servo1;
servoControl servo2(true); // ??
servoControl servo3;
servoControl servo4;
servoControl servo5;
servoControl servo6;
servoControl servo7;

uint8_t onReceiveData[BUFFERONRECEIVESIZE];
uint8_t onRequestData[BUFFERONREQUESTSIZE];
int lenghtOnRequest;

void receiveEvent(int numBytes);
void requestEvent();

void setup() {
  Serial.begin(115200);
  Serial.println("start 2");

  initServo(servo1, PIN_SERVOMOTEUR_1, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servo2, PIN_SERVOMOTEUR_2, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servo3, PIN_SERVOMOTEUR_3, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servo4, PIN_SERVOMOTEUR_4, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servo4, PIN_SERVOMOTEUR_5, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servo4, PIN_SERVOMOTEUR_6, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);
  initServo(servo4, PIN_SERVOMOTEUR_7, SERVO_MIN_VALUE, SERVO_MAX_VALUE, 0);

  initPin(PIN_STEPPER_SLEEP, false);
  initPin(PIN_STEPPER_RESET, false);
  delay(1); // ??
  initStepper(stepper1, PIN_STEPPER_ENABLE_1, DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL);
  initStepper(stepper2, PIN_STEPPER_ENABLE_1, DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL);
  initStepper(stepper3, PIN_STEPPER_ENABLE_1, DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL);

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
  stepper1.run();
  stepper2.run();
  stepper3.run();
  servo1.run();
  servo2.run();
  servo3.run();
  servo4.run();
  servo5.run();
  servo6.run();
  servo7.run();
  // loop_test();
  // servo8.run();
  //delay(100);
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

  int commande;
  arrayToParameter(onReceiveData,BUFFERONRECEIVESIZE,"1%d",&commande);

  switch (commande)
  {
  case 1 :{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    servo1.write(position);
    break;
  }

  case 2 :{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    servo2.write(position);
    break;
  }

  case 3 :{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    servo3.write(position);
    break;
  }

  case 4 :{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    servo4.write(position);
    break;
  }

  case 5 :{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    servo5.write(position);
    break;
  }

  case 6 :{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    servo6.write(position);
    break;
  }

  case 7 :{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    servo7.write(position);
    break;
  }

  // case 8 :{
  //   int position = 0; 
  //   arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
  //   servo8.write(position);
  //   break;
  // }

  case 11:{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    stepper1.moveTo(position);
    break;
  }

  case 12:{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    stepper2.moveTo(position);
    break;
  }

  case 13:{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    stepper3.moveTo(position);
    break;
  }
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

  default:
    break;
  }
}

void requestEvent(){
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

  case 109:
    parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_PLANTE));
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