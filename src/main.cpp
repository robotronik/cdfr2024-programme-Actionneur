#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"
#include "conversionArray.h"
#include "servoControl.h"


AccelStepper stepper1(AccelStepper::DRIVER, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1);
AccelStepper stepper2(AccelStepper::DRIVER, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2);
AccelStepper stepper3(AccelStepper::DRIVER, PIN_STEPPER_STEP_3, PIN_STEPPER_DIR_3);


servoControl servo1;
servoControl servo2;
servoControl servo3;
servoControl servo4;
servoControl servo5;
servoControl servo6;
servoControl servo7;
servoControl servo8;

uint8_t onReceiveData[BUFFERONRECEIVESIZE];
uint8_t onRequestData[BUFFERONREQUESTSIZE];
int lenghtOnRequest;

void receiveEvent(int numBytes);
void requestEvent();

void setup() {
  Serial.begin(115200);
  Serial.println("start 2");

  servo1.attach(PIN_SERVOMOTEUR_1);
  servo2.attach(PIN_SERVOMOTEUR_2);
  servo3.attach(PIN_SERVOMOTEUR_3);
  servo4.attach(PIN_SERVOMOTEUR_4);
  servo5.attach(PIN_SERVOMOTEUR_5);
  servo6.attach(PIN_SERVOMOTEUR_6);
  servo7.attach(PIN_SERVOMOTEUR_7);
  servo8.attach(PIN_SERVOMOTEUR_8);
  servo1.setMinMaxValue(80,160);
  servo2.setMinMaxValue(18,35);
  servo3.setMinMaxValue(0,180);
  servo4.setMinMaxValue(0,180);
  servo5.setMinMaxValue(0,180);
  servo6.setMinMaxValue(0,180);
  servo7.setMinMaxValue(0,180);
  servo8.setMinMaxValue(0,180);
  servo1.write(160);
  servo2.write(18);
  servo3.write(0);
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);
  servo8.write(0);
  // setup vitesse max and acceleration max
  //servo1.setParamater(120,60,60,120,60,60);
  //servo1.setParamater(-1,-1,-1,-1,-1,-1);
  pinMode(PIN_STEPPER_SLEEP, OUTPUT);
  pinMode(PIN_STEPPER_RESET, OUTPUT);
  digitalWrite(PIN_STEPPER_SLEEP, HIGH);
  digitalWrite(PIN_STEPPER_RESET, HIGH);
  delay(1);

  stepper1.setMaxSpeed(DEFAULT_MAX_SPEED);
  stepper1.setAcceleration(DEFAULT_MAX_ACCEL);
  stepper2.setMaxSpeed(DEFAULT_MAX_SPEED);
  stepper2.setAcceleration(DEFAULT_MAX_ACCEL);
  stepper3.setMaxSpeed(DEFAULT_MAX_SPEED);
  stepper3.setAcceleration(DEFAULT_MAX_ACCEL);
  pinMode(PIN_STEPPER_ENABLE_1, OUTPUT);
  pinMode(PIN_STEPPER_ENABLE_2, OUTPUT);
  pinMode(PIN_STEPPER_ENABLE_3, OUTPUT);
  digitalWrite(PIN_STEPPER_ENABLE_1,LOW);
  digitalWrite(PIN_STEPPER_ENABLE_2,HIGH);
  digitalWrite(PIN_STEPPER_ENABLE_3,HIGH);

  pinMode(PIN_ACTIONNEUR_1, OUTPUT);
  pinMode(PIN_ACTIONNEUR_2, OUTPUT);
  pinMode(PIN_ACTIONNEUR_3, OUTPUT);
  digitalWrite(PIN_ACTIONNEUR_1,LOW);
  digitalWrite(PIN_ACTIONNEUR_2,LOW);
  digitalWrite(PIN_ACTIONNEUR_3,LOW);

  pinMode(PIN_MOTEURDC_REVERSE_1, OUTPUT);
  pinMode(PIN_MOTEURDC_FORWARD_1, OUTPUT);
  digitalWrite(PIN_MOTEURDC_REVERSE_1,LOW);
  digitalWrite(PIN_MOTEURDC_FORWARD_1,LOW);
  pinMode(PIN_MOTEURDC_REVERSE_2, OUTPUT);
  pinMode(PIN_MOTEURDC_FORWARD_2, OUTPUT);
  digitalWrite(PIN_MOTEURDC_REVERSE_2,HIGH);
  digitalWrite(PIN_MOTEURDC_FORWARD_2,HIGH);

  pinMode(PIN_CAPTEUR_1, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_2, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_3, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_4, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_5, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_6, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_7, INPUT_PULLUP);
  pinMode(PIN_CAPTEUR_8, INPUT_PULLUP);

  Wire.begin(100);
  Wire.setTimeout(1000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
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
  servo8.run();
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

  case 8 :{
    int position = 0; 
    arrayToParameter(onReceiveData+1,BUFFERONRECEIVESIZE,"2%d",&position);
    servo8.write(position);
    break;
  }

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
    parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_4));
    lenghtOnRequest = 2;
    break;

  case 104 :
    parameterToArray(onRequestData,BUFFERONREQUESTSIZE,"2%d",!digitalRead(PIN_CAPTEUR_5));
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