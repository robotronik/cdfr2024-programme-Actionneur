#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"
#include "conversionArray.h"



AccelStepper stepper1(AccelStepper::DRIVER, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1);
AccelStepper stepper2(AccelStepper::DRIVER, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2);


Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;

uint8_t onRequestData[BUFFERONREQUESTSIZE];

void receiveEvent(int numBytes);
void requestEvent();

void setup() {
  servo1.attach(PIN_SERVOMOTEUR_1);
  servo2.attach(PIN_SERVOMOTEUR_2);
  servo3.attach(PIN_SERVOMOTEUR_3);
  servo4.attach(PIN_SERVOMOTEUR_4);
  servo5.attach(PIN_SERVOMOTEUR_5);
  servo6.attach(PIN_SERVOMOTEUR_6);
  servo7.attach(PIN_SERVOMOTEUR_7);
  servo8.attach(PIN_SERVOMOTEUR_8);
  servo1.write(150);
  servo2.write(0);
  servo3.write(0);
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);
  servo8.write(0);

  stepper1.setMaxSpeed(DEFAULT_MAX_SPEED);
  stepper1.setAcceleration(DEFAULT_MAX_ACCEL);
  stepper1.setMaxSpeed(DEFAULT_MAX_SPEED);
  stepper1.setAcceleration(DEFAULT_MAX_ACCEL);
  pinMode(PIN_STEPPER_ENABLE_1, OUTPUT);
  pinMode(PIN_STEPPER_ENABLE_2, OUTPUT);
  digitalWrite(PIN_STEPPER_ENABLE_1,HIGH);
  digitalWrite(PIN_STEPPER_ENABLE_2,HIGH);

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

  Serial.begin(115200);
  Serial.println("start");
  Wire.begin(100);
  Wire.setTimeout(1000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  stepper1.run();
  stepper2.run();

}


void receiveEvent(int numBytes) {
  uint8_t onReceiveData[BUFFERONRECEIVESIZE];
  int i = 0;
  while (Wire.available() > 0) { // Tant qu'il y a des données disponibles
    if(i<BUFFERONRECEIVESIZE){
      onReceiveData[i] = Wire.read(); // Lecture du caractère reçu
      i++;
    }
    else{
      Wire.read();
    }
   
  }

  int commande;
  arrayToParameter(onReceiveData,BUFFERONREQUESTSIZE,"1%d",&commande);

  switch (commande)
  {
  case 1 :
    int position;
    arrayToParameter(onRequestData+1,BUFFERONREQUESTSIZE,"2%d",&position);
    servo1.write(position);
    break;

  case 2 :
    break;
  
  default:
    break;
  }
}

void requestEvent(){
  Wire.write(onRequestData, BUFFERONREQUESTSIZE);
}