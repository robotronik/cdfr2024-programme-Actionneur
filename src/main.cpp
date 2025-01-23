#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"
#include "utils.h"
#include "servoControl.h"

// TODO: move these defines later
#define SERVO_COUNT 7
#define STEPPER_COUNT 3

#define CMD_MOVE_SERVO 0x01
#define CMD_READ_SENSOR 0x02
#define CMD_ENABLE_STEPPER 0x03
#define CMD_DISABLE_STEPPER 0x04
#define CMD_LED_ON 0x05
#define CMD_LED_OFF 0x06
#define CMD_MOVE_STEPPER 0x07
#define CMD_SET_STEPPER 0x08
#define CMD_GET_STEPPER 0x09

AccelStepper steppers[STEPPER_COUNT] = { 
  {AccelStepper::DRIVER, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1, PIN_STEPPER_ENABLE_1},
  {AccelStepper::DRIVER, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2, PIN_STEPPER_ENABLE_2},
  {AccelStepper::DRIVER, PIN_STEPPER_STEP_3, PIN_STEPPER_DIR_3, PIN_STEPPER_ENABLE_3},
};

servoControl servos[SERVO_COUNT];

uint8_t onReceiveData[BUFFERONRECEIVESIZE];
uint8_t onRequestData[BUFFERONREQUESTSIZE];

void receiveEvent(int numBytes);
void requestEvent();
void initServo(servoControl servo, int pin, int min, int max, int initialPos);
void initStepper(AccelStepper stepper, int maxSpeed, int Accel);
void initOutPin(int pin, bool low);

void setup() {
  Serial.begin(115200);
  Serial.println("start 2");

  for(int i=0; i<SERVO_COUNT; i++) {
    servos[i] = new servoControl; 
  }

  initServo(servos[0], PIN_SERVOMOTEUR_1, 0, 180, 0);
  initServo(servos[1], PIN_SERVOMOTEUR_2, 0, 180, 0);
  initServo(servos[2], PIN_SERVOMOTEUR_3, 0, 180, 0);
  initServo(servos[3], PIN_SERVOMOTEUR_4, 0, 180, 0);
  initServo(servos[4], PIN_SERVOMOTEUR_5, 0, 180, 0);
  initServo(servos[5], PIN_SERVOMOTEUR_6, 0, 180, 0);
  initServo(servos[6], PIN_SERVOMOTEUR_7, 0, 180, 0);

  initOutPin(PIN_STEPPER_SLEEP, false);
  initOutPin(PIN_STEPPER_RESET, false);
  delay(1); 
  initStepper(steppers[0], DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL);
  initStepper(steppers[1], DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL);
  initStepper(steppers[2], DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL);

  initOutPin(PIN_ACTIONNEUR_1, false);
  initOutPin(PIN_ACTIONNEUR_2, false);
  initOutPin(PIN_ACTIONNEUR_3, false);

  initOutPin(PIN_MOTEURDC_REVERSE_1, true);
  initOutPin(PIN_MOTEURDC_FORWARD_1, true);
  initOutPin(PIN_MOTEURDC_REVERSE_2, true);
  initOutPin(PIN_MOTEURDC_FORWARD_2, true);

  pinMode(PIN_SENSOR_1, INPUT_PULLUP);
  pinMode(PIN_SENSOR_2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_3, INPUT_PULLUP);
  pinMode(PIN_SENSOR_4, INPUT_PULLUP);
  pinMode(PIN_SENSOR_5, INPUT_PULLUP);
  pinMode(PIN_SENSOR_6, INPUT_PULLUP);

  initOutPin(PIN_LED_1, true); 
  initOutPin(PIN_LED_2, true);

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
    if(i<BUFFERONRECEIVESIZE) {
      onReceiveData[i] = Wire.read();
      i++;
    }
    else {
      Wire.read();
    }
  }

  uint8_t* ptr = onReceiveData;
  uint8_t command = ReadInt8(&ptr);
  uint8_t number = ReadInt8(&ptr);
  switch(command) {
    case CMD_MOVE_SERVO:
      servos[number - 1].write(ReadInt8(&ptr));
      break;
    case CMD_MOVE_STEPPER:
      steppers[number - 1].moveTo(ReadInt32(&ptr));
      break;
    case CMD_ENABLE_STEPPER: 
      steppers[number - 1].enableOutputs();
      break;
    case CMD_DISABLE_STEPPER:
      steppers[number - 1].disableOutputs();
      break;
    case CMD_LED_ON:
      switch(number) {
        case 1:  
          digitalWrite(PIN_LED_1, HIGH);
          break;
        case 2:
          digitalWrite(PIN_LED_2, HIGH);
          break;
        default:
          break;
      } 
    case CMD_LED_OFF:
      switch(number) {
        case 1:
          digitalWrite(PIN_LED_1, LOW);
          break;
        case 2:
          digitalWrite(PIN_LED_2, LOW);
          break;
        default:
          break;
      }
      break;
    case CMD_SET_STEPPER: // sets current position as the new zero
      steppers[number - 1].setCurrentPosition(steppers[number - 1].currentPosition());
      break;
    default:
      break;      
  }

  return;
}

void requestEvent() { // used to read sensor data 
  uint8_t* ptr_receive = onReceiveData;
  uint8_t* ptr_request = onRequestData;
  int8_t command = ReadInt8(&ptr_receive);
  uint8_t number = ReadInt8(&ptr_receive);
  size_t byte_amount;

  switch (command)
  {
    case CMD_GET_STEPPER:
      WriteInt32(&ptr_request, steppers[number - 1].currentPosition());
      byte_amount = 4;
      break;
    case CMD_READ_SENSOR:
      byte_amount = 1;
      switch(number) {
        case 1:
          WriteInt8(&ptr_request, digitalRead(PIN_SENSOR_1));
          break;
        case 2:
          WriteInt8(&ptr_request, digitalRead(PIN_SENSOR_2));
          break;
        case 3:
          WriteInt8(&ptr_request, digitalRead(PIN_SENSOR_3));
          break;
        case 4:
          WriteInt8(&ptr_request, digitalRead(PIN_SENSOR_4));
          break;
        case 5:
          WriteInt8(&ptr_request, digitalRead(PIN_SENSOR_5));
          break;
        case 6:
          WriteInt8(&ptr_request, digitalRead(PIN_SENSOR_6));
          break;
        case 7:
          WriteInt8(&ptr_request, digitalRead(PIN_LED_1));
          break;
        case 8:
          WriteInt8(&ptr_request, digitalRead(PIN_LED_2));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  Wire.write(onRequestData, byte_amount);
}

void initServo(servoControl servo, int pin, int min, int max, int initialPos) {
  servo.attach(pin);
  servo.setMinMaxValue(min, max);
  servo.write(initialPos);
  return;
}

void initStepper(AccelStepper stepper, int maxSpeed, int accel) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);
  stepper.disableOutputs();
  return;
}

void initOutPin(int pin, bool low) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, low ? LOW : HIGH);
  return;
}