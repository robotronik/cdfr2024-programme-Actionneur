#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"
#include "utils.h"
#include "servoControl.h"
#include "RGB_LED.h"

// Comment this line to disable serial debug
// #define SERIAL_DEBUG

// TODO: move these defines later
#define SERVO_COUNT 7
#define STEPPER_COUNT 3
#define SENSOR_COUNT 5

#define CMD_MOVE_SERVO 0x01
#define CMD_READ_SENSOR 0x02
#define CMD_ENABLE_STEPPER 0x03
#define CMD_DISABLE_STEPPER 0x04
#define CMD_RGB_LED 0x05
#define CMD_SET_PWM_LIDAR 0x06
#define CMD_MOVE_STEPPER 0x07
#define CMD_SET_STEPPER 0x08
#define CMD_GET_STEPPER 0x09

RGB_LED led(PIN_LED_1_R, PIN_LED_1_G, PIN_LED_1_B);
const int sensor_pins[SENSOR_COUNT] = {PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5};

AccelStepper steppers[STEPPER_COUNT] = { 
  {AccelStepper::DRIVER, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1, PIN_STEPPER_ENABLE_1},
  {AccelStepper::DRIVER, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2, PIN_STEPPER_ENABLE_2},
  {AccelStepper::DRIVER, PIN_STEPPER_STEP_3, PIN_STEPPER_DIR_3, PIN_STEPPER_ENABLE_3},
};

servoControl servos[SERVO_COUNT] = {
  servoControl(),
  servoControl(),
  servoControl(),
  servoControl(),
  servoControl(),
  servoControl(),
  servoControl()
};

uint8_t onReceiveData[BUFFERONRECEIVESIZE];
//int onReceiveDataSize = 0;
uint8_t ResponseData[BUFFERONRECEIVESIZE];
int ResponseDataSize = 0;

void receiveEvent(int numBytes);
void requestEvent();
void initServo(servoControl& servo, int pin, int min, int max, int initialPos);
void initStepper(AccelStepper& stepper, int maxSpeed, int Accel);
void initOutPin(int pin, bool low);
void initInPin(int pin);
void setFastPWM(uint8_t val);

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println("Starting !");
#endif

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
  initStepper(steppers[1], DEFAULT_MAX_SPEED/2, DEFAULT_MAX_ACCEL/2);
  initStepper(steppers[2], DEFAULT_MAX_SPEED/4, DEFAULT_MAX_ACCEL/4);

  initOutPin(PIN_ACTIONNEUR_1, false);
  initOutPin(PIN_ACTIONNEUR_2, false);
  initOutPin(PIN_ACTIONNEUR_3, false);

  initOutPin(PIN_PWM_LIDAR, true);
  // Configure Timer1 for Fast PWM Mode, Non-inverting, No prescaler
  TCCR1A = _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = 800;  // Sets PWM frequency to 20kHz
  setFastPWM(0);
  // Also affects pin 10 and 11

  initOutPin(PIN_MOTEURDC_REVERSE_1, true);
  initOutPin(PIN_MOTEURDC_FORWARD_1, true);
  initOutPin(PIN_MOTEURDC_REVERSE_2, true);
  initOutPin(PIN_MOTEURDC_FORWARD_2, true);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    initInPin(sensor_pins[i]);
  }

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
  led.run();
}

void receiveEvent(int numBytes) {
  if (!Wire.available()) return;

  Wire.readBytes(onReceiveData, numBytes);
  //onReceiveDataSize += numBytes;

#ifdef SERIAL_DEBUG
  Serial.print("Received: 0x ");
  for (int i = 0; i < numBytes; i++) {
    Serial.print(onReceiveData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif

  uint8_t* ptr = onReceiveData;
  uint8_t command = ReadUInt8(&ptr);
  uint8_t number = ReadUInt8(&ptr);
  
#ifdef SERIAL_DEBUG
  Serial.print("Command: ");
  Serial.println(command, HEX);
  Serial.print("Number: ");
  Serial.println(number);
#endif

  uint8_t* resp_ptr = ResponseData;// + ResponseDataSize; // For requests
  switch(command) {
    case CMD_MOVE_SERVO:
      if (number > SERVO_COUNT || number < 1) break;
      servos[number - 1].write(ReadUInt8(&ptr));
      break;
    case CMD_MOVE_STEPPER:
      if (number > STEPPER_COUNT || number < 1) break;
      steppers[number - 1].moveTo(ReadInt32(&ptr));
      break;
    case CMD_ENABLE_STEPPER: 
      if (number > STEPPER_COUNT || number < 1) break;
      steppers[number - 1].enableOutputs();
      break;
    case CMD_DISABLE_STEPPER:
      if (number > STEPPER_COUNT || number < 1) break;
      steppers[number - 1].disableOutputs();
      break;
    case CMD_RGB_LED:
      if (number != 1) break;
      led.recieveData(ptr);
      break; 
    case CMD_SET_PWM_LIDAR:
      setFastPWM(number);
      break;
    case CMD_SET_STEPPER: // Set the stepper position at the recieved posititon
      steppers[number - 1].setCurrentPosition(ReadInt32(&ptr));
      break;
      
    // Request commands
    case CMD_GET_STEPPER:
      if (number > STEPPER_COUNT || number < 1) break;
      WriteInt32(&resp_ptr, steppers[number - 1].currentPosition());
#ifdef SERIAL_DEBUG
      Serial.print("Stepper value is :");
      Serial.println(steppers[number - 1].currentPosition());
#endif
      break;
    case CMD_READ_SENSOR:
      if (number > SENSOR_COUNT || number < 1) break;
      WriteUInt8(&resp_ptr, !digitalRead(sensor_pins[number - 1]));
      break;
    default:
      break;      
  }
  //onReceiveDataSize -= ptr - onReceiveData;
  ResponseDataSize += resp_ptr - ResponseData;

  return;
}

void requestEvent() {
#ifdef SERIAL_DEBUG
  Serial.print("Request ! Sending: 0x ");
  for (int i = 0; i < ResponseDataSize; i++) {
    Serial.print(ResponseData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  if (ResponseDataSize == 4){
    Serial.print("In long is :");
    uint8_t* ptr = ResponseData;
    long val = ReadInt32(&ptr);
    Serial.println(val);
  }
#endif

  Wire.write(ResponseData, ResponseDataSize);
  ResponseDataSize = 0;
}

void initServo(servoControl& servo, int pin, int min, int max, int initialPos) {
  servo.attach(pin);
  servo.setMinMaxValue(min, max);
  servo.write(initialPos);
  return;
}

void initStepper(AccelStepper& stepper, int maxSpeed, int accel) {
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

void initInPin(int pin) {
  pinMode(pin, INPUT_PULLUP);
  return;
} 

void setFastPWM(uint8_t val){
  if (val == 0){
    OCR1B = 0x0000;
    TCCR1A &= ~_BV(COM1B1); // Disconnect OC1B (PWM stops)
    digitalWrite(PIN_PWM_LIDAR, LOW);
  }
  else{
    TCCR1A |= _BV(COM1B1); // Re-enable PWM on OC1B
    ICR1 = 800;  // Sets PWM frequency to 20kHz
    OCR1B = (uint16_t)(val) * 3;
  }
}