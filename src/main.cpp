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
#define STEPPER_COUNT 4
#define SENSOR_COUNT 8

#define CMD_MOVE_SERVO 0x01
#define CMD_READ_SENSOR 0x02
#define CMD_ENABLE_STEPPER 0x03
#define CMD_DISABLE_STEPPER 0x04
#define CMD_RGB_LED 0x05
#define CMD_SET_PWM_LIDAR 0x06
#define CMD_MOVE_STEPPER 0x07
#define CMD_SET_STEPPER 0x08
#define CMD_GET_STEPPER 0x09
#define CMD_SET_MOSFET 0x0A
#define CMD_SET_DCMOTOR 0x0B

RGB_LED led(PIN_LED_1_R, PIN_LED_1_G, PIN_LED_1_B);
const int sensor_pins[SENSOR_COUNT] = {PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5, PIN_SENSOR_6, PIN_SENSOR_7, PIN_SENSOR_8};

AccelStepper steppers[STEPPER_COUNT] = {
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1, PIN_STEPPER_ENABLE_1},
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2, PIN_STEPPER_ENABLE_2},
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_3, PIN_STEPPER_DIR_3, PIN_STEPPER_ENABLE_3},
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_4, PIN_STEPPER_DIR_4, PIN_STEPPER_ENABLE_4},
};

servoControl servos[SERVO_COUNT] = {
    servoControl(),
    servoControl(),
    servoControl(),
    servoControl(),
    servoControl(),
    servoControl(),
    servoControl()};

uint8_t onReceiveData[BUFFERONRECEIVESIZE];
// int onReceiveDataSize = 0;
uint8_t ResponseData[BUFFERONRECEIVESIZE];
int ResponseDataSize = 0;

void receiveEvent(int numBytes);
void requestEvent();
void initServo(servoControl &servo, int pin, int min, int max, int initialPos);
void initStepper(AccelStepper &stepper, int maxSpeed, int Accel, int enablePin);
void initOutPin(int pin, bool low);
void initInPin(int pin);
void setPWM_P44(uint8_t val);
void setPWM_P45(uint8_t val);
void setPWM_P46(uint8_t val);

void setup()
{
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println("Starting !");
#endif

  initServo(servos[0], PIN_SERVOMOTEUR_1, 0, 250, 0);
  initServo(servos[1], PIN_SERVOMOTEUR_2, 20, 140, 140);
  initServo(servos[2], PIN_SERVOMOTEUR_3, 0, 120, 0);
  initServo(servos[3], PIN_SERVOMOTEUR_4, 0, 180, 180);
  initServo(servos[4], PIN_SERVOMOTEUR_5, 0, 250, 0);
  initServo(servos[5], PIN_SERVOMOTEUR_6, 0, 250, 0);
  initServo(servos[6], PIN_SERVOMOTEUR_7, 0, 250, 0);

  initOutPin(PIN_STEPPER_SLEEP, false);
  initOutPin(PIN_STEPPER_RESET, false);
  delay(1);
  initStepper(steppers[0], DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL, PIN_STEPPER_ENABLE_1);
  initStepper(steppers[1], DEFAULT_MAX_SPEED / 8, DEFAULT_MAX_ACCEL / 8, PIN_STEPPER_ENABLE_2);
  initStepper(steppers[2], DEFAULT_MAX_SPEED / 3, DEFAULT_MAX_ACCEL / 3, PIN_STEPPER_ENABLE_3);
  initStepper(steppers[3], DEFAULT_MAX_SPEED / 3, DEFAULT_MAX_ACCEL / 3, PIN_STEPPER_ENABLE_4);

  initOutPin(PIN_ACTIONNEUR_1, false);
  setPWM_P44(0);

  initOutPin(PIN_PWM_LIDAR, true);
  setPWM_P46(0);

  initOutPin(PIN_MOTEURDC_REVERSE_1, true);
  initOutPin(PIN_MOTEURDC_FORWARD_1, true);
  setPWM_P45(0);

  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    initInPin(sensor_pins[i]);
  }

  Wire.begin(100);
  Wire.setTimeout(1000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  // current_time = millis();
}

void loop()
{
  for (int i = 0; i < SERVO_COUNT; i++)
    servos[i].run();
  for (int i = 0; i < STEPPER_COUNT; i++)
    steppers[i].run();
  led.run();
}

void receiveEvent(int numBytes)
{
  if (!Wire.available())
    return;

  Wire.readBytes(onReceiveData, numBytes);
  // onReceiveDataSize += numBytes;

#ifdef SERIAL_DEBUG
  Serial.print("Received: 0x ");
  for (int i = 0; i < numBytes; i++)
  {
    Serial.print(onReceiveData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif

  uint8_t *ptr = onReceiveData;
  uint8_t command = ReadUInt8(&ptr);
  uint8_t number = ReadUInt8(&ptr);

#ifdef SERIAL_DEBUG
  Serial.print("Command: ");
  Serial.println(command, HEX);
  Serial.print("Number: ");
  Serial.println(number);
#endif

  uint8_t *resp_ptr = ResponseData; // + ResponseDataSize; // For requests
  switch (command)
  {
  case CMD_MOVE_SERVO:
    if (number > SERVO_COUNT || number < 1)
      break;
    uint16_t target = ReadUInt16(&ptr);
    uint16_t speed = ReadUInt16(&ptr);
    servos[number - 1].target(target, speed);
    break;
  case CMD_MOVE_STEPPER:
    if (number > STEPPER_COUNT || number < 1)
      break;
    steppers[number - 1].moveTo(ReadInt32(&ptr));
    break;
  case CMD_ENABLE_STEPPER:
    if (number > STEPPER_COUNT || number < 1)
      break;
    steppers[number - 1].enableOutputs();
    break;
  case CMD_DISABLE_STEPPER:
    if (number > STEPPER_COUNT || number < 1)
      break;
    steppers[number - 1].disableOutputs();
    break;
  case CMD_RGB_LED:
    if (number != 1)
      break;
    led.recieveData(ptr);
    break;
  case CMD_SET_PWM_LIDAR:
    setPWM_P46(number);
    break;
  case CMD_SET_STEPPER: // Set the stepper position at the recieved posititon
    if (number > STEPPER_COUNT || number < 1)
      break;
    steppers[number - 1].setCurrentPosition(ReadInt32(&ptr));
    break;
  case CMD_SET_MOSFET: // Set the mosfet pwm to the recieved value
    setPWM_P44(number);
    break;
  case CMD_SET_DCMOTOR:
  { // Set the dc motor bridge to the recieved value and direction
    uint8_t direction = ReadUInt8(&ptr);
    if (direction == 0)
    { // Forward
      setPWM_P45(number);
      analogWrite(PIN_MOTEURDC_REVERSE_1, 0);
    }
    else
    { // Reverse
      setPWM_P45(0);
      analogWrite(PIN_MOTEURDC_REVERSE_1, number);
    }
    break;
  }

  // Request commands
  case CMD_GET_STEPPER:
    if (number > STEPPER_COUNT || number < 1)
      break;
    WriteInt32(&resp_ptr, steppers[number - 1].currentPosition());
#ifdef SERIAL_DEBUG
    // Serial.print("Stepper value is :");
    // Serial.println(steppers[number - 1].currentPosition());
#endif
    break;
  case CMD_READ_SENSOR:
    if (number > SENSOR_COUNT || number < 1)
      break;
    WriteUInt8(&resp_ptr, !digitalRead(sensor_pins[number - 1]));
    break;
  default:
    break;
  }
  // onReceiveDataSize -= ptr - onReceiveData;
  ResponseDataSize += resp_ptr - ResponseData;

  return;
}

void requestEvent()
{
#ifdef SERIAL_DEBUG
  Serial.print("Request ! Sending: 0x ");
  for (int i = 0; i < ResponseDataSize; i++)
  {
    Serial.print(ResponseData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  if (ResponseDataSize == 4)
  {
    Serial.print("In long is :");
    uint8_t *ptr = ResponseData;
    long val = ReadInt32(&ptr);
    Serial.println(val);
  }
#endif

  Wire.write(ResponseData, ResponseDataSize);
  ResponseDataSize = 0;
}

void initServo(servoControl &servo, int pin, int min, int max, int initialPos)
{
  servo.attach(pin);
  servo.setMinMaxValue(min, max);
  servo.target(initialPos, 0);
  return;
}

void initStepper(AccelStepper &stepper, int maxSpeed, int accel, int enablePin)
{
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);
  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(false, false, true);
  stepper.disableOutputs();
  return;
}

void initOutPin(int pin, bool low)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, low ? LOW : HIGH);
  return;
}

void initInPin(int pin)
{
  pinMode(pin, INPUT_PULLUP);
  return;
}

// Configure Timer5 for Fast PWM Mode, Non-inverting, No prescaler (PWM Pin 44, 45 and 46)
void configTMR5()
{
  // TCCR5A = _BV(WGM51) | _BV(WGM50) | _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1);
  // TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS50);
  // ICR5 = 800;  // 20kHz PWM frequency
}

void setPWM_P44(uint8_t val)
{
  if (val == 0)
  {
    OCR5A = 0x0000;
    TCCR5A &= ~_BV(COM5A1);
    digitalWrite(44, LOW); // PIN_ACTIONNEUR_1
  }
  else
  {
    configTMR5();
    TCCR5A |= _BV(COM5A1);
    OCR5A = (uint16_t)(val) * 32 / 10;
  }
}
void setPWM_P45(uint8_t val)
{
  if (val == 0)
  {
    OCR5B = 0x0000;
    TCCR5A &= ~_BV(COM5B1);
    digitalWrite(45, LOW); // PIN_MOTEURDC_REVERSE_1
  }
  else
  {
    configTMR5();
    TCCR5A |= _BV(COM5B1);
    OCR5B = (uint16_t)(val) * 32 / 10;
  }
}
void setPWM_P46(uint8_t val)
{
  digitalWrite(46, val == 0 ? LOW : HIGH);
  /*
  if (val == 0){
    OCR5C = 0x0000;
    TCCR5A &= ~_BV(COM5C1);
    digitalWrite(46, LOW); // PIN_PWM_LIDAR
  }
  else{
    configTMR5();
    TCCR5A |= _BV(COM5C1);
    OCR5C = (uint16_t)(val) * 32 / 10;
  }
  */
}