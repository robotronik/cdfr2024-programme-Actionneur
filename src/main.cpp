#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"
#include "utils.h"
#include "servoControl.h"
#include "RGB_LED.h"
#include "MotorDC.h"
#include "timer5PWM.h"
#include "common/protocol.h"

// Comment this line to disable serial debug
// #define SERIAL_DEBUG

// TODO: move these defines later
#define SERVO_COUNT 7
#define STEPPER_COUNT 4
#define SENSOR_COUNT 8

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

MotorDC motorDC(PIN_MOTEURDC_FORWARD_1, PIN_MOTEURDC_REVERSE_1, PIN_SENSOR_8, false, 200);

uint8_t onReceiveData[BUFFERONRECEIVESIZE];
// int onReceiveDataSize = 0;
uint8_t ResponseData[BUFFERONRECEIVESIZE];
int ResponseDataSize = 0;

void receiveEvent(int numBytes);
void requestEvent();
void initServo(servoControl &servo, int ID, int pin, int min, int max, int initialPos);
void initStepper(AccelStepper &stepper, int maxSpeed, int Accel, int enablePin);
void initOutPin(int pin, bool low);
void initInPin(int pin);

void setup()
{
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println("Starting !");
#endif

  initServo(servos[0], 1, PIN_SERVOMOTEUR_1, 0, 180, 130);
  initServo(servos[1], 2, PIN_SERVOMOTEUR_2, 0, 180, 180);
  initServo(servos[2], 3, PIN_SERVOMOTEUR_3, 0, 180, 0);
  initServo(servos[3], 4, PIN_SERVOMOTEUR_4, 0, 180, 90);
  initServo(servos[4], 5, PIN_SERVOMOTEUR_5, 0, 270, 0);
  initServo(servos[5], 6, PIN_SERVOMOTEUR_6, 0, 270, 0);
  initServo(servos[6], 7, PIN_SERVOMOTEUR_7, 0, 270, 0);

  initOutPin(PIN_STEPPER_SLEEP, false);
  initOutPin(PIN_STEPPER_RESET, false);
  initOutPin(PIN_SERVOS_POWER, true);
  delay(1);
  initStepper(steppers[0], DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL, PIN_STEPPER_ENABLE_1);
  initStepper(steppers[1], DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL, PIN_STEPPER_ENABLE_2);
  initStepper(steppers[2], DEFAULT_MAX_SPEED / 3, DEFAULT_MAX_ACCEL / 3, PIN_STEPPER_ENABLE_3);
  initStepper(steppers[3], DEFAULT_MAX_SPEED / 3, DEFAULT_MAX_ACCEL / 3, PIN_STEPPER_ENABLE_4);

  configTMR5();

  setPWM_P44(0); // PIN_MOTEURDC_FORWARD_1
  setPWM_P45(0); // PIN_MOTEURDC_REVERSE_1
  setPWM_P46(0); // PIN_PWM_LIDAR

  // Enable non-inverting PWM for all channels
  TCCR5A |= (1 << COM5A1) | (1 << COM5B1) | (1 << COM5C1);


  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    initInPin(sensor_pins[i]);
  }

  Wire.begin(I2C_ADDRESS);
  Wire.setTimeout(1000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop()
{
  for (int i = 0; i < SERVO_COUNT; i++)
    servos[i].run();
  for (int i = 0; i < STEPPER_COUNT; i++)
    steppers[i].run();
  led.run();
  motorDC.run();
}

void receiveEvent(int numBytes)
{
  if (!Wire.available())
    return;

  Wire.readBytes(onReceiveData, numBytes);
  // onReceiveDataSize += numBytes;

#ifdef SERIAL_DEBUG
/*
  Serial.print("Received: 0x ");
  for (int i = 0; i < numBytes; i++)
  {
    Serial.print(onReceiveData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  */
#endif

  uint8_t *ptr = onReceiveData;
  uint8_t command = ReadUInt8(&ptr);
  uint8_t number = ReadUInt8(&ptr);

#ifdef SERIAL_DEBUG
/*
  Serial.print("Command: ");
  Serial.println(command, HEX);
  Serial.print("Number: ");
  Serial.println(number);
  */
#endif

  uint8_t *resp_ptr = ResponseData; // + ResponseDataSize; // For requests
  switch (command)
  {
  case CMD_POWER_SERVOS:
  {
    bool power = number == 1 ? true : false;
    digitalWrite(PIN_SERVOS_POWER, power);
  }
  case CMD_MOVE_SERVO:
  {
    if (number > SERVO_COUNT || number < 1)
      break;
    uint16_t s_target = ReadUInt16(&ptr);
    uint16_t s_speed = ReadUInt16(&ptr);
    servos[number - 1].target(s_target, s_speed);
    break;
  }
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
  case CMD_SET_STEPPER_SPEED: // Set the stepper speed at the recieved posititon
  {
    if (number > STEPPER_COUNT || number < 1)
      break;
    int32_t speed = ReadInt32(&ptr);
    if (speed > 0)
      steppers[number - 1].setMaxSpeed(speed);
    else
      steppers[number - 1].setMaxSpeed(DEFAULT_MAX_SPEED);
  }
  case CMD_MOVE_DC_MOTOR:
  {
    if (number != 1)
      break;
    uint8_t speed = ReadUInt8(&ptr);
    uint8_t holdSpeed = ReadUInt8(&ptr);
    motorDC.moveToLimit(speed, holdSpeed);
    break;
  }
  case CMD_STOP_DC_MOTOR:
    if (number != 1)
      break;
    motorDC.stop();
    break;

  // Request commands
  case CMD_GET_VERSION:
    WriteUInt8(&resp_ptr, API_VERSION);
  break;
  case CMD_GET_SERVO:
    if (number > SERVO_COUNT || number < 1)
      break;
    WriteInt16(&resp_ptr, servos[number - 1].current_angle);
    break;
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
/*
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
    */
#endif

  Wire.write(ResponseData, ResponseDataSize);
  ResponseDataSize = 0;
}

void initServo(servoControl &servo, int ID, int pin, int min, int max, int initialPos)
{
  servo.attach(pin, min, max, ID);
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