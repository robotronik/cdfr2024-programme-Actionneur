#include <Arduino.h>
#include <AccelStepper.h>
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

// Set to 1 while validating PCA9685 in standalone I2C master mode.
#define PCA9685_MASTER_TEST_MODE 1
#define SERVO_SWEEP_TEST_MODE 1

// TODO: move these defines later
#define SERVO_COUNT 16
#define STEPPER_COUNT 4
#define SENSOR_COUNT 7

RGB_LED led(PIN_LED_1_R, PIN_LED_1_G, PIN_LED_1_B);
const int sensor_pins[SENSOR_COUNT] = {PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5, PIN_SENSOR_6, PIN_SENSOR_7};

AccelStepper steppers[STEPPER_COUNT] = {
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1, PIN_STEPPER_ENABLE_1},
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2, PIN_STEPPER_ENABLE_2},
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_3, PIN_STEPPER_DIR_3, PIN_STEPPER_ENABLE_3},
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_4, PIN_STEPPER_DIR_4, PIN_STEPPER_ENABLE_4},
};

servoControl servos[SERVO_COUNT];

MotorDC motorDC(PIN_MOTEURDC_FORWARD_1, PIN_MOTEURDC_REVERSE_1);

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
bool pca9685WriteRegister(uint8_t reg, uint8_t value);
bool pca9685WriteChannelRaw(uint8_t channel, uint16_t on_ticks, uint16_t off_ticks);
void runPCA9685MasterModeTest();
void runServoSweepTest();

void setup()
{
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println("Starting !");
#endif

  // Temporarily repurpose the firmware to I2C master mode for PCA9685 bring-up.
  runPCA9685MasterModeTest();
  servoControl::beginPCA9685(PCA9685_I2C_ADDRESS, PCA9685_PWM_FREQUENCY);

  for (int i = 0; i < SERVO_COUNT; i++)
  {
    initServo(servos[i], i, 0, 180, 90);
  }

  //Pompe
  initOutPin(PIN_SENSOR_8, false); // false → HIGH → pompe OFF

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

#if !PCA9685_MASTER_TEST_MODE
  Wire.begin(I2C_ADDRESS);
  Wire.setTimeout(1000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
#endif
}

void loop()
{
#if SERVO_SWEEP_TEST_MODE
  runServoSweepTest();
#endif

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
    break;
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
    uint8_t is_forward = ReadUInt8(&ptr);
    motorDC.moveSpeed(speed, is_forward);
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
  case CMD_WRITE_PIN:{
      if (number != 1) break;
      uint8_t state = ReadUInt8(&ptr);
  
      if (state) digitalWrite(PIN_SENSOR_8, LOW);   // ON
      else digitalWrite(PIN_SENSOR_8, HIGH);  // OFF
      break;
    }
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

void initServo(servoControl &servo, int pin, int min, int max, int initialPos)
{
  servo.attach(pin, min, max);
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

bool pca9685WriteRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool pca9685WriteChannelRaw(uint8_t channel, uint16_t on_ticks, uint16_t off_ticks)
{
  if (channel > 15)
    return false;

  const uint8_t base = 0x06 + 4 * channel;
  Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  Wire.write(base);
  Wire.write(on_ticks & 0xFF);
  Wire.write((on_ticks >> 8) & 0x0F);
  Wire.write(off_ticks & 0xFF);
  Wire.write((off_ticks >> 8) & 0x0F);
  return Wire.endTransmission() == 0;
}

void runPCA9685MasterModeTest()
{
  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(1000);

  // Basic bring-up instructions to verify write access to the PCA9685.
  pca9685WriteRegister(0x00, 0x00); // MODE1: normal mode, internal oscillator
  pca9685WriteRegister(0x01, 0x04); // MODE2: totem-pole output driver
  pca9685WriteChannelRaw(PCA9685_SERVO_CHANNEL_1, 0, 307); // ~1.5ms pulse at 50Hz
}

void runServoSweepTest()
{
  static bool initialized = false;
  static bool go_right = true;
  static unsigned long last_toggle_ms = 0;
  const unsigned long interval_ms = 1500;

  if (!initialized)
  {
    servos[0].target(0, 0);
    servos[1].target(180, 0);
    initialized = true;
    last_toggle_ms = millis();
    return;
  }

  if (millis() - last_toggle_ms < interval_ms)
    return;

  if (go_right)
  {
    servos[0].target(180, 300);
    servos[1].target(0, 180);
  }
  else
  {
    servos[0].target(0, 180);
    servos[1].target(180, 300);
  }

  go_right = !go_right;
  last_toggle_ms = millis();
}
