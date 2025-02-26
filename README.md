# Robotic System Controller

This project is an Arduino-based robotic system controller designed to manage multiple servos, stepper motors, LEDs, and sensors. It communicates using the I2C protocol and provides a variety of commands to control and query the system.

## Features

- **Servo Motor Control**: Supports up to 7 servo motors, allowing position adjustments with defined ranges.
- **Stepper Motor Control**: Supports up to 4 stepper motors, including position setting, movement, and enabling/disabling outputs.
- **RGB LED Control**: Supports an RGB LED with dimming.
- **Sensor Monitoring**: Reads the state of up to 8 digital sensors.
- **I2C Communication**: Listens for commands and sends responses over the I2C bus.
- **Debugging**: Optional serial debugging for monitoring system activity.

## Components Used

- **Libraries**:
  - `Arduino.h`: Core Arduino functionality.
  - `AccelStepper`: For stepper motor control.
  - `Servo`: For servo motor control.
  - `Wire`: For I2C communication.

- **Hardware**:
  - 7 servo motors
  - 4 stepper motors (driver mode)
  - 1 RGB LED
  - 8 digital sensors

## Commands

The controller supports the following commands:

| Command               | Code  | Description                                   |
|-----------------------|-------|-----------------------------------------------|
| `CMD_MOVE_SERVO`      | 0x01  | Move a specific servo motor to a position.   |
| `CMD_READ_SENSOR`     | 0x02  | Read the state of a specified sensor.        |
| `CMD_ENABLE_STEPPER`  | 0x03  | Enable outputs for a specific stepper motor. |
| `CMD_DISABLE_STEPPER` | 0x04  | Disable outputs for a specific stepper motor.|
| `CMD_RGB_LED`         | 0x05  | Controls the RGB LED                         |
| `CMD_MOVE_STEPPER`    | 0x07  | Move a specific stepper motor to a position. |
| `CMD_SET_STEPPER`     | 0x08  | Set a stepper motor's current position.      |
| `CMD_GET_STEPPER`     | 0x09  | Get the current position of a stepper motor. |

### Message Structure

Commands are sent to the controller in the following format:

| Byte Index | Description                          |
|------------|--------------------------------------|
| 0          | Command Code                         |
| 1          | Target Number (Servo, Stepper, etc.) |
| 2+         | Additional Data (if applicable)      |

## How It Works

1. **Initialization**: All servos, stepper motors, LEDs, and sensors are initialized during setup.
2. **Event Handling**:
   - Commands are received via I2C (`receiveEvent`).
   - Responses are sent back via I2C (`requestEvent`).
3. **Main Loop**:
   - Continuously updates the state of servos and stepper motors.

## Setup Instructions

1. **Hardware**:
   - Connect servos, stepper motors, LEDs, and sensors to the appropriate pins defined in `config.h`.

2. **Software**:
   - Have PlatformIO installed

3. **Testing**:
   - Enable serial debugging by defining `SERIAL_DEBUG` in the code.
   - Monitor communication and system status using a serial monitor at 115200 bauds.

## I2C Communication

The controller uses the I2C protocol for communication. Commands are sent as bytes to control the system or request data. Ensure the I2C master device (e.g., Raspberry Pi) sends correctly formatted messages.

## Debugging

When `SERIAL_DEBUG` is enabled, the system outputs detailed logs via the serial interface at 115200 bauds, providing insights into received commands, current states, and responses.

## Future Improvements

- Implement error handling for invalid commands.
- Expand sensor support for analog readings.
