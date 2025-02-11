# Festoon Machine Control System

## Overview
This project is a C++ implementation to control a Festoon Machine, which manages the placement of fiber into a container. The system coordinates three motors to move the container in the X-axis, control the fiber's speed through a pinch roller, and adjust the ramp for Y-axis positioning. It uses ClearPath motors from Teknic Inc. for precision control.

## Objective
The main goal of this project is to automate and precisely control the placement of fiber using the following components:
- **X-axis Motor:** Moves the container.
- **Y-axis Motor:** Adjusts the ramp under the pinch roller.
- **Pinch Roller Motor:** Controls the fiber's speed.

---

## Hardware Requirements
1. **ClearPath Motors**:
   - X-axis and Y-axis motors configured for **16 Positions (Home to Hard Stop)** mode.
   - Pinch Roller motor configured for **Manual Velocity Control** mode.
2. **Inputs**:
   - Start Button
   - Stop Button
   - Quadrature Encoder (16 pulses per revolution) for speed control.
3. **Outputs**:
   - Status Indicator Light (indicates when the machine is running)
   - Motor Enable/Disable Controls

---

## Software Requirements
- **ClearCore Library** (Teknic)
- **MSP Software** for ClearPath motor configuration.
- **Arduino IDE or other C++ development environment**.

---

## Motor Configuration
### X-axis and Y-axis Motors
Configured to operate in **Move to Absolute Position (16 Positions)** mode:
1. Define 16 absolute positions using the MSP software.
2. Set the Homing and High-Level Feedback (HLFB) mode to "ASG-Position with Measured Torque" at a PWM frequency of 482 Hz.

### Pinch Roller Motor
Configured for **Manual Velocity Control**:
1. Set maximum clockwise and counter-clockwise velocities.
2. Use a rotary encoder to control the speed.
3. HLFB mode set to "ASG-Velocity with Measured Torque" at a PWM frequency of 482 Hz.

---

## System Logic
### Initialization (`setup`)
- Configures motor modes and pins.
- Waits for the **Start Button** to enable the motors and initiate homing.
- Once homing completes, enables all motors and the status indicator light.

### Main Loop (`loop`)
- Continuously monitors the **Start** and **Stop** buttons to enable or disable motors.
- Uses the rotary encoder to adjust the speed of the ramp motor.
- Executes predefined movement patterns between two positions on the X-axis motor.

### Helper Functions
1. **`MoveToPositionM1(int positionNum)`**: Moves the X-axis motor to a specified absolute position and waits for confirmation from HLFB.
2. **`checkButtonStatus()`**: Monitors the Start and Stop buttons to enable/disable motors.
3. **`checkEncoder()`**: Reads the rotary encoder for speed adjustment and displays the direction and pulse count via serial output.
4. **`changeRampMotor2Speed()`**: Adjusts the Y-axis ramp motor speed based on the rotary encoder input.

---

## Wiring and Pin Setup
| Pin | Description               |
|-----|---------------------------|
| DI6 | Start Button              |
| DI7 | Stop Button               |
| 4   | Encoder A (Pinch Roller)  |
| 5   | Encoder B (Pinch Roller)  |
| IO1 | Status Indicator Light     |
| IO2 | Pinch Roller Motor Enable |

---

## Usage Instructions
1. Connect and configure the motors using the **MSP software** according to the specified modes.
2. Compile and upload the code to the ClearCore controller.
3. Press the **Start Button** to enable the motors and begin operation.
4. Use the **rotary encoder** to control the speed of the ramp motor.
5. Press the **Stop Button** to disable the motors and stop the system.

---

## Links and Documentation
- [ClearCore Documentation](https://teknic-inc.github.io/ClearCore-library/)
- [ClearCore User Manual](https://www.teknic.com/files/downloads/clearcore_user_manual.pdf)
- [ClearPath User Manual (DC Power)](https://www.teknic.com/files/downloads/clearpath_user_manual.pdf)
- [ClearPath User Manual (AC Power)](https://www.teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf)
