# Hardware Setup Guide

This guide will walk you through the process of assembling your 4-wheel robot line follower.

## Components List

Before starting, make sure you have all the necessary components:

- 1 × Arduino UNO or compatible board
- 1 × L293D or L298N motor driver
- 4 × DC motors (3-6V)
- 4 × Wheels compatible with your motors
- 2 × IR sensors for line detection
- 1 × HC-SR04 ultrasonic sensor
- 1 × 9V battery (or other suitable power source)
- 1 × Battery connector
- 1 × Robot chassis
- Jumper wires (various lengths)
- 1 × Small breadboard or PCB
- Optional: Switch for power control
- Optional: LEDs for status indication

## Assembly Steps

### 1. Prepare the Chassis

1. Mount the 4 DC motors to the chassis
2. Attach the wheels to the motors
3. Ensure the chassis has enough space for the Arduino, motor driver, and sensors

### 2. Mount the Electronics

1. Secure the Arduino UNO to the chassis
2. Mount the L293D/L298N motor driver near the Arduino
3. Attach the small breadboard if needed for connections
4. Mount the battery holder in a position that balances the robot's weight

### 3. Position the Sensors

1. Mount the 2 IR sensors at the front of the robot, pointing downward
   - Position them about 2-3 cm above the ground
   - Space them approximately 3-4 cm apart
   - They should be slightly ahead of the front wheels

2. Mount the HC-SR04 ultrasonic sensor at the front of the robot, pointing forward
   - Position it to have a clear view ahead without obstruction
   - Secure it firmly to prevent false readings due to vibration

### 4. Wire the Components

Follow the circuit diagram in the `diagrams` folder to connect all components:

#### Power Connections:
- Connect Arduino 5V to L293D/L298N VCC (logic power)
- Connect 9V Battery positive to L293D/L298N VS (motor power)
- Connect Arduino GND, Battery negative, and L293D/L298N GND pins together

#### IR Sensors:
- Left IR Sensor:
  - VCC → Arduino 5V
  - GND → Arduino GND
  - OUT → Arduino A0
- Right IR Sensor:
  - VCC → Arduino 5V
  - GND → Arduino GND
  - OUT → Arduino A1

#### Ultrasonic Sensor:
- VCC → Arduino 5V
- GND → Arduino GND
- TRIG → Arduino pin 11
- ECHO → Arduino pin 12

#### L293D/L298N Motor Driver:
- Enable pins:
  - ENA (Enable A) → Arduino pin 5 (PWM)
  - ENB (Enable B) → Arduino pin 6 (PWM)
- Control pins:
  - IN1 (Input 1) → Arduino pin 7
  - IN2 (Input 2) → Arduino pin 8
  - IN3 (Input 3) → Arduino pin 9
  - IN4 (Input 4) → Arduino pin 10
- Output pins:
  - Connect the left side motors (Motors 1 & 2) to OUT1 and OUT2
  - Connect the right side motors (Motors 3 & 4) to OUT3 and OUT4

### 5. Final Checks

1. Double-check all connections against the circuit diagram
2. Ensure there are no loose wires that could get caught in the wheels
3. Verify that the sensors are properly positioned and secured
4. Check that the battery is securely mounted and connected

## Testing the Hardware

Before uploading the final code, it's a good idea to test each component:

1. **Test the Motors**:
   - Upload a simple sketch that runs each motor individually
   - Verify that all motors are turning in the expected direction
   - If any motor turns in the wrong direction, swap its connections

2. **Test the IR Sensors**:
   - Upload a sketch that reads and prints the IR sensor values
   - Place the sensors over white and black surfaces to verify they detect the difference

3. **Test the Ultrasonic Sensor**:
   - Upload a sketch that reads and prints the distance measurements
   - Move objects in front of the sensor to verify it detects them correctly

Once all components are working individually, you can proceed to upload the main robot code.
