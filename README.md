# Arduino 4-Wheel Robot Line Follower

A robust line-following robot that navigates U-shaped tracks using 2 IR sensors and 1 ultrasonic sensor.

## Project Overview

This project implements a 4-wheel robot capable of following the borders of a U-shaped track. The robot uses two IR sensors to detect track boundaries and one ultrasonic sensor for obstacle detection. It can navigate the track while staying within the boundaries, avoid obstacles, and park in a designated area.

### Features

- Line following using 2 IR sensors
- Obstacle detection and avoidance using ultrasonic sensor
- Automatic parking at the end of the track
- Robust navigation algorithm that keeps the robot within track boundaries
- Configurable speed settings for different track conditions

## Hardware Requirements

- Arduino UNO or compatible board
- L293D or L298N motor driver
- 4 DC motors with wheels
- 2 IR sensors for line detection
- HC-SR04 ultrasonic sensor for obstacle detection
- 9V battery or other suitable power source
- Chassis for mounting components
- Jumper wires
- Breadboard or PCB for connections

## Circuit Diagram

For detailed wiring instructions, see the text-based circuit diagram in the `diagrams` folder and the [Hardware Setup](docs/hardware_setup.md) guide.

## Software

The robot is controlled by an Arduino sketch that reads sensor inputs and controls the motors accordingly. The main algorithm uses a state-based approach to navigate the track, detect obstacles, and park at the designated area.

Key components of the code:
- Sensor reading and calibration
- Motor control functions
- Line following algorithm
- Obstacle avoidance logic
- Parking detection and execution

## Getting Started

### 1. Assemble the Hardware

Follow the [Hardware Setup Guide](docs/hardware_setup.md) to assemble your robot.

### 2. Upload the Code

1. Clone this repository:
   ```
   git clone https://github.com/yourusername/arduino-4wheel-line-follower.git
   ```

2. Open `src/robot_line_follower.ino` in the Arduino IDE

3. Connect your Arduino board to your computer

4. Upload the code to your Arduino

### 3. Test the Robot

Place the robot on your U-shaped track and power it on. The robot should:
1. Follow the track boundaries
2. Avoid any obstacles placed in its path
3. Park at the designated area at the end of the track

## Customization

You can customize the robot's behavior by modifying these parameters in the code:

- `NORMAL_SPEED`, `SLOW_SPEED`, `TURN_SPEED`: Adjust motor speeds
- `IR_THRESHOLD`: Change the threshold for line detection
- `OBSTACLE_THRESHOLD`: Modify the distance for obstacle detection

## Contributing

Contributions to improve the project are welcome! Please feel free to submit a pull request or open an issue to discuss potential changes/additions.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Thanks to the Arduino community for their valuable resources
- Inspired by various line follower robot designs
