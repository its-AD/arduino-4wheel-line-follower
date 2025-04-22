/**
 * Arduino 4-Wheel Robot Line Follower
 * 
 * This sketch controls a 4-wheel robot that follows the borders of a U-shaped track
 * using 2 IR sensors for line detection and 1 ultrasonic sensor for obstacle detection.
 * 
 * Hardware:
 * - Arduino UNO
 * - L293D or L298N motor driver
 * - 4 DC motors
 * - 2 IR sensors
 * - HC-SR04 ultrasonic sensor
 * 
 * Connections:
 * - IR Sensors: A0 (left), A1 (right)
 * - Ultrasonic Sensor: pins 11 (TRIG) and 12 (ECHO)
 * - Motor Driver: pins 5-10
 * 
 * Created: 2025
 * Author: Adil
 * License: MIT
 */

// Motor driver pins
#define ENA 5  // Enable motor A (PWM)
#define ENB 6  // Enable motor B (PWM)
#define IN1 7  // Motor A input 1
#define IN2 8  // Motor A input 2
#define IN3 9  // Motor B input 1
#define IN4 10 // Motor B input 2

// IR Sensor pins
#define LEFT_IR_SENSOR A0    // Left IR sensor
#define RIGHT_IR_SENSOR A1   // Right IR sensor

// Ultrasonic sensor pins
#define TRIG_PIN 11  // Ultrasonic sensor trigger pin
#define ECHO_PIN 12  // Ultrasonic sensor echo pin

// Motor speed settings
#define MAX_SPEED 255     // Maximum motor speed (0-255)
#define NORMAL_SPEED 160  // Normal motor speed
#define SLOW_SPEED 120    // Slow motor speed for turns
#define TURN_SPEED 150    // Speed during turns

// Threshold for IR sensors (adjust based on your sensors)
#define IR_THRESHOLD 500  // Value to distinguish between black line and white surface

// Threshold for obstacle detection (in cm)
#define OBSTACLE_THRESHOLD 15  // Distance in cm to detect obstacles

// Variables to store sensor readings
int leftIRValue;
int rightIRValue;
float obstacleDistance;

// Parking detection variables
unsigned long parkingStartTime = 0;
bool parkingDetected = false;
bool parked = false;

// Track navigation variables
unsigned long lastLineDetectionTime = 0;
bool wasOnLine = false;

/**
 * Setup function - runs once at startup
 * Initializes pins and serial communication
 */
void setup() {
  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize IR sensor pins
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Start with motors off
  stopMotors();
  
  // For debugging
  Serial.begin(9600);
  
  // Wait for 2 seconds before starting
  delay(2000);
}

/**
 * Main loop - runs repeatedly
 * Reads sensors and controls the robot's movement
 */
void loop() {
  // If already parked, do nothing
  if (parked) {
    stopMotors();
    return;
  }
  
  // Read sensor values
  readSensors();
  
  // Print sensor values for debugging
  printSensorValues();
  
  // Check for obstacles
  if (obstacleDistance < OBSTACLE_THRESHOLD && obstacleDistance > 0) {
    // Obstacle detected, perform avoidance
    avoidObstacle();
  } 
  // Check for parking zone
  else if (detectParkingZone()) {
    handleParking();
  } 
  // Normal track navigation
  else {
    navigateTrack();
  }
  
  // Small delay to stabilize readings
  delay(10);
}

/**
 * Reads values from all sensors
 */
void readSensors() {
  // Read IR sensor values
  leftIRValue = analogRead(LEFT_IR_SENSOR);
  rightIRValue = analogRead(RIGHT_IR_SENSOR);
  
  // Read ultrasonic sensor
  obstacleDistance = readUltrasonicDistance();
}

/**
 * Reads distance from ultrasonic sensor
 * @return Distance in centimeters
 */
float readUltrasonicDistance() {
  // Clear the TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Set the TRIG_PIN HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the ECHO_PIN, return the sound wave travel time in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance
  float distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  
  return distance;
}

/**
 * Main navigation function for following the track
 */
void navigateTrack() {
  // With only 2 IR sensors, we'll use a different strategy for staying within the track
  
  // Both sensors detect line (at a border or crossing)
  if (leftIRValue > IR_THRESHOLD && rightIRValue > IR_THRESHOLD) {
    // We're likely at a border crossing or at the bottom of the U
    // Continue forward but slower
    moveForward(SLOW_SPEED);
    wasOnLine = true;
    lastLineDetectionTime = millis();
  }
  // Left sensor detects line (approaching left border)
  else if (leftIRValue > IR_THRESHOLD) {
    // Turn right to stay within track
    turnRight(TURN_SPEED);
    wasOnLine = true;
    lastLineDetectionTime = millis();
  }
  // Right sensor detects line (approaching right border)
  else if (rightIRValue > IR_THRESHOLD) {
    // Turn left to stay within track
    turnLeft(TURN_SPEED);
    wasOnLine = true;
    lastLineDetectionTime = millis();
  }
  // No line detected
  else {
    // If we recently saw a line, we might be moving away from it
    if (wasOnLine && (millis() - lastLineDetectionTime < 1000)) {
      // Continue in the same direction but at normal speed
      moveForward(NORMAL_SPEED);
    } else {
      // We've been away from any line for a while
      // We might be in the middle of the track or lost
      // Implement a search pattern to find the track again
      searchForTrack();
    }
  }
}

/**
 * Search pattern to find the track if lost
 */
void searchForTrack() {
  // Simple search pattern: move forward while slightly turning
  // This helps the robot find the track again if it gets lost
  
  // Alternate between turning slightly left and slightly right
  static bool turnDirection = false;
  
  if (turnDirection) {
    // Turn slightly right while moving forward
    analogWrite(ENA, NORMAL_SPEED);
    analogWrite(ENB, SLOW_SPEED);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    // Turn slightly left while moving forward
    analogWrite(ENA, SLOW_SPEED);
    analogWrite(ENB, NORMAL_SPEED);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  
  // Switch direction for next time
  turnDirection = !turnDirection;
  
  // If we've been searching for too long, we might need to turn more aggressively
  if (millis() - lastLineDetectionTime > 3000) {
    // Make a more aggressive turn to find the track
    if (turnDirection) {
      turnRight(TURN_SPEED);
    } else {
      turnLeft(TURN_SPEED);
    }
    delay(200);
  }
}

/**
 * Detects if the robot has reached the parking zone
 * @return true if parking zone detected, false otherwise
 */
bool detectParkingZone() {
  // For the U-shaped track, the parking zone is at the end of the right side
  
  // With only 2 IR sensors, we need to use a different approach
  // We can detect the parking zone when we reach the end of the U
  // and both sensors no longer detect any lines for a sustained period
  
  // If both sensors don't detect any lines for a while, we might be at the parking zone
  if (leftIRValue < IR_THRESHOLD && rightIRValue < IR_THRESHOLD) {
    // Check if we've been in this state for a while
    if (!parkingDetected) {
      parkingStartTime = millis();
      parkingDetected = true;
    }
    
    // If we've been in this state for more than 1 second, it might be the parking zone
    if (millis() - parkingStartTime > 1000) {
      // Additional check: we should be near a wall (the end of the U)
      if (obstacleDistance < 30 && obstacleDistance > 0) {
        return true;
      }
    }
  } else {
    parkingDetected = false;
  }
  
  return false;
}

/**
 * Handles the parking maneuver
 */
void handleParking() {
  // Slow down as we approach parking
  moveForward(SLOW_SPEED);
  delay(300);
  
  // Turn right to align with parking spot
  turnRight(SLOW_SPEED);
  delay(400);
  
  // Move forward into parking spot
  moveForward(SLOW_SPEED);
  delay(500);
  
  // Stop motors when parked
  stopMotors();
  parked = true;
  
  // Optional: Indicate successful parking with LED or buzzer
  Serial.println("Successfully parked!");
}

/**
 * Handles obstacle avoidance
 */
void avoidObstacle() {
  // Stop briefly
  stopMotors();
  delay(200);
  
  // Back up slightly
  moveBackward(SLOW_SPEED);
  delay(300);
  
  // Turn to avoid obstacle
  // With only 2 IR sensors, we'll use a simple strategy
  // Check which IR sensor last detected a line
  if (leftIRValue > rightIRValue) {
    // Left sensor detected more, so turn right
    turnRight(TURN_SPEED);
  } else {
    // Right sensor detected more, so turn left
    turnLeft(TURN_SPEED);
  }
  delay(500);
  
  // Resume forward motion
  moveForward(NORMAL_SPEED);
}

/**
 * Moves the robot forward at the specified speed
 * @param speed Motor speed (0-255)
 */
void moveForward(int speed) {
  // Left motors
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Right motors
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Set speed
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

/**
 * Moves the robot backward at the specified speed
 * @param speed Motor speed (0-255)
 */
void moveBackward(int speed) {
  // Left motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // Right motors
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Set speed
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

/**
 * Turns the robot left at the specified speed
 * @param speed Motor speed (0-255)
 */
void turnLeft(int speed) {
  // Left motors slower or backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // Right motors forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Set speed
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

/**
 * Turns the robot right at the specified speed
 * @param speed Motor speed (0-255)
 */
void turnRight(int speed) {
  // Left motors forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Right motors slower or backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Set speed
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

/**
 * Stops all motors
 */
void stopMotors() {
  // Stop all motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

/**
 * Prints sensor values to Serial for debugging
 */
void printSensorValues() {
  Serial.print("Left IR: ");
  Serial.print(leftIRValue);
  Serial.print(" Right IR: ");
  Serial.print(rightIRValue);
  Serial.print(" Obstacle Distance: ");
  Serial.print(obstacleDistance);
  Serial.println(" cm");
}
