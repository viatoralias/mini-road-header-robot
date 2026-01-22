/*
Mini Road-Header Robot - Full Control
Main sketch for controlling 4 DC motors and 5 servos
Uses PCA9685 servo driver via I2C
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
int servoAngles[] = {90, 90, 90, 90, 90};
bool servoDirections[] = {true, true, true, true, true};

// Motor configuration (adjust pins for your shield)
const int motorPins[4][2] = {{16, 17}, {18, 19}, {21, 22}, {23, 25}};
const int MOTOR_SPEED = 150;

void setup() {
  Serial.begin(115200);
  Serial.println("Mini Road-Header Robot Starting...");
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize PCA9685 (try different addresses if needed)
  if (!pwm.begin()) {
    Serial.println("PCA9685 not found!");
    while(1);
  }
  pwm.setPWMFreq(50);
  
  // Initialize motors
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i][0], OUTPUT);
    pinMode(motorPins[i][1], OUTPUT);
    digitalWrite(motorPins[i][0], HIGH); // Forward
    digitalWrite(motorPins[i][1], LOW);
  }
  
  Serial.println("System Ready!");
}

void loop() {
  // Continuous motor operation
  runMotors();
  
  // Servo sweeping demonstration
  sweepServos();
  
  delay(20); // Main loop delay
}

void runMotors() {
  // All motors run forward continuously
  // In a real implementation, you would control speed via PWM
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    Serial.println("Motors running...");
    lastUpdate = millis();
  }
}

void sweepServos() {
  // Simple back-and-forth sweep for demonstration
  static unsigned long lastServoUpdate = 0;
  static int sweepAngle = 90;
  static bool sweepingUp = true;
  
  if (millis() - lastServoUpdate > 50) {
    if (sweepingUp) {
      sweepAngle += 5;
      if (sweepAngle >= 135) sweepingUp = false;
    } else {
      sweepAngle -= 5;
      if (sweepAngle <= 45) sweepingUp = true;
    }
    
    // Move all servos to the same angle for demo
    for (int i = 0; i < 5; i++) {
      moveServo(i, sweepAngle);
    }
    
    lastServoUpdate = millis();
  }
}

void moveServo(int servoNum, int angle) {
  int pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  pwm.setPWM(servoNum, 0, pulse);
}
