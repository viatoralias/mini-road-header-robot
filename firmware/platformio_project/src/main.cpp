#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ========================================
// PCA9685 Servo Driver Configuration
// ========================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo pulse limits for MG996R (in microseconds)
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

// Servo mapping: Port S1-S5 = PCA9685 channels 0-4
int servoAngles[] = {90, 90, 90, 90, 90}; // Start positions for 5 servos
bool servoDirections[] = {true, true, true, true, true}; // true = sweeping up

// Servo sweep parameters
const int SERVO_MIN_ANGLE = 45;
const int SERVO_MAX_ANGLE = 135;
const int SERVO_SWEEP_SPEED = 3; // Degrees per step

// ========================================
// Motor Configuration (ADJUST THESE PINS!)
// ========================================
// IMPORTANT: Check your shield's actual pin mapping
// Common configuration for 4 DC motors:
const int motorPins[4][3] = {
  // {DIR1, DIR2, PCA9685_PWM_CHANNEL}
  {16, 17, 8},   // Motor 1 (M1)
  {18, 19, 9},   // Motor 2 (M2)
  {21, 22, 10},  // Motor 3 (M3) - WARNING: GPIO21/22 are often I2C pins!
  {23, 25, 11}   // Motor 4 (M4)
};

// Motor speed (0-255) and direction
const int MOTOR_SPEED = 180; // Medium-high speed
const bool MOTOR_DIRECTION = true; // true = forward

// ========================================
// Timing Variables
// ========================================
unsigned long lastServoUpdate = 0;
const int SERVO_UPDATE_INTERVAL = 50; // ms between servo updates
int activeServoIndex = 0; // Which servo is currently moving

// ========================================
// Function Declarations
// ========================================
void moveServo(int servoNum, int angle);
void updateServoSweep();
void initializeMotors();
void setMotorSpeed(int motorIndex, int speed, bool forward);
void stopAllMotors();
void scanI2C();
void emergencyStop();

// ========================================
// SETUP FUNCTION
// ========================================
void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for Serial Monitor
  
  Serial.println("\n========================================");
  Serial.println("   Mini Road-Header Robot Control");
  Serial.println("========================================");
  
  // Initialize I2C with explicit pins for Wemos D1 R32
  Wire.begin(21, 22); // SDA=GPIO21, SCL=GPIO22
  delay(100);
  
  // Scan I2C bus to find PCA9685
  Serial.println("\n[1] Scanning I2C bus...");
  scanI2C();
  
  // Try to initialize PCA9685 (common addresses: 0x40, 0x41)
  Serial.println("\n[2] Initializing PCA9685 servo driver...");
  if (!pwm.begin()) {
    // Try alternate address 0x41
    pwm = Adafruit_PWMServoDriver(0x41);
    if (!pwm.begin()) {
      Serial.println("ERROR: PCA9685 not found!");
      Serial.println("Check: 1. I2C wiring 2. Power to shield 3. I2C address jumpers");
      while(1); // Halt if PCA9685 not found
    }
    Serial.println("PCA9685 found at address 0x41");
  } else {
    Serial.println("PCA9685 found at address 0x40");
  }
  
  // Set PWM frequency for servos (standard is 50Hz)
  pwm.setPWMFreq(50);
  Serial.println("PCA9685 initialized at 50Hz");
  
  // Initialize motor control
  Serial.println("\n[3] Initializing DC motors...");
  initializeMotors();
  
  // Initialize servos to start position
  Serial.println("\n[4] Moving servos to start position (90°)...");
  for (int i = 0; i < 5; i++) {
    moveServo(i, servoAngles[i]);
    delay(100);
  }
  
  Serial.println("\n[5] System Ready!");
  Serial.println("========================================");
  Serial.println("Motors: Running forward continuously");
  Serial.println("Servos: Sweeping sequentially 45°-135°");
  Serial.println("Send 'E' for emergency stop");
  Serial.println("========================================");
}

// ========================================
// MAIN LOOP
// ========================================
void loop() {
  unsigned long currentTime = millis();
  
  // 1. Update servo positions (non-blocking)
  if (currentTime - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
    updateServoSweep();
    lastServoUpdate = currentTime;
  }
  
  // 2. Check for emergency stop command
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'E' || command == 'e') {
      emergencyStop();
      Serial.println("EMERGENCY STOP ACTIVATED");
    }
  }
  
  // 3. Optional: Add heartbeat indicator
  static unsigned long lastHeartbeat = 0;
  if (currentTime - lastHeartbeat >= 5000) {
    Serial.println("System running...");
    lastHeartbeat = currentTime;
  }
}

// ========================================
// SERVO CONTROL FUNCTIONS
// ========================================

void moveServo(int servoNum, int angle) {
  // Constrain angle to safe limits
  angle = constrain(angle, 0, 180);
  
  // Convert angle to pulse length
  int pulseLength = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  // Send to PCA9685 (servoNum 0-4 = shield ports S1-S5)
  pwm.setPWM(servoNum, 0, pulseLength);
  
  // Update stored angle
  servoAngles[servoNum] = angle;
}

void updateServoSweep() {
  // This function sweeps servos one at a time for sequential movement
  
  int &currentAngle = servoAngles[activeServoIndex];
  int targetAngle = servoDirections[activeServoIndex] ? SERVO_MAX_ANGLE : SERVO_MIN_ANGLE;
  
  // Move active servo toward its target
  if (servoDirections[activeServoIndex]) {
    currentAngle += SERVO_SWEEP_SPEED;
    if (currentAngle >= targetAngle) {
      currentAngle = targetAngle;
      servoDirections[activeServoIndex] = false;
    }
  } else {
    currentAngle -= SERVO_SWEEP_SPEED;
    if (currentAngle <= targetAngle) {
      currentAngle = targetAngle;
      servoDirections[activeServoIndex] = true;
    }
  }
  
  // Apply the movement
  moveServo(activeServoIndex, currentAngle);
  
  // Switch to next servo when current one reaches target
  if (currentAngle == targetAngle) {
    activeServoIndex = (activeServoIndex + 1) % 5;
  }
}

// ========================================
// MOTOR CONTROL FUNCTIONS
// ========================================

void initializeMotors() {
  // Setup all motor control pins and start motors
  
  for (int i = 0; i < 4; i++) {
    // Setup direction pins
    pinMode(motorPins[i][0], OUTPUT);
    pinMode(motorPins[i][1], OUTPUT);
    
    // Set direction
    if (MOTOR_DIRECTION) {
      digitalWrite(motorPins[i][0], HIGH); // Forward
      digitalWrite(motorPins[i][1], LOW);
    } else {
      digitalWrite(motorPins[i][0], LOW); // Reverse
      digitalWrite(motorPins[i][1], HIGH);
    }
    
    // Set initial speed (convert 0-255 to 0-4095 for PCA9685)
    int pwmValue = map(MOTOR_SPEED, 0, 255, 0, 4095);
    pwm.setPWM(motorPins[i][2], 0, pwmValue);
    
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" initialized on pins ");
    Serial.print(motorPins[i][0]);
    Serial.print(",");
    Serial.print(motorPins[i][1]);
    Serial.print(" (PWM channel ");
    Serial.print(motorPins[i][2]);
    Serial.println(")");
  }
  
  Serial.print("All motors started at speed ");
  Serial.println(MOTOR_SPEED);
}

void setMotorSpeed(int motorIndex, int speed, bool forward) {
  // Control individual motor speed and direction
  
  if (motorIndex < 0 || motorIndex > 3) return;
  
  // Set direction pins
  if (forward) {
    digitalWrite(motorPins[motorIndex][0], HIGH);
    digitalWrite(motorPins[motorIndex][1], LOW);
  } else {
    digitalWrite(motorPins[motorIndex][0], LOW);
    digitalWrite(motorPins[motorIndex][1], HIGH);
  }
  
  // Set speed (stop if speed is 0)
  if (speed == 0) {
    pwm.setPWM(motorPins[motorIndex][2], 0, 0);
  } else {
    int pwmValue = map(speed, 0, 255, 0, 4095);
    pwm.setPWM(motorPins[motorIndex][2], 0, pwmValue);
  }
}

void stopAllMotors() {
  // Stop all DC motors immediately
  
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i][0], LOW);
    digitalWrite(motorPins[i][1], LOW);
    pwm.setPWM(motorPins[i][2], 0, 0);
  }
  Serial.println("All motors stopped");
}

// ========================================
// DIAGNOSTIC FUNCTIONS
// ========================================

void scanI2C() {
  // Scan I2C bus and display found devices
  
  byte error, address;
  int foundDevices = 0;
  
  Serial.println("Scanning I2C addresses 1-127...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("  Found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify common devices
      if (address == 0x40 || address == 0x41) Serial.print(" (Likely PCA9685)");
      
      Serial.println();
      foundDevices++;
    }
  }
  
  if (foundDevices == 0) {
    Serial.println("  No I2C devices found!");
  } else {
    Serial.print("  Total devices found: ");
    Serial.println(foundDevices);
  }
}

void emergencyStop() {
  // Immediately stop all motion
  
  Serial.println("\n--- EMERGENCY STOP ---");
  
  // Stop all motors
  stopAllMotors();
  
  // Freeze servos at current position
  for (int i = 0; i < 5; i++) {
    moveServo(i, servoAngles[i]); // Hold current position
  }
  
  Serial.println("All actuators stopped");
  Serial.println("Send 'R' to reset (you'll need to restart)");
  
  // Wait for reset command
  while(1) {
    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'R' || cmd == 'r') {
        Serial.println("Reset commanded - restart system");
        break;
      }
    }
    delay(100);
  }
}
