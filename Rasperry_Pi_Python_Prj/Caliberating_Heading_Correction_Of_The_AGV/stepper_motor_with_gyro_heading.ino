#include "MPU6050.h"
#include "Wire.h"

MPU6050 Gyro;

// ----------------------
// Stepper Pins
// ----------------------
const int stepPin1 = 22;
const int dirPin1  = 24;
const int enPin1   = 26;

const int stepPin2 = 23;
const int dirPin2  = 25;
const int enPin2   = 27;

// ----------------------
// Stepper Control
// ----------------------
volatile unsigned long lastStepTime1 = 0;
volatile unsigned long lastStepTime2 = 0;

unsigned long baseInterval = 1000;  // base speed (microseconds)
unsigned long leftInterval  = 1000;
unsigned long rightInterval = 1000;

volatile boolean stepState1 = LOW;
volatile boolean stepState2 = LOW;

// ----------------------
// Gyro Variables
// ----------------------
float gzBias = 0;
float yaw = 0;
float targetYaw = 0;

const float dt = 0.01;  // 100 Hz
unsigned long lastGyroTime = 0;

// ----------------------
// Heading Control
// ----------------------
float Kp = 2.0;  // tune this
bool movingStraight = true;

// ----------------------
// Serial Command Control
// ----------------------
char currentDirection = 's';  // s=stopped, f=forward, b=backward, l=left, r=right
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 500;  // stop if no command for 500ms

// ----------------------
// Setup
// ----------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  Gyro.initialize();
  Gyro.setSleepEnabled(false);
  Gyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  calibrateGyroBias();

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enPin2, OUTPUT);

  digitalWrite(enPin1, LOW);
  digitalWrite(enPin2, LOW);

  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, LOW);

  targetYaw = yaw;  // lock heading
  lastGyroTime = millis();
  lastCommandTime = millis();

  Serial.println("AGV Ready");
  Serial.println("Commands: f=forward, b=backward, l=left, r=right, s=stop");
}

// ----------------------
// Main Loop
// ----------------------
void loop() {

  // ----------------------
  // Handle Serial Commands
  // ----------------------
  while (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'f' || cmd == 'b' || cmd == 'l' || cmd == 'r' || cmd == 's') {
      currentDirection = cmd;
      lastCommandTime = millis();
      Serial.print("CMD: ");
      Serial.println(currentDirection);
    }
  }

  // Auto-stop if no command received (safety feature)
  if (millis() - lastCommandTime > commandTimeout) {
    currentDirection = 's';
  }

  unsigned long currentMicros = micros();

  // ----------------------
  // Stepper Motor 1
  // ----------------------
  if (currentMicros - lastStepTime1 >= leftInterval / 2) {
    stepState1 = !stepState1;
    digitalWrite(stepPin1, stepState1);
    lastStepTime1 = currentMicros;
  }

  // ----------------------
  // Stepper Motor 2
  // ----------------------
  if (currentMicros - lastStepTime2 >= rightInterval / 2) {
    stepState2 = !stepState2;
    digitalWrite(stepPin2, stepState2);
    lastStepTime2 = currentMicros;
  }

  // ----------------------
  // Gyro Update at 100 Hz
  // ----------------------
  if (millis() - lastGyroTime >= 10) {

    lastGyroTime += 10;

    int16_t gx, gy, gzRaw;
    Gyro.getRotation(&gx, &gy, &gzRaw);

    float gz = gzRaw / 131.0f - gzBias;

    yaw += gz * dt;

    // Wrap yaw
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;

    // Update movement based on command
    updateMovement();

    if (movingStraight) {
      float error = targetYaw - yaw;

      if (error > 180) error -= 360;
      if (error < -180) error += 360;

      float correction = Kp * error;

      // Apply correction to step intervals
      leftInterval  = baseInterval + correction;
      rightInterval = baseInterval - correction;

      // Clamp intervals
      leftInterval  = constrain(leftInterval, 500, 2000);
      rightInterval = constrain(rightInterval, 500, 2000);
    }

    Serial.print("Yaw: ");
    Serial.print(yaw);
    Serial.print(" | Dir: ");
    Serial.print(currentDirection);
    Serial.print(" | Left: ");
    Serial.print(leftInterval);
    Serial.print(" | Right: ");
    Serial.println(rightInterval);
  }
}

// ----------------------
// Gyro Bias Calibration
// ----------------------
void calibrateGyroBias() {

  long sum = 0;
  const int samples = 1000;

  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    Gyro.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(10);
  }

  gzBias = (sum / (float)samples) / 131.0f;
}

// ----------------------
// Update Movement Based on Command
// ----------------------
void updateMovement() {
  switch (currentDirection) {

    case 'f':  // Forward
      movingStraight = true;
      baseInterval = 1000;
      digitalWrite(dirPin1, HIGH);
      digitalWrite(dirPin2, LOW);
      break;

    case 'b':  // Backward
      movingStraight = true;
      baseInterval = 1000;
      digitalWrite(dirPin1, LOW);
      digitalWrite(dirPin2, HIGH);
      break;

    case 'l':  // Turn Left
      movingStraight = false;
      leftInterval = 100;     // slow down left motor
      rightInterval = 1500;   // speed up right motor
      digitalWrite(dirPin1, HIGH);
      digitalWrite(dirPin2, LOW);
      break;

    case 'r':  // Turn Right
      movingStraight = false;
      leftInterval = 1500;    // speed up left motor
      rightInterval = 100;    // slow down right motor
      digitalWrite(dirPin1, HIGH);
      digitalWrite(dirPin2, LOW);
      break;

    case 's':  // Stop
      movingStraight = false;
      leftInterval = 10000;   // very slow
      rightInterval = 10000;  // very slow
      break;
  }
}