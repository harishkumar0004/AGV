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

unsigned long baseInterval = 50;    // microseconds per half-step
unsigned long leftInterval  = 1000;
unsigned long rightInterval = 1000;

volatile boolean stepState1 = LOW;
volatile boolean stepState2 = LOW;

volatile int stepCount1 = 0;
volatile int stepCount2 = 0;
const int MAX_STEPS = 52166;          // max steps before auto-pause

bool motorsRunning = false;           // MOTORS DO NOTHING until Pi sends a command

// ----------------------
// Gyro Variables
// ----------------------
float gzBias = 0;
float yaw = 0;
float targetYaw = 0;

const float dt = 0.01;
unsigned long lastGyroTime = 0;

// ----------------------
// Heading Control
// ----------------------
float Kp = 2.0;
float maxCorrection = 400.0;

// ----------------------
// Serial Command Control
// ----------------------
char currentDirection = 's';         // Start STOPPED — no movement on power-on
char prevDirection    = 's';
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 600;  // ms

// ----------------------
// Setup
// ----------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  Gyro.initialize();
  Gyro.setSleepEnabled(false);
  Gyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1,  OUTPUT);
  pinMode(enPin1,   OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2,  OUTPUT);
  pinMode(enPin2,   OUTPUT);

  // Keep motors DISABLED during calibration
  digitalWrite(enPin1, HIGH);
  digitalWrite(enPin2, HIGH);

  // Step pins LOW
  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);

  Serial.println("Calibrating gyro, keep robot still...");
  calibrateGyroBias();
  Serial.print("Gyro bias: ");
  Serial.println(gzBias, 4);

  // Enable motors AFTER calibration — but motorsRunning is still false
  digitalWrite(enPin1, LOW);
  digitalWrite(enPin2, LOW);

  yaw = 0.0f;
  targetYaw = 0.0f;
  lastGyroTime  = millis();
  lastCommandTime = millis();

  Serial.println("AGV Ready — waiting for command from Pi");
  Serial.println("Commands: f=forward, b=backward, l=left, r=right, s=stop");
}

// ----------------------
// Main Loop
// ----------------------
void loop() {

  // ----------------------
  // Read Serial Commands from Pi
  // ----------------------
  while (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '?') {
      // Ping/health check from Pi
      Serial.println("ACK");
      lastCommandTime = millis();
    } else if (cmd == 'h') {
      // Heading snapshot for debugging
      Serial.print("STATUS yaw:");
      Serial.print(yaw, 2);
      Serial.print(" tgt:");
      Serial.print(targetYaw, 2);
      Serial.print(" dir:");
      Serial.println(currentDirection);
      lastCommandTime = millis();
    } else if (cmd == 'f' || cmd == 'b' || cmd == 'l' || cmd == 'r' || cmd == 's') {
      currentDirection = cmd;
      lastCommandTime  = millis();
    }
  }

  // Safety: auto-stop if Pi goes silent
  if (millis() - lastCommandTime > commandTimeout) {
    currentDirection = 's';
  }

  // Apply direction only when it changes
  if (currentDirection != prevDirection) {
    applyDirection(currentDirection);
    prevDirection = currentDirection;
  }

  // ----------------------
  // Stepper Pulse Generation
  // — only runs when motorsRunning = true
  // ----------------------
  unsigned long currentMicros = micros();

  // Motor 1 (Left)
  if (motorsRunning && stepCount1 < MAX_STEPS &&
      (currentMicros - lastStepTime1 >= leftInterval / 2)) {

    stepState1 = !stepState1;
    digitalWrite(stepPin1, stepState1);
    lastStepTime1 = currentMicros;

    if (stepState1 == LOW) stepCount1++;   // count on falling edge
  }

  // Motor 2 (Right)
  if (motorsRunning && stepCount2 < MAX_STEPS &&
      (currentMicros - lastStepTime2 >= rightInterval / 2)) {

    stepState2 = !stepState2;
    digitalWrite(stepPin2, stepState2);
    lastStepTime2 = currentMicros;

    if (stepState2 == LOW) stepCount2++;   // count on falling edge
  }

  // ----------------------
  // MAX_STEPS Reached — pause and reset
  // (only relevant for forward/backward continuous runs)
  // ----------------------
  if (motorsRunning &&
      currentDirection != 's' &&
      stepCount1 >= MAX_STEPS && stepCount2 >= MAX_STEPS) {

    Serial.println("MAX_STEPS reached — pausing 1s");
    motorsRunning = false;

    delay(1000);  // 1 second pause

    // Reset counters and resume
    stepCount1 = 0;
    stepCount2 = 0;
    lastStepTime1 = micros();
    lastStepTime2 = micros();

    if (currentDirection != 's') {
      motorsRunning = true;   // continue if still commanded
    }
  }

  // ----------------------
  // Gyro Update at 100 Hz
  // ----------------------
  if (millis() - lastGyroTime >= 10) {
    lastGyroTime += 10;

    int16_t gx, gy, gzRaw;
    Gyro.getRotation(&gx, &gy, &gzRaw);

    float gz = (gzRaw / 131.0f) - gzBias;
    if (abs(gz) < 0.05f) gz = 0.0f;   // deadband

    yaw += gz * dt;

    if (yaw >  180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;

    // Heading correction — only when driving straight
    if (motorsRunning && (currentDirection == 'f' || currentDirection == 'b')) {
      float error = targetYaw - yaw;
      if (error >  180.0f) error -= 360.0f;
      if (error < -180.0f) error += 360.0f;

      float correction = Kp * error;
      correction = constrain(correction, -maxCorrection, maxCorrection);

      long base     = (long)baseInterval;
      long corrInt  = (long)correction;

      leftInterval  = (unsigned long)constrain(base + corrInt, 25L, 200L);
      rightInterval = (unsigned long)constrain(base - corrInt, 25L, 200L);
    }

    // Debug to Pi
    Serial.print("Yaw:");    Serial.print(yaw, 2);
    Serial.print(" Tgt:");   Serial.print(targetYaw, 2);
    Serial.print(" Dir:");   Serial.print(currentDirection);
    Serial.print(" Steps1:"); Serial.print(stepCount1);
    Serial.print(" Steps2:"); Serial.print(stepCount2);
    Serial.print(" L:");     Serial.print(leftInterval);
    Serial.print(" R:");     Serial.println(rightInterval);
  }
}

// ----------------------
// Apply Direction (called only on change)
// ----------------------
void applyDirection(char dir) {
  switch (dir) {

    case 'f':
      baseInterval  = 50;
      leftInterval  = baseInterval;
      rightInterval = baseInterval;
      stepCount1    = 0;
      stepCount2    = 0;
      targetYaw     = yaw;          // lock heading
      motorsRunning = true;
      digitalWrite(dirPin1, HIGH);
      digitalWrite(dirPin2, LOW);
      Serial.println("CMD: Forward");
      break;

    case 'b':
      baseInterval  = 50;
      leftInterval  = baseInterval;
      rightInterval = baseInterval;
      stepCount1    = 0;
      stepCount2    = 0;
      targetYaw     = yaw;
      motorsRunning = true;
      digitalWrite(dirPin1, LOW);
      digitalWrite(dirPin2, HIGH);
      Serial.println("CMD: Backward");
      break;

    case 'l':
      leftInterval  = 100;
      rightInterval = 100;
      stepCount1    = 0;
      stepCount2    = 0;
      motorsRunning = true;
      digitalWrite(dirPin1, LOW);   // left motor reverse
      digitalWrite(dirPin2, LOW);   // right motor forward
      Serial.println("CMD: Left");
      break;

    case 'r':
      leftInterval  = 100;
      rightInterval = 100;
      stepCount1    = 0;
      stepCount2    = 0;
      motorsRunning = true;
      digitalWrite(dirPin1, HIGH);  // left motor forward
      digitalWrite(dirPin2, HIGH);  // right motor reverse
      Serial.println("CMD: Right");
      break;

    case 's':
      motorsRunning = false;        // stop pulse generation immediately
      stepCount1    = 0;
      stepCount2    = 0;
      Serial.println("CMD: Stop");
      break;
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
    delay(5);
  }
  gzBias = (sum / (float)samples) / 131.0f;
  yaw    = 0.0f;
}
