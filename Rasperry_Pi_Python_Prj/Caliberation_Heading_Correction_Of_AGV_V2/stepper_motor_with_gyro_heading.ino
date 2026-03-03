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
const int MAX_STEPS = 52166;          // max steps for distance calculation

bool motorsRunning = false;           // MOTORS DO NOTHING until Pi sends a command
bool distanceReached = false;         // flag for distance-based movement
unsigned long targetSteps = 0;        // target distance in steps

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

// For rotation-based commands (l, r, b - 180 degree)
float initialYaw = 0.0f;
float targetRotation = 0.0f;        // target rotation angle in degrees
bool isRotationCommand = false;     // flag to indicate if current command is rotation-based

// ----------------------
// Serial Command Control
// ----------------------
char currentDirection = 's';         // Start STOPPED — no movement on power-on
char prevDirection    = 's';
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 600;  // ms

// For parsing distance commands
String serialBuffer = "";            // buffer for serial input

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
    } else if (cmd == 'd') {
      // Distance command: parse number until newline
      serialBuffer = "";
      while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
          break;
        }
        if (isdigit(c)) {
          serialBuffer += c;
        }
      }
      if (serialBuffer.length() > 0) {
        targetSteps = (unsigned long)serialBuffer.toInt();
        Serial.print("Distance set to: ");
        Serial.print(targetSteps);
        Serial.println(" steps");
        lastCommandTime = millis();
      }
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
  if (motorsRunning && (currentMicros - lastStepTime1 >= leftInterval / 2)) {
    stepState1 = !stepState1;
    digitalWrite(stepPin1, stepState1);
    lastStepTime1 = currentMicros;

    if (stepState1 == LOW) stepCount1++;   // count on falling edge
  }

  // Motor 2 (Right)
  if (motorsRunning && (currentMicros - lastStepTime2 >= rightInterval / 2)) {
    stepState2 = !stepState2;
    digitalWrite(stepPin2, stepState2);
    lastStepTime2 = currentMicros;

    if (stepState2 == LOW) stepCount2++;   // count on falling edge
  }

  // ----------------------
  // Check if target distance reached (for f command)
  // ----------------------
  if (motorsRunning && currentDirection == 'f' && targetSteps > 0) {
    if (stepCount1 >= targetSteps && stepCount2 >= targetSteps) {
      motorsRunning = false;
      currentDirection = 's';
      Serial.println("Target distance reached — stopping");
    }
  }

  // ----------------------
  // Check if rotation target reached (for b, l, r commands)
  // ----------------------
  if (motorsRunning && isRotationCommand) {
    float currentRotation = yaw - initialYaw;
    if (currentRotation > 180.0f) currentRotation -= 360.0f;
    if (currentRotation < -180.0f) currentRotation += 360.0f;

    if (abs(currentRotation - targetRotation) < 5.0f) {  // 5 degree tolerance
      motorsRunning = false;
      currentDirection = 's';
      isRotationCommand = false;
      Serial.print("Target rotation reached (");
      Serial.print(targetRotation, 1);
      Serial.println(" deg) — stopping");
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
      targetYaw     = yaw;          // lock current heading
      targetSteps   = MAX_STEPS;    // default: use full MAX_STEPS, can be overridden by Pi
      isRotationCommand = false;
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
      initialYaw    = yaw;          // save current heading
      targetRotation = 180.0f;      // turn exactly 180 degrees
      isRotationCommand = true;
      motorsRunning = true;
      digitalWrite(dirPin1, LOW);   // reverse direction
      digitalWrite(dirPin2, HIGH);
      Serial.println("CMD: Backward (180-degree turn)");
      break;

    case 'l':
      leftInterval  = 100;
      rightInterval = 100;
      stepCount1    = 0;
      stepCount2    = 0;
      initialYaw    = yaw;          // save current heading
      targetRotation = -90.0f;      // turn 90 degrees left
      isRotationCommand = true;
      motorsRunning = true;
      digitalWrite(dirPin1, LOW);   // left motor reverse
      digitalWrite(dirPin2, LOW);   // right motor forward
      Serial.println("CMD: Left (90-degree turn)");
      break;

    case 'r':
      leftInterval  = 100;
      rightInterval = 100;
      stepCount1    = 0;
      stepCount2    = 0;
      initialYaw    = yaw;          // save current heading
      targetRotation = 90.0f;       // turn 90 degrees right
      isRotationCommand = true;
      motorsRunning = true;
      digitalWrite(dirPin1, HIGH);  // left motor forward
      digitalWrite(dirPin2, HIGH);  // right motor reverse
      Serial.println("CMD: Right (90-degree turn)");
      break;

    case 's':
      motorsRunning = false;        // stop pulse generation immediately
      stepCount1    = 0;
      stepCount2    = 0;
      isRotationCommand = false;
      targetSteps = 0;
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
