#include "MPU6050.h"
#include "Wire.h"

MPU6050 Gyro;

// Stepper Pins
const int stepPin1 = 22;
const int dirPin1  = 24;
const int enPin1   = 26;
const int stepPin2 = 23;
const int dirPin2  = 25;
const int enPin2   = 27;

// Speed constants (microseconds per half-step)
// At 20000 pulses/rev, 122mm wheel diameter:
//   50us  = 60 RPM = 0.38 m/s  (travel)
//   80us  = 37 RPM             (turn fast)
//   500us = 6 RPM              (turn slow/creep)
const unsigned long TRAVEL_INTERVAL  = 50;
const unsigned long TURN_FAST        = 80;
const unsigned long TURN_SLOW        = 500;
const unsigned long STOPPED_INTERVAL = 99999;

unsigned long leftInterval  = STOPPED_INTERVAL;
unsigned long rightInterval = STOPPED_INTERVAL;
volatile boolean stepState1 = LOW;
volatile boolean stepState2 = LOW;
volatile unsigned long lastStepTime1 = 0;
volatile unsigned long lastStepTime2 = 0;
bool motorsRunning = false;

// Gyro
float gzBias  = 0;
float yaw     = 0;
float targetYaw = 0;
unsigned long lastGyroTime = 0;
float gzFiltered = 0.0f;

// Straight-line P controller
const float Kp_straight    = 1.5f;      // lower gain to tame oscillation
const float MAX_CORRECTION = 150.0f;    // cap steering asymmetry

// Rotation P controller zones (degrees)
const float FAST_ZONE = 30.0f;
const float SLOW_ZONE = 10.0f;  // ease into stop sooner
const float STOP_ZONE = 5.0f;   // stop a little early to avoid endless spin

float initialYaw     = 0.0f;
float targetRotation = 0.0f;
bool  isRotating     = false;

// Telemetry throttle
const unsigned long TELEMETRY_INTERVAL = 50;  // ms
unsigned long lastTelemetry = 0;

// Command control
char currentDirection = 's';
char prevDirection    = 'X';   // force first applyDirection on boot
unsigned long lastCommandTime = 0;
const unsigned long CMD_TIMEOUT = 700;

// ======================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(stepPin1, OUTPUT); pinMode(dirPin1, OUTPUT); pinMode(enPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT); pinMode(dirPin2, OUTPUT); pinMode(enPin2, OUTPUT);

  // Motors OFF during calibration
  digitalWrite(enPin1, HIGH);
  digitalWrite(enPin2, HIGH);
  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);

  Gyro.initialize();
  Gyro.setSleepEnabled(false);
  Gyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  // Warm-up: let MPU6050 thermally stabilise before calibrating
  Serial.println("Warming up (3s) keep still...");
  delay(3000);

  calibrateGyroBias();

  Serial.print("Bias: ");
  Serial.println(gzBias, 5);

  // Enable motors
  digitalWrite(enPin1, LOW);
  digitalWrite(enPin2, LOW);

  yaw = 0.0f;
  targetYaw = 0.0f;
  lastGyroTime    = millis();
  lastCommandTime = millis();

  Serial.println("READY f=fwd b=back l=left90 r=right90 s=stop");
}

// ======================================================
void loop() {

  // --- Serial input ---
  while (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'f' || cmd == 'b' || cmd == 'l' || cmd == 'r' || cmd == 's') {
      currentDirection = cmd;
      lastCommandTime  = millis();
    } else if (cmd == '?') {
      Serial.print("ACK y:");
      Serial.print(yaw, 2);
      Serial.print(" d:");
      Serial.println(currentDirection);
      lastCommandTime = millis();
    }
    // ignore newlines, spaces, other chars
  }

  // Safety: Pi silent -> stop
  if (millis() - lastCommandTime > CMD_TIMEOUT) {
    currentDirection = 's';
  }

  // Apply only on change
  if (currentDirection != prevDirection) {
    applyDirection(currentDirection);
    prevDirection = currentDirection;
  }

  // --- Stepper pulses ---
  unsigned long now = micros();
  if (motorsRunning) {
    if (now - lastStepTime1 >= leftInterval / 2) {
      stepState1 = !stepState1;
      digitalWrite(stepPin1, stepState1);
      lastStepTime1 = now;
    }
    if (now - lastStepTime2 >= rightInterval / 2) {
      stepState2 = !stepState2;
      digitalWrite(stepPin2, stepState2);
      lastStepTime2 = now;
    }
  }

  // --- Gyro & control loop ---
  unsigned long nowGyro = millis();
  if (nowGyro - lastGyroTime >= 5) { // run at ~200 Hz max
    float dt = (nowGyro - lastGyroTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;  // clamp outliers
    lastGyroTime = nowGyro;

    int16_t gx, gy, gzRaw;
    Gyro.getRotation(&gx, &gy, &gzRaw);

    float gz = (gzRaw / 131.0f) - gzBias;
    if (fabsf(gz) < 0.08f) gz = 0.0f;  // noise deadband
    // simple low-pass to smooth spikes
    gzFiltered = 0.9f * gzFiltered + 0.1f * gz;
    gz = gzFiltered;

    yaw += gz * dt;
    if (yaw >  180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;

    if (motorsRunning) {
      if (isRotating) {
        runRotationController();
      } else {
        runStraightController();
      }
    }

    // Telemetry throttled to avoid slowing control loop
    if (nowGyro - lastTelemetry >= TELEMETRY_INTERVAL) {
      lastTelemetry = nowGyro;
      Serial.print("Y:"); Serial.print(yaw, 2);
      Serial.print(" T:"); Serial.print(isRotating ? (initialYaw + targetRotation) : targetYaw, 2);
      Serial.print(" E:"); Serial.print(isRotating ? getRotationError() : (targetYaw - yaw), 2);
      Serial.print(" D:"); Serial.print(currentDirection);
      Serial.print(" L:"); Serial.print(leftInterval);
      Serial.print(" R:"); Serial.println(rightInterval);
    }
  }
}

// ======================================================
float getRotationError() {
  float rotated = yaw - initialYaw;
  if (rotated >  180.0f) rotated -= 360.0f;
  if (rotated < -180.0f) rotated += 360.0f;
  float err = targetRotation - rotated;
  if (err >  180.0f) err -= 360.0f;
  if (err < -180.0f) err += 360.0f;
  return err;
}

// ======================================================
void runRotationController() {
  float err    = getRotationError();
  float absErr = fabsf(err);

  if (absErr < STOP_ZONE) {
    // Rotation complete
    motorsRunning    = false;
    isRotating       = false;
    leftInterval     = STOPPED_INTERVAL;
    rightInterval    = STOPPED_INTERVAL;
    targetYaw        = yaw;
    currentDirection = 's';
    prevDirection    = 's';
    Serial.print("ROT_DONE y:");
    Serial.println(yaw, 2);
    return;
  }

  // Proportional speed
  unsigned long interval;
  if (absErr >= FAST_ZONE) {
    interval = TURN_FAST;
  } else if (absErr <= SLOW_ZONE) {
    interval = TURN_SLOW;
  } else {
    float t  = (absErr - SLOW_ZONE) / (FAST_ZONE - SLOW_ZONE);
    interval = (unsigned long)(TURN_SLOW - t * (TURN_SLOW - TURN_FAST));
  }
  leftInterval  = interval;
  rightInterval = interval;

  // Auto-correct direction if overshoot
  if (err > 0) {
    digitalWrite(dirPin1, HIGH);  // left fwd
    digitalWrite(dirPin2, HIGH);  // right back  -> turns right
  } else {
    digitalWrite(dirPin1, LOW);   // left back
    digitalWrite(dirPin2, LOW);   // right fwd   -> turns left
  }
}

// ======================================================
void runStraightController() {
  float err = targetYaw - yaw;
  if (err >  180.0f) err -= 360.0f;
  if (err < -180.0f) err += 360.0f;

  float correction = Kp_straight * err;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  long base    = (long)TRAVEL_INTERVAL;
  long corrInt = (long)correction;

  // keep both wheels within a sensible speed band to avoid hunting
  leftInterval  = (unsigned long)constrain(base + corrInt, 35L, 220L);
  rightInterval = (unsigned long)constrain(base - corrInt, 35L, 220L);
}

// ======================================================
void applyDirection(char dir) {
  switch (dir) {
    case 'f':
      isRotating    = false;
      targetYaw     = yaw;
      leftInterval  = TRAVEL_INTERVAL;
      rightInterval = TRAVEL_INTERVAL;
      motorsRunning = true;
      digitalWrite(dirPin1, HIGH);
      digitalWrite(dirPin2, LOW);
      Serial.println("CMD:f");
      break;

    case 'b':
      isRotating    = false;
      targetYaw     = yaw;
      leftInterval  = TRAVEL_INTERVAL;
      rightInterval = TRAVEL_INTERVAL;
      motorsRunning = true;
      digitalWrite(dirPin1, LOW);
      digitalWrite(dirPin2, HIGH);
      Serial.println("CMD:b");
      break;

    case 'l':
      initialYaw     = yaw;
      targetRotation = -90.0f;
      isRotating     = true;
      leftInterval   = TURN_FAST;
      rightInterval  = TURN_FAST;
      motorsRunning  = true;
      digitalWrite(dirPin1, LOW);
      digitalWrite(dirPin2, LOW);
      Serial.println("CMD:l");
      break;

    case 'r':
      initialYaw     = yaw;
      targetRotation = 90.0f;
      isRotating     = true;
      leftInterval   = TURN_FAST;
      rightInterval  = TURN_FAST;
      motorsRunning  = true;
      digitalWrite(dirPin1, HIGH);
      digitalWrite(dirPin2, HIGH);
      Serial.println("CMD:r");
      break;

    case 's':
      motorsRunning  = false;
      isRotating     = false;
      leftInterval   = STOPPED_INTERVAL;
      rightInterval  = STOPPED_INTERVAL;
      Serial.println("CMD:s");
      break;
  }
}

// ======================================================
// Calibration: 2000 samples x 5ms = 10 seconds
// Robot MUST be completely still
// ======================================================
void calibrateGyroBias() {
  long sum = 0;
  const int SAMPLES = 2000;
  Serial.println("Calibrating...");
  for (int i = 0; i < SAMPLES; i++) {
    int16_t gx, gy, gz;
    Gyro.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(5);
    if (i % 500 == 0) {
      Serial.print("  ");
      Serial.print((i * 100) / SAMPLES);
      Serial.println("%");
    }
  }
  gzBias = (sum / (float)SAMPLES) / 131.0f;
  yaw    = 0.0f;
  Serial.println("  100% done");
}
