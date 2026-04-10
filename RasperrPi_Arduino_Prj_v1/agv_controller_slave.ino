/*
  AGV Controller - Trapezoidal Motion (Slave Mode)
  Receives commands from Raspberry Pi (Master) via Serial
  Sends status messages back to Master
*/

const int STEP_PIN1 = 22;
const int DIR_PIN1  = 24;
const int EN_PIN1   = 26;

const int STEP_PIN2 = 23;
const int DIR_PIN2  = 25;
const int EN_PIN2   = 27;

// ===== CONFIG =====
const unsigned long PULSES_PER_REV = 10000UL;

const float MAX_RPM = 100.0;
const float INITIAL_RPM = 5.0;
const float ACC_RPM_PER_SEC = 20.0;

const float ACCEL_TIME = (MAX_RPM - INITIAL_RPM) / ACC_RPM_PER_SEC;
const float CRUISE_TIME = 3.0;   // adjust

// ===== STATE =====
bool moving = false;

unsigned long lastStepTime = 0;
unsigned long motion_start_time = 0;

bool stepState = LOW;

float current_speed_rpm = 0;

// ===== STATE MACHINE =====
enum AGVState {
  STOPPED,
  MOVING_FORWARD,
  PIVOTING
};

AGVState state = STOPPED;

// ===== SERIAL COMMAND BUFFER =====
String serialBuffer = "";

// ===== STEP INTERVAL =====
unsigned long calculate_interval(float rpm) {
  if (rpm <= 1.0) return 1000000000;

  unsigned long interval = (unsigned long)(60000000.0 / (PULSES_PER_REV * rpm));

  if (interval < 50) interval = 50;
  return interval;
}

// ===== TRAPEZOIDAL SPEED =====
float get_speed(float t) {

  float accel_rate = (MAX_RPM - INITIAL_RPM) / ACCEL_TIME;

  // ACCEL
  if (t < ACCEL_TIME) {
    return INITIAL_RPM + accel_rate * t;
  }

  // CRUISE
  else if (t < (ACCEL_TIME + CRUISE_TIME)) {
    return MAX_RPM;
  }

  // DECEL
  else {
    float td = t - (ACCEL_TIME + CRUISE_TIME);
    float speed = MAX_RPM - accel_rate * td;

    if (speed < 0) speed = 0;
    return speed;
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(EN_PIN1, OUTPUT);

  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(EN_PIN2, OUTPUT);

  digitalWrite(EN_PIN1, LOW);
  digitalWrite(EN_PIN2, LOW);

  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(DIR_PIN2, LOW);

  delay(1000);
  Serial.println("READY:AGV_Controller_Slave");
}

// ===== LOOP =====
void loop() {

  // ---- SERIAL COMMAND ----
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }

  // ---- MOTION ----
  if (moving) {
    runMotion();
  }
}

// ===== PROCESS COMMAND =====
void processCommand(String cmd) {
  cmd.trim();
  
  if (cmd == "f" || cmd == "F") {
    startForward();
  }
  else if (cmd == "p" || cmd == "P") {
    startPivot();
  }
  else if (cmd == "s" || cmd == "S") {
    stopAll();
  }
  else if (cmd == "STATUS") {
    sendStatus();
  }
}

// ===== START FORWARD =====
void startForward() {
  if (moving) {
    // Already moving, ignore
    return;
  }
  
  state = MOVING_FORWARD;
  motion_start_time = micros();
  lastStepTime = motion_start_time;
  moving = true;

  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(DIR_PIN2, LOW);

  Serial.println("EVENT:FORWARD_START");
}

// ===== STOP =====
void stopAll() {
  moving = false;
  state = STOPPED;

  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);

  Serial.println("EVENT:STOPPED");
}

// ===== PIVOT =====
void startPivot() {
  if (moving) {
    // Already moving, ignore
    return;
  }
  
  state = PIVOTING;
  moving = true;
  motion_start_time = micros();
  lastStepTime = motion_start_time;

  // Motor 1 forward, Motor 2 backward for pivot
  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(DIR_PIN2, HIGH);

  Serial.println("EVENT:PIVOT_START");
}

// ===== MOTION CORE =====
void runMotion() {

  unsigned long now = micros();
  float t = (now - motion_start_time) / 1000000.0;

  current_speed_rpm = get_speed(t);

  unsigned long interval = calculate_interval(current_speed_rpm);

  if (now - lastStepTime >= interval) {

    lastStepTime = now;

    // STEP PULSE
    digitalWrite(STEP_PIN1, HIGH);
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);
  }

  // STOP automatically after full trapezoid
  if (current_speed_rpm <= 0 && t > (ACCEL_TIME + CRUISE_TIME)) {
    moving = false;
    state = STOPPED;
    
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);

    if (state == PIVOTING) {
      Serial.println("EVENT:PIVOT_DONE");
    } else {
      Serial.println("EVENT:MOTION_COMPLETE");
    }
  }
}

// ===== SEND STATUS =====
void sendStatus() {
  Serial.print("STATUS:");
  Serial.print("state=");
  
  if (state == STOPPED) {
    Serial.print("STOPPED");
  } else if (state == MOVING_FORWARD) {
    Serial.print("MOVING_FORWARD");
  } else if (state == PIVOTING) {
    Serial.print("PIVOTING");
  }
  
  Serial.print("|speed=");
  Serial.print(current_speed_rpm, 2);
  Serial.println("rpm");
}
