/*
  AGV Controller - Trapezoidal Motion (Master-Slave Ready)
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

  Serial.println("Send 'f' (forward), 'p' (pivot), 's' (stop)");
}

// ===== LOOP =====
void loop() {

  // ---- SERIAL COMMAND ----
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'f') startForward();
    else if (cmd == 'p') startPivot();
    else if (cmd == 's') stopAll();
  }

  // ---- MOTION ----
  if (moving) {
    runMotion();
  }
}

// ===== START FORWARD =====
void startForward() {
  state = MOVING_FORWARD;

  motion_start_time = micros();
  lastStepTime = motion_start_time;

  moving = true;

  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(DIR_PIN2, LOW);

  Serial.println("Forward motion started");
}

// ===== STOP =====
void stopAll() {
  moving = false;
  state = STOPPED;

  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);

  Serial.println("Stopped");
}

// ===== PIVOT =====
void startPivot() {
  state = PIVOTING;

  moving = true;
  motion_start_time = micros();
  lastStepTime = motion_start_time;

  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(DIR_PIN2, LOW);

  Serial.println("Pivot started");
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

    // // DEBUG
    // Serial.print("t: ");
    // Serial.print(t, 2);

    // Serial.print(" | speed: ");
    // Serial.print(current_speed_rpm, 2);

    // Serial.print(" | interval: ");
    // Serial.println(interval);
  }

  // STOP automatically after full trapezoid
  if (current_speed_rpm <= 0 && t > (ACCEL_TIME + CRUISE_TIME)) {
    stopAll();
    Serial.println("Motion complete");
  }
}