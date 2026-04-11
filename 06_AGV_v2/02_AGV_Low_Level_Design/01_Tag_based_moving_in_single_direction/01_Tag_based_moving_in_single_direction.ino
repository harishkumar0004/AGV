const int Step_pin1 = 22;
const int Dir_pin1  = 24;
const int Enb_pin1  = 26;

const int Step_pin2 = 23;
const int Dir_pin2  = 25;
const int Enb_pin2  = 27;

const long Steps_per_rev = 10000;

const int Max_rpm = 100;
const int Initial_rpm = 2;
const int Acc_rpm_per_sec = 10;

const int Wheel_dia = 117;
const float Wheel_cir = PI * Wheel_dia;

long Total_dist = 0;

unsigned long Start_time = 0;
unsigned long last_step_time1 = 0;

bool motion_running = false;
bool motion_complete = false;

unsigned long current_step_interval = 0;
long steps_completed = 0;
float current_speed_rpm = 0;

float accel_time_sec = 0.0;
float cruise_time_sec = 0.0;
float decel_time_sec = 0.0;

long accel_steps = 0;
long cruise_steps = 0;
long decel_steps = 0;

String input = "";

unsigned long calculate_step_interval(float rpm) {
  if (rpm <= 0) return 1000000000;

  unsigned long interval = (unsigned long)(60000000.0 / (Steps_per_rev * rpm));
  // if (interval < 50) interval = 50;

  return interval;
}

// ================= MOTION PROFILE =================
void calculate_motion_profile() {

  accel_time_sec = (float)(Max_rpm - Initial_rpm) / Acc_rpm_per_sec;

  float avg_accel_speed = (float)(Initial_rpm + Max_rpm) / 2.0;
  accel_steps = (long)((avg_accel_speed * accel_time_sec * Steps_per_rev) / 60.0);

  decel_time_sec = accel_time_sec;
  decel_steps = accel_steps;

  cruise_steps = Total_dist - accel_steps - decel_steps;

  if (cruise_steps > 0) {
    cruise_time_sec = (float)cruise_steps / ((Max_rpm * Steps_per_rev) / 60.0);
  } else {
    cruise_steps = 0;
    cruise_time_sec = 0;
  }
}

float get_current_speed(float elapsed_sec) {

  if (elapsed_sec <= accel_time_sec) {
    return Initial_rpm + (Acc_rpm_per_sec * elapsed_sec);
  }
  else if (elapsed_sec <= (accel_time_sec + cruise_time_sec)) {
    return Max_rpm;
  }
  else {
    float decel_elapsed = elapsed_sec - (accel_time_sec + cruise_time_sec);
    float speed = Max_rpm - (Acc_rpm_per_sec * decel_elapsed);

    if (speed < 0) speed = 0;
    return speed;
  }
}

void Start_motion() {

  Start_time = micros();
  last_step_time1 = Start_time;

  steps_completed = 0;
  motion_complete = false;
  motion_running = true;

  digitalWrite(Enb_pin1, LOW);
  digitalWrite(Enb_pin2, LOW);
}


void execute_motion() {

  unsigned long now = micros();
  float elapsed_sec = (now - Start_time) / 1000000.0;

  current_speed_rpm = get_current_speed(elapsed_sec);
  current_step_interval = calculate_step_interval(current_speed_rpm);

  if (now - last_step_time1 >= current_step_interval) {

    PORTA = B00000011;
    delayMicroseconds(5);
    PORTA = B00000000;

    last_step_time1 = now;
    steps_completed++;

    if (steps_completed >= Total_dist) {
      Stop_motion();
      Serial.println("EVT:DONE");
    }
  }
}


void Stop_motion() {

  motion_running = false;
  motion_complete = true;

  digitalWrite(Enb_pin1, HIGH);
  digitalWrite(Enb_pin2, HIGH);
}

void processCommand(String cmd) {

  cmd.trim();

  if (cmd.startsWith("CMD:FWD:")) {

    long distance = cmd.substring(8).toInt();

    if (distance <= 0) {
      Serial.println("EVT:ERROR");
      return;
    }

    Serial.println("ACK:FWD");

    float rev = distance / Wheel_cir;
    Total_dist = (long)(rev * Steps_per_rev);

    calculate_motion_profile();
    Start_motion();
    return;
  }

  if (cmd == "CMD:STOP") {
    Stop_motion();
    Serial.println("ACK:STOP");
    return;
  }

  Serial.println("EVT:UNKNOWN");
}

void setup() {

  Serial.begin(115200);

  pinMode(Step_pin1, OUTPUT);
  pinMode(Dir_pin1, OUTPUT);
  pinMode(Enb_pin1, OUTPUT);

  pinMode(Step_pin2, OUTPUT);
  pinMode(Dir_pin2, OUTPUT);
  pinMode(Enb_pin2, OUTPUT);

  digitalWrite(Step_pin1, LOW);
  digitalWrite(Step_pin2, LOW);

  digitalWrite(Dir_pin1, HIGH);
  digitalWrite(Dir_pin2, LOW);

  digitalWrite(Enb_pin1, HIGH);
  digitalWrite(Enb_pin2, HIGH);
}

void loop() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {
      processCommand(input);
      input = "";
    } else {
      input += c;
    }
  }

  if (motion_running && !motion_complete) {
    execute_motion();
  }
}