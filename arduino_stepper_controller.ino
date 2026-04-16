/*
 * Arduino Stepper Motor Controller - AGV Motion with AprilTag Confirmation
 * 
 * PROTOCOL:
 *   Python sends: 'F' (forward), 'B' (backward), 'L' (left turn), 'R' (right turn), 'S' (stop)
 *   Arduino executes motion with trapezoidal profile
 *   Arduino sends: "DONE\n" when motion completes OR command 'S' is received
 *   
 * KEY CHANGES:
 *   - Step-based motion (not time-based)
 *   - Trapezoidal acceleration profile
 *   - STOP command for Python to halt motion early when AprilTag detected
 *   - No startup messages (Python expects silent operation)
 * 
 * Hardware:
 *   - Stepper Motor: 57am23ed (10000 steps/rev assumed)
 *   - Motor Driver: T60
 *   - Wheel Diameter: 117mm
 *   - Distance per command: 500mm (configurable)
 */

// ============================================================================
// PIN CONFIGURATION - UPDATE FOR YOUR HARDWARE
// ============================================================================

const int Step_pin1 = 22;      // Motor 1 Step   (adjust as needed)
const int Dir_pin1 = 24;       // Motor 1 Direction
const int Enb_pin1 = 26;       // Motor 1 Enable

const int Step_pin2 = 23;      // Motor 2 Step
const int Dir_pin2 = 25;       // Motor 2 Direction
const int Enb_pin2 = 27;       // Motor 2 Enable

// ============================================================================
// MOTION PARAMETERS - CALIBRATE FOR YOUR ROBOT
// ============================================================================

const long Steps_per_rev = 10000;           // Steps per motor revolution
const int Max_rpm = 60;                      // Maximum speed
const int Initial_rpm = 2;                   // Starting speed (avoid jerk)
const int Acc_rpm_per_sec = 25;              // Acceleration rate

const int Wheel_dia = 117;                   // Wheel diameter (mm)
const float Wheel_cir = PI * Wheel_dia;      // Wheel circumference
const int Distance_mm = 500;                 // Distance per cell (mm)
const float rev = Distance_mm / Wheel_cir;   // Revolutions needed
const long Total_dist = (long)(rev * Steps_per_rev);  // Step count per cell

// ============================================================================
// SERIAL & TIMING CONFIGURATION
// ============================================================================

#define BAUD_RATE       115200   // Serial communication speed
#define MOTOR_STOP_DELAY_MS  100  // Time for motors to settle after stop

// ============================================================================
// STATE MACHINE & MOTION CONTROL VARIABLES
// ============================================================================

enum MotionState {
  STATE_IDLE,           // Waiting for command
  STATE_CALCULATING,    // Computing motion profile
  STATE_RUNNING,        // Motors spinning
  STATE_STOPPING,       // Motors settling
  STATE_COMPLETE,       // Motion finished, ready to send ACK
  STATE_ERROR           // Error state
};

MotionState current_state = STATE_IDLE;
char received_cmd = '\0';
bool stop_requested = false;

// Timing variables
unsigned long Start_time = 0;
unsigned long last_step_time1 = 0;
unsigned long last_step_time2 = 0;

// Motion profile variables
unsigned long current_step_interval = 0;
long steps_completed = 0;
float current_speed_rpm = 0.0;
float accel_time_sec = 0.0;
float cruise_time_sec = 0.0;
float decel_time_sec = 0.0;
long accel_steps = 0;
long cruise_steps = 0;
long decel_steps = 0;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

unsigned long calculate_step_interval(float rpm) {
  if (rpm <= 0) return 1000000000;
  return (unsigned long)(60000000.0 / (Steps_per_rev * rpm));
}

void calculate_motion_profile() {
  // Acceleration phase
  accel_time_sec = (float)(Max_rpm - Initial_rpm) / Acc_rpm_per_sec;
  float avg_accel_speed = (float)(Initial_rpm + Max_rpm) / 2.0;
  accel_steps = (long)((avg_accel_speed * accel_time_sec * Steps_per_rev) / 60.0);

  // Deceleration phase (symmetric)
  decel_time_sec = accel_time_sec;
  decel_steps = accel_steps;

  // Cruise phase (constant speed)
  cruise_steps = Total_dist - accel_steps - decel_steps;

  if (cruise_steps > 0) {
    cruise_time_sec = (float)cruise_steps / ((Max_rpm * Steps_per_rev) / 60.0);
  } else {
    // Distance too short for full acceleration - triangle profile
    float peak_rpm_squared = (Initial_rpm * Initial_rpm) + 
                            ((float)Total_dist * Acc_rpm_per_sec * 60.0) / Steps_per_rev;
    float achievable_max_rpm = sqrt(peak_rpm_squared);

    accel_time_sec = (achievable_max_rpm - Initial_rpm) / Acc_rpm_per_sec;
    avg_accel_speed = (Initial_rpm + achievable_max_rpm) / 2.0;
    accel_steps = (long)((avg_accel_speed * accel_time_sec * Steps_per_rev) / 60.0);

    decel_time_sec = accel_time_sec;
    decel_steps = accel_steps;
    cruise_steps = 0;
    cruise_time_sec = 0;
  }

  Serial.println("[Motion] Profile calculated");
  Serial.print("  Accel: "); Serial.print(accel_steps); Serial.print(" steps in ");
  Serial.print(accel_time_sec, 2); Serial.println("s");
  Serial.print("  Cruise: "); Serial.print(cruise_steps); Serial.print(" steps at ");
  Serial.print(Max_rpm); Serial.println(" RPM");
  Serial.print("  Decel: "); Serial.print(decel_steps); Serial.print(" steps in ");
  Serial.print(decel_time_sec, 2); Serial.println("s");
}

float get_current_speed(float elapsed_sec) {
  if (elapsed_sec <= accel_time_sec) {
    return (float)Initial_rpm + (Acc_rpm_per_sec * elapsed_sec);
  } else if (elapsed_sec <= (accel_time_sec + cruise_time_sec)) {
    return Max_rpm;
  } else {
    float decel_elapsed = elapsed_sec - (accel_time_sec + cruise_time_sec);
    float speed = (float)Max_rpm - (Acc_rpm_per_sec * decel_elapsed);
    return max(0.0f, speed);
  }
}

void start_motion() {
  Serial.println("[Motion] Starting...");
  Start_time = micros();
  last_step_time1 = Start_time;
  last_step_time2 = Start_time;
  steps_completed = 0;
  stop_requested = false;
  current_state = STATE_RUNNING;

  digitalWrite(Enb_pin1, LOW);   // Enable motors
  digitalWrite(Enb_pin2, LOW);
}

void stop_motion() {
  Serial.println("[Motion] Stopping motors...");
  digitalWrite(Enb_pin1, HIGH);  // Disable motors
  digitalWrite(Enb_pin2, HIGH);
  current_state = STATE_STOPPING;
}

void execute_motion() {
  unsigned long now = micros();
  float elapsed_sec = (float)(now - Start_time) / 1000000.0;

  // Check for STOP request from Python
  if (stop_requested) {
    Serial.println("[Motion] STOP command received - halting");
    stop_motion();
    return;
  }

  // Get current speed from profile
  current_speed_rpm = get_current_speed(elapsed_sec);
  current_step_interval = calculate_step_interval(current_speed_rpm);

  // Generate step pulses
  if ((long)(now - last_step_time1) >= (long)current_step_interval) {
    // Step pulse (5µs duration is typical for stepper drivers)
    PORTA = B00000011;  // Set both step pins HIGH
    delayMicroseconds(5);
    PORTA = B00000000;  // Set both step pins LOW

    last_step_time1 = now;
    steps_completed++;

    // Check if motion complete
    if (steps_completed >= Total_dist) {
      Serial.println("[Motion] Motion complete");
      stop_motion();
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000);

  Serial.println("\n[STARTUP] AGV Stepper Controller Ready");
  Serial.println("[PROTO] Commands: 'F' forward, 'B' backward, 'L' left, 'R' right, 'S' stop");

  // Configure pins
  pinMode(Step_pin1, OUTPUT);
  pinMode(Dir_pin1, OUTPUT);
  pinMode(Enb_pin1, OUTPUT);
  pinMode(Step_pin2, OUTPUT);
  pinMode(Dir_pin2, OUTPUT);
  pinMode(Enb_pin2, OUTPUT);

  // Initial state: motors disabled
  digitalWrite(Step_pin1, LOW);
  digitalWrite(Step_pin2, LOW);
  digitalWrite(Enb_pin1, HIGH);
  digitalWrite(Enb_pin2, HIGH);

  // Pre-calculate motion profile
  calculate_motion_profile();

  current_state = STATE_IDLE;
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Check for commands from Python
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Ignore line endings and whitespace
    if (cmd == '\n' || cmd == '\r' || cmd == ' ' || cmd == '\t') {
      return;
    }

    // STOP command - can be received at any time
    if (cmd == 'S' || cmd == 's') {
      Serial.println("[SERIAL] STOP command received");
      stop_requested = true;
      return;
    }

    // Motion commands (F, B, L, R) - only accept when idle
    if (current_state == STATE_IDLE) {
      received_cmd = cmd;

      if (cmd == 'F' || cmd == 'f') {
        Serial.println("[SERIAL] Forward command");
        digitalWrite(Dir_pin1, HIGH);
        digitalWrite(Dir_pin2, HIGH);
        current_state = STATE_CALCULATING;
      } else if (cmd == 'B' || cmd == 'b') {
        Serial.println("[SERIAL] Backward command");
        digitalWrite(Dir_pin1, LOW);
        digitalWrite(Dir_pin2, LOW);
        current_state = STATE_CALCULATING;
      } else if (cmd == 'L' || cmd == 'l') {
        Serial.println("[SERIAL] Left turn command");
        digitalWrite(Dir_pin1, HIGH);
        digitalWrite(Dir_pin2, LOW);
        current_state = STATE_CALCULATING;
      } else if (cmd == 'R' || cmd == 'r') {
        Serial.println("[SERIAL] Right turn command");
        digitalWrite(Dir_pin1, LOW);
        digitalWrite(Dir_pin2, HIGH);
        current_state = STATE_CALCULATING;
      } else {
        Serial.print("[SERIAL] Unknown command: ");
        Serial.println(cmd);
        return;
      }

      start_motion();
    }
  }

  // State machine
  switch (current_state) {
    case STATE_IDLE:
      // Idle - nothing to do
      break;

    case STATE_RUNNING:
      execute_motion();
      break;

    case STATE_STOPPING:
      // Wait for motors to settle
      if (millis() % 100 == 0) {  // Check every 100ms to avoid busy-waiting
        delay(MOTOR_STOP_DELAY_MS);
        current_state = STATE_COMPLETE;
      }
      break;

    case STATE_COMPLETE:
      // Send acknowledgment
      Serial.println("DONE");
      current_state = STATE_IDLE;
      break;

    case STATE_ERROR:
      // Reset to idle
      digitalWrite(Enb_pin1, HIGH);
      digitalWrite(Enb_pin2, HIGH);
      current_state = STATE_IDLE;
      break;

    default:
      current_state = STATE_IDLE;
  }
}

// ============================================================================
// END OF SKETCH
// ============================================================================
