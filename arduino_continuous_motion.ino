/*
 * Arduino Stepper Controller - Continuous Motion Execution
 * 
 * KEY DIFFERENCE FROM PREVIOUS VERSION:
 * - Executes ONE long trapezoidal profile for TOTAL path distance
 * - NOT per-cell 500mm profiles
 * - Intermediate position reports sent for trajectory verification
 * - No stopping between waypoints
 * 
 * PROTOCOL:
 *   Python sends: Command structure with total_distance_mm and profile parameters
 *   Arduino: Continuous motion with intermediate position updates
 *   Arduino sends: Position updates during motion, "DONE" at end
 *   
 * HARDWARE:
 *   - Stepper: 57am23ed (steps_per_rev = 10000)
 *   - Driver: T60
 *   - Wheel: 117mm diameter
 */

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

const int Step_pin1 = 22;
const int Dir_pin1 = 24;
const int Enb_pin1 = 26;

const int Step_pin2 = 23;
const int Dir_pin2 = 25;
const int Enb_pin2 = 27;

// ============================================================================
// MOTOR & WHEEL PARAMETERS - CALIBRATE FOR YOUR HARDWARE
// ============================================================================

const long Steps_per_rev = 10000;           // Steps per motor revolution
const int Wheel_dia_mm = 117;               // Wheel diameter (mm)
const float Wheel_cir_mm = PI * Wheel_dia_mm;  // Wheel circumference

// ============================================================================
// SERIAL & BAUD RATE
// ============================================================================

#define BAUD_RATE 115200
#define POSITION_REPORT_INTERVAL_MS 100   // Report position every 100ms

// ============================================================================
// STATE MACHINE
// ============================================================================

enum MotionState {
  STATE_IDLE,
  STATE_RECEIVING_CONFIG,
  STATE_RUNNING,
  STATE_STOPPING,
  STATE_COMPLETE,
  STATE_ERROR
};

MotionState current_state = STATE_IDLE;

// ============================================================================
// MOTION PARAMETERS (received from Python)
// ============================================================================

long total_distance_mm = 0;             // Total distance for entire path
float distance_per_cell_mm = 500.0;     // Distance per cell (for reference)
int max_rpm = 60;                       // Maximum RPM
int initial_rpm = 2;                    // Starting RPM
int acc_rpm_per_sec = 25;               // Acceleration rate

// Calculated motion profile
unsigned long total_steps_needed = 0;   // Total steps for total distance
float accel_time_sec = 0.0;
float cruise_time_sec = 0.0;
float decel_time_sec = 0.0;
float profile_peak_rpm = 0.0;
long accel_steps = 0;
long cruise_steps = 0;
long decel_steps = 0;

// ============================================================================
// MOTION EXECUTION VARIABLES
// ============================================================================

unsigned long motion_start_time = 0;
unsigned long last_step_time = 0;
unsigned long last_position_report_time = 0;
long steps_completed = 0;
float current_speed_rpm = 0.0;
unsigned long current_step_interval = 0;
bool stop_requested = false;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

unsigned long calculate_step_interval(float rpm) {
  if (rpm <= 0) return 1000000000;
  return (unsigned long)(60000000.0 / (Steps_per_rev * rpm));
}

long distance_to_steps(float distance_mm) {
  float revolutions = distance_mm / Wheel_cir_mm;
  return (long)(revolutions * Steps_per_rev);
}

float steps_to_distance(long steps) {
  float revolutions = (float)steps / Steps_per_rev;
  return revolutions * Wheel_cir_mm;
}

void calculate_motion_profile() {
  Serial.println("\n[Motion] Calculating trapezoidal profile for continuous motion...");
  Serial.print("  Total distance: ");
  Serial.print(total_distance_mm);
  Serial.println(" mm");
  
  // Convert distance to steps
  total_steps_needed = distance_to_steps(total_distance_mm);
  Serial.print("  Total steps needed: ");
  Serial.println(total_steps_needed);

  profile_peak_rpm = (float)max_rpm;
  
  // Acceleration phase: time to reach max_rpm from initial_rpm
  accel_time_sec = (float)(max_rpm - initial_rpm) / acc_rpm_per_sec;
  float avg_accel_rpm = (float)(initial_rpm + max_rpm) / 2.0;
  accel_steps = (long)((avg_accel_rpm * accel_time_sec * Steps_per_rev) / 60.0);
  
  // Deceleration phase (symmetric)
  decel_time_sec = accel_time_sec;
  decel_steps = accel_steps;
  
  // Cruise phase
  cruise_steps = total_steps_needed - accel_steps - decel_steps;
  
  if (cruise_steps > 0) {
    // Has cruise phase
    cruise_time_sec = (float)cruise_steps / ((max_rpm * Steps_per_rev) / 60.0);
    
    Serial.println("  Trapezoidal profile:");
    Serial.print("    Accel: "); Serial.print(accel_steps); Serial.print(" steps (");
    Serial.print(steps_to_distance(accel_steps), 1); Serial.println(" mm)");
    Serial.print("    Cruise: "); Serial.print(cruise_steps); Serial.print(" steps (");
    Serial.print(steps_to_distance(cruise_steps), 1); Serial.println(" mm)");
    Serial.print("    Decel: "); Serial.print(decel_steps); Serial.print(" steps (");
    Serial.print(steps_to_distance(decel_steps), 1); Serial.println(" mm)");
  } else {
    // Triangle profile (distance too short for cruise)
    Serial.println("  Using triangular profile (distance too short for cruise)");
    
    float peak_rpm_squared = (initial_rpm * initial_rpm) + 
                            ((float)total_distance_mm * acc_rpm_per_sec * 60.0) / Wheel_cir_mm;
    profile_peak_rpm = sqrt(peak_rpm_squared);
    
    accel_time_sec = (profile_peak_rpm - initial_rpm) / acc_rpm_per_sec;
    avg_accel_rpm = (initial_rpm + profile_peak_rpm) / 2.0;
    accel_steps = (long)((avg_accel_rpm * accel_time_sec * Steps_per_rev) / 60.0);
    
    decel_time_sec = accel_time_sec;
    decel_steps = accel_steps;
    cruise_steps = 0;
    cruise_time_sec = 0;
    
    Serial.print("    Peak RPM: "); Serial.println(profile_peak_rpm, 2);
  }
  
  float total_time = accel_time_sec + cruise_time_sec + decel_time_sec;
  Serial.print("  Total execution time: ");
  Serial.print(total_time, 2);
  Serial.println(" seconds");
}

void start_continuous_motion() {
  Serial.println("[Motion] Starting continuous motion...");
  motion_start_time = micros();
  last_step_time = motion_start_time;
  last_position_report_time = millis();
  steps_completed = 0;
  stop_requested = false;
  current_state = STATE_RUNNING;
  
  digitalWrite(Enb_pin1, LOW);   // Enable motors
  digitalWrite(Enb_pin2, LOW);
}

float get_speed_for_step(long completed_steps) {
  float min_running_rpm = max(0.5f, (float)initial_rpm);
  long steps_remaining = (long)total_steps_needed - completed_steps;

  if (accel_steps > 0 && completed_steps < accel_steps) {
    float accel_progress = (float)completed_steps / (float)accel_steps;
    return min_running_rpm + ((profile_peak_rpm - min_running_rpm) * accel_progress);
  }

  if (decel_steps > 0 && steps_remaining <= decel_steps) {
    float decel_progress = 1.0f - ((float)steps_remaining / (float)decel_steps);
    return profile_peak_rpm - ((profile_peak_rpm - min_running_rpm) * decel_progress);
  }

  return profile_peak_rpm;
}

void execute_continuous_motion() {
  // Check if motion is complete
  if (steps_completed >= total_steps_needed) {
    Serial.print("[Motion] ✓ COMPLETE! Total steps: ");
    Serial.print(steps_completed);
    Serial.print(" / ");
    Serial.print(total_steps_needed);
    Serial.print(" (");
    Serial.print(steps_to_distance(steps_completed), 1);
    Serial.println(" mm)");
    
    digitalWrite(Enb_pin1, HIGH);  // Disable motors
    digitalWrite(Enb_pin2, HIGH);
    current_state = STATE_COMPLETE;
    return;
  }
  
  // Check for stop request
  if (stop_requested) {
    Serial.println("[Motion] Stop requested - halting");
    digitalWrite(Enb_pin1, HIGH);
    digitalWrite(Enb_pin2, HIGH);
    current_state = STATE_STOPPING;
    return;
  }
  
  unsigned long now = micros();
  current_speed_rpm = get_speed_for_step(steps_completed);
  
  current_step_interval = calculate_step_interval(current_speed_rpm);
  
  // Generate step pulse
  if ((long)(now - last_step_time) >= (long)current_step_interval) {
    // Step pulse
    PORTA = B00000011;  // Set both step pins HIGH
    delayMicroseconds(5);
    PORTA = B00000000;  // Set both step pins LOW
    
    last_step_time = now;
    steps_completed++;
  }
  
  // Periodic position report (for trajectory monitoring)
  unsigned long now_ms = millis();
  if (now_ms - last_position_report_time >= POSITION_REPORT_INTERVAL_MS) {
    float distance_completed = steps_to_distance(steps_completed);
    float percent_complete = (100.0 * steps_completed) / total_steps_needed;
    
    Serial.print("[POS]");
    Serial.print(" d="); Serial.print(distance_completed, 1);
    Serial.print("mm");
    Serial.print(" p="); Serial.print(percent_complete, 1);
    Serial.print("%");
    Serial.print(" s="); Serial.print(steps_completed);
    Serial.print("/"); Serial.print(total_steps_needed);
    Serial.print(" rpm="); Serial.println(current_speed_rpm, 1);
    
    last_position_report_time = now_ms;
  }
}

// ============================================================================
// SERIAL COMMAND PARSING
// ============================================================================

void parse_start_command() {
  /*
   * Command format from Python:
   * Full: START:2000:500:60:2:25
   * (The 'S' is already consumed by the main loop)
   * So we read: TART:2000:500:60:2:25
   * 
   * But we need to skip "TART:" and parse only the numbers:
   * 2000:500:60:2:25
   */
  
  // Read until newline
  String command = "";
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') break;
    command += ch;
  }
  
  if (command.length() == 0) {
    Serial.println("[ERROR] Empty command - no data after 'S'");
    return;
  }
  
  // DEBUG: Print what was received
  Serial.print("[DEBUG] Raw data after 'S': '");
  Serial.print(command);
  Serial.println("'");
  
  // Skip "TART:" if present (part of "START:")
  String params = command;
  if (command.startsWith("TART:")) {
    params = command.substring(5);  // Skip "TART:"
  }
  
  Serial.print("[DEBUG] Parsing params: '");
  Serial.print(params);
  Serial.println("'");
  
  // Parse command by splitting on ':'
  int values[5] = {0};
  int value_count = 0;
  String current_value = "";
  
  for (int i = 0; i < params.length(); i++) {
    char ch = params[i];
    if (ch == ':') {
      if (value_count < 5) {
        values[value_count] = current_value.toInt();
        Serial.print("[DEBUG] Param ");
        Serial.print(value_count);
        Serial.print(": ");
        Serial.println(values[value_count]);
        value_count++;
        current_value = "";
      }
    } else {
      current_value += ch;
    }
  }
  
  // Last value
  if (current_value.length() > 0 && value_count < 5) {
    values[value_count] = current_value.toInt();
    Serial.print("[DEBUG] Param ");
    Serial.print(value_count);
    Serial.print(": ");
    Serial.println(values[value_count]);
    value_count++;
  }
  
  if (value_count != 5) {
    Serial.print("[ERROR] Invalid command format. Got ");
    Serial.print(value_count);
    Serial.println(" values, expected 5");
    return;
  }
  
  // Set parameters
  total_distance_mm = values[0];
  distance_per_cell_mm = values[1];
  max_rpm = values[2];
  initial_rpm = values[3];
  acc_rpm_per_sec = values[4];
  
  Serial.println("[CONFIG] Parameters received:");
  Serial.print("  Total distance: "); Serial.print(total_distance_mm); Serial.println(" mm");
  Serial.print("  Max RPM: "); Serial.println(max_rpm);
  Serial.print("  Initial RPM: "); Serial.println(initial_rpm);
  Serial.print("  Acceleration: "); Serial.print(acc_rpm_per_sec); Serial.println(" RPM/s");
  
  // Calculate profile
  calculate_motion_profile();
  
  // Start motion
  start_continuous_motion();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000);
  
  Serial.println("\n[STARTUP] AGV Continuous Motion Controller");
  Serial.println("[PROTO] Expecting: START:<total_dist>:<cell_dist>:<max_rpm>:<init_rpm>:<acc_rate>");
  Serial.println("[READY] Waiting for command...");
  
  // Configure GPIO
  pinMode(Step_pin1, OUTPUT);
  pinMode(Dir_pin1, OUTPUT);
  pinMode(Enb_pin1, OUTPUT);
  pinMode(Step_pin2, OUTPUT);
  pinMode(Dir_pin2, OUTPUT);
  pinMode(Enb_pin2, OUTPUT);
  
  // Initial state: motors disabled
  digitalWrite(Step_pin1, LOW);
  digitalWrite(Step_pin2, LOW);
  digitalWrite(Dir_pin1, HIGH);   // Forward by default
  digitalWrite(Dir_pin2, HIGH);
  digitalWrite(Enb_pin1, HIGH);
  digitalWrite(Enb_pin2, HIGH);
  
  current_state = STATE_IDLE;
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.peek();
    
    if (current_state == STATE_IDLE) {
      if (cmd == 'S') {
        // START command
        Serial.read();  // consume 'S'
        delay(50);      // Give time for rest of command to arrive
        parse_start_command();
      } else if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R') {
        // Direction command
        char dir = Serial.read();
        
        // Clear any trailing whitespace/newlines
        while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r' || Serial.peek() == ' ')) {
          Serial.read();
        }
        
        Serial.print("[DIR] ");
        Serial.println(dir);
        
        if (dir == 'F') {
          digitalWrite(Dir_pin1, HIGH);
          digitalWrite(Dir_pin2, HIGH);
        } else if (dir == 'B') {
          digitalWrite(Dir_pin1, LOW);
          digitalWrite(Dir_pin2, LOW);
        } else if (dir == 'L') {
          digitalWrite(Dir_pin1, HIGH);
          digitalWrite(Dir_pin2, LOW);
        } else if (dir == 'R') {
          digitalWrite(Dir_pin1, LOW);
          digitalWrite(Dir_pin2, HIGH);
        }
        delay(50);  // Give time to settle before next command
      } else if (cmd == '\n' || cmd == '\r' || cmd == ' ' || cmd == '\t') {
        Serial.read();
      } else {
        // Unknown command - ignore and remove
        char unknown = Serial.read();
        Serial.print("[IGNORE] Unknown character: '");
        Serial.print(unknown);
        Serial.println("'");
      }
    } else if (current_state == STATE_RUNNING) {
      char cmd = Serial.read();
      if (cmd == 'X') {
        // Emergency stop
        Serial.println("[EMERGENCY] Stop requested");
        stop_requested = true;
      }
    }
  }
  
  // Execute motion
  switch (current_state) {
    case STATE_RUNNING:
      execute_continuous_motion();
      break;
    
    case STATE_STOPPING:
      delay(100);  // Let motors settle
      Serial.println("DONE");
      current_state = STATE_IDLE;
      break;
    
    case STATE_COMPLETE:
      delay(100);
      Serial.println("DONE");
      current_state = STATE_IDLE;
      break;
    
    case STATE_IDLE:
      // Idle - do nothing
      delay(1);
      break;
  }
}

// ============================================================================
// END OF SKETCH
// ============================================================================
