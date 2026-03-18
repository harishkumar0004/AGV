/*
  AGV Motor Controller - Stage 1

  Raspberry Pi (master):
  - Detect Tag 1 -> send 'f'
  - Detect Tag 2 -> send 'p'

  Arduino Mega (slave):
  - 'f' -> move forward continuously
  - 'p' -> pivot 180 degrees in place, then stop automatically
  - 's' -> immediate stop
*/

// ===== MOTOR PINS =====
const int STEP_PIN1 = 22;
const int DIR_PIN1 = 24;
const int EN_PIN1 = 26;

const int STEP_PIN2 = 23;
const int DIR_PIN2 = 25;
const int EN_PIN2 = 27;

// ===== TIMING =====
const unsigned long STEP_INTERVAL = 50;  // 50 microseconds

volatile unsigned long lastStepTime1 = 0;
volatile unsigned long lastStepTime2 = 0;

bool stepState1 = LOW;
bool stepState2 = LOW;

// ===== STATE VARIABLES =====
bool moving = false;

// ===== STATE MACHINE =====
enum AGVState {
  STOPPED,
  MOVING_FORWARD,
  PIVOTING_180,
  STAGE_COMPLETE
};

AGVState agv_state = STOPPED;
unsigned long state_start_time = 0;

// Timing for movements
const unsigned long PIVOT_DURATION = 4000;  // Tune this on the floor for a true 180 turn.

// ===== COUNTERS =====
unsigned long total_commands = 0;
unsigned long total_steps1 = 0;
unsigned long total_steps2 = 0;

// ===== DEBUG FUNCTIONS =====
void debug_print(const char* msg) {
  Serial.print("[");
  Serial.print(millis() / 1000);
  Serial.print("s] ");
  Serial.println(msg);
}

void debug_state(const char* state_name, const char* action) {
  Serial.print("[");
  Serial.print(millis() / 1000);
  Serial.print("s] STATE: ");
  Serial.print(state_name);
  Serial.print(" | ");
  Serial.println(action);
}

// ===== SETUP =====
void setup() {
  Serial.begin(9600);
  delay(1000);
  
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(EN_PIN1, OUTPUT);
  
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(EN_PIN2, OUTPUT);
  
  digitalWrite(EN_PIN1, LOW);
  digitalWrite(EN_PIN2, LOW);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(DIR_PIN2, LOW);
  
  Serial.println("\n" + String(70, '='));
  Serial.println("AGV STAGE 1 - Pivot 180° Rotation");
  Serial.println("Tag 1: Forward | Tag 2: Pivot 180° -> Stop");
  Serial.println(String(70, '='));
  Serial.println("Commands:");
  Serial.println("  f: Move forward continuously");
  Serial.println("  p: Pivot 180 degrees and stop");
  Serial.println("  s: Stop immediately");
  Serial.println(String(70, '=') + "\n");
  
  debug_print("System initialized - STOPPED");
  agv_state = STOPPED;
}

// ===== MAIN LOOP =====
void loop() {
  // Read serial commands
  while (Serial.available()) {
    char incoming = Serial.read();

    if (incoming == '\n' || incoming == '\r' || incoming == ' ') {
      continue;
    }

    total_commands++;
    Serial.print("[CMD] ");
    Serial.println(incoming);
    executeCommand(incoming);
  }
  
  // Update state machine
  updateAGVState();
  
  // Generate step pulses
  if (moving) {
    stepMotors();
  }
}

// ===== COMMAND EXECUTION =====
void executeCommand(char cmd) {
  switch (cmd) {
    case 'f':
      moveForward();
      break;
      
    case 'p':
      pivot180Degrees();
      break;
      
    case 's':
      stopMotors();
      break;
      
    case '?':
      printStatus();
      break;
      
    default:
      Serial.print("Unknown: ");
      Serial.println(cmd);
      break;
  }
}

// ===== MOVEMENT FUNCTIONS =====

void moveForward() {
  if (agv_state != MOVING_FORWARD) {
    agv_state = MOVING_FORWARD;
    state_start_time = millis();
    
    moving = true;
    stepState1 = LOW;
    stepState2 = LOW;
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);
    lastStepTime1 = micros();
    lastStepTime2 = micros();
    
    digitalWrite(DIR_PIN1, HIGH);  // Forward
    digitalWrite(DIR_PIN2, LOW);   // Forward
    
    debug_state("MOVING_FORWARD", "Tag 1 detected - Moving forward");
    Serial.println("  Motor 1: Forward (full speed)");
    Serial.println("  Motor 2: Forward (full speed)");
  }
}

void pivot180Degrees() {
  /*
    TRUE PIVOT 180° ROTATION - In Place
    Motor 1 (Left): BACKWARD (rotate wheel backward)
    Motor 2 (Right): FORWARD (rotate wheel forward)
    Result: AGV rotates 180° in place without moving laterally
  */
  if (agv_state != PIVOTING_180) {
    agv_state = PIVOTING_180;
    state_start_time = millis();
    
    moving = true;
    stepState1 = LOW;
    stepState2 = LOW;
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);
    lastStepTime1 = micros();
    lastStepTime2 = micros();
    
    // CRITICAL: Opposite directions for pivot
    digitalWrite(DIR_PIN1, LOW);   // Motor 1: BACKWARD
    digitalWrite(DIR_PIN2, HIGH);  // Motor 2: FORWARD
    
    debug_state("PIVOTING_180", "Executing in-place 180° pivot");
    Serial.println("  Motor 1 (Left):  BACKWARD");
    Serial.println("  Motor 2 (Right): FORWARD");
    Serial.println("  Result: True pivot rotation in place");
    Serial.print("  Duration: ");
    Serial.print(PIVOT_DURATION);
    Serial.println(" ms");
  }
}

void stopMotors() {
  moving = false;
  stepState1 = LOW;
  stepState2 = LOW;
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  
  debug_state("STOPPED", "Motors halted");
  agv_state = STOPPED;
}

void completePivotAndStop() {
  moving = false;
  stepState1 = LOW;
  stepState2 = LOW;
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);

  debug_state("STAGE_COMPLETE", "180° pivot completed -> stopped");
  Serial.println("  AGV now faces the opposite direction");
  Serial.println("  Motors are stopped");

  agv_state = STAGE_COMPLETE;
}

// ===== STATE MACHINE UPDATE =====
void updateAGVState() {
  unsigned long elapsed = millis() - state_start_time;
  
  switch (agv_state) {
    case PIVOTING_180:
      if (elapsed >= PIVOT_DURATION) {
        completePivotAndStop();
      }
      break;
      
    default:
      break;
  }
}

// ===== STEPPING CONTROL =====
void stepMotors() {
  unsigned long now = micros();
  
  // Motor 1
  if (now - lastStepTime1 >= STEP_INTERVAL) {
    lastStepTime1 = now;
    stepState1 = !stepState1;
    digitalWrite(STEP_PIN1, stepState1);
    
    if (stepState1 == LOW) {
      total_steps1++;
    }
  }
  
  // Motor 2
  if (now - lastStepTime2 >= STEP_INTERVAL) {
    lastStepTime2 = now;
    stepState2 = !stepState2;
    digitalWrite(STEP_PIN2, stepState2);
    
    if (stepState2 == LOW) {
      total_steps2++;
    }
  }
}

// ===== STATUS =====
void printStatus() {
  Serial.println("\n" + String(70, '='));
  Serial.println("SYSTEM STATUS");
  Serial.println(String(70, '='));
  
  Serial.print("Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  
  Serial.print("Commands: ");
  Serial.println(total_commands);
  
  Serial.print("Motor 1 steps: ");
  Serial.println(total_steps1);
  
  Serial.print("Motor 2 steps: ");
  Serial.println(total_steps2);
  
  Serial.print("State: ");
  switch (agv_state) {
    case STOPPED: Serial.println("STOPPED"); break;
    case MOVING_FORWARD: Serial.println("MOVING_FORWARD"); break;
    case PIVOTING_180: Serial.println("PIVOTING_180"); break;
    case STAGE_COMPLETE: Serial.println("STAGE_COMPLETE"); break;
    default: Serial.println("UNKNOWN");
  }
  
  Serial.println(String(70, '=') + "\n");
}
