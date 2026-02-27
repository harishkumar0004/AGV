/*
 * Arduino Slave Controller - AGV Motion Execution
 * 
 * CRITICAL: This code must send "DONE\n" after each command completes.
 * Do NOT send any startup messages.
 * Baud rate MUST be 9600 to match Python
 * 
 * Protocol:
 *   Python sends: 'F' (or 'B', 'L', 'R') 
 *   Arduino executes for specified time
 *   Arduino sends: "DONE\n"
 *   Repeat
 */

// ============================================================================
// CONFIGURATION - COMMUNICATION-ONLY MODE (NO HARDWARE)
// ============================================================================
// This sketch is intentionally hardware-free. It only implements the
// command/ack timing protocol for Raspberry Pi testing.

// Timing parameters (milliseconds) - CALIBRATE FOR YOUR ROBOT
#define MOVE_DURATION_MS     2000   // Time to move 1 cell forward/backward
#define TURN_DURATION_MS     800    // Time to rotate 90 degrees
#define MOTOR_STOP_DELAY_MS  100    // Pause before sending ACK

// Serial
#define BAUD_RATE       9600    // MUST match Python (9600)

// ============================================================================
// STATE MACHINE & VARIABLES
// ============================================================================

enum State {
  STATE_IDLE,           // Waiting for command
  STATE_EXECUTING,      // Command in progress
  STATE_STOPPING,       // Motor stopping phase
  STATE_SENDING_ACK     // About to send "DONE"
};

State current_state = STATE_IDLE;
char received_cmd = '\0';
unsigned long exec_start_time = 0;
unsigned long last_command_time = 0;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);  // CRITICAL: Must be 9600 for Python communication

  last_command_time = millis();
  
  // DO NOT SEND ANYTHING HERE - Python expects silent startup!
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Check for new command from Python
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Ignore line endings and whitespace from host tools/terminals
    if (cmd == '\n' || cmd == '\r' || cmd == ' ' || cmd == '\t') {
      return;
    }

    // Only accept valid commands when idle
    if (current_state == STATE_IDLE) {
      if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R') {
        received_cmd = cmd;
        exec_start_time = millis();
        current_state = STATE_EXECUTING;
        dispatch_command(cmd);
        last_command_time = millis();
      } else {
        // Unknown command: ignore without ACK to avoid desync
      }
    }
  }
  
  // Process state machine
  switch (current_state) {
    case STATE_IDLE:
      // Do nothing, wait for command
      break;
      
    case STATE_EXECUTING:
      // Check if execution time expired
      if (millis() - exec_start_time >= get_duration_ms(received_cmd)) {
        // Time to stop motors
        stop_all_motors();
        current_state = STATE_STOPPING;
        exec_start_time = millis();
      }
      break;
      
    case STATE_STOPPING:
      // Wait for motors to settle
      if (millis() - exec_start_time >= MOTOR_STOP_DELAY_MS) {
        // Send acknowledgment
        Serial.println("DONE");
        current_state = STATE_IDLE;
      }
      break;
      
    case STATE_SENDING_ACK:
      current_state = STATE_IDLE;
      break;
  }
}

// ============================================================================
// COMMAND DISPATCH
// ============================================================================

void dispatch_command(char cmd) {
  // Communication-only: no hardware actions.
  // We still honor timing based on the command.
  (void)cmd;
}

unsigned long get_duration_ms(char cmd) {
  if (cmd == 'F' || cmd == 'B') {
    return MOVE_DURATION_MS;
  } else if (cmd == 'L' || cmd == 'R') {
    return TURN_DURATION_MS;
  }
  return 0;
}

// ============================================================================
// MOVEMENT FUNCTIONS
// ============================================================================

void stop_all_motors() {
  // Communication-only: nothing to stop.
}

// ============================================================================
// END OF ARDUINO SKETCH
// ============================================================================
