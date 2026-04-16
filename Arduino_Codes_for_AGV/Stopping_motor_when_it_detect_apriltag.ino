const int stepPin1 = 22;
const int dirPin1  = 24;
const int enPin1   = 26;

const int stepPin2 = 23;
const int dirPin2  = 25;
const int enPin2   = 27;

char current_cmd = 's';   // start stopped

void setup(){
  Serial.begin(9600);

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enPin2, OUTPUT);

  digitalWrite(enPin1, LOW);  // enable drivers
  digitalWrite(enPin2, LOW);
}

void loop(){
  if (Serial.available()) {
    current_cmd = Serial.read();
  }

  if (current_cmd == 'f') {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    stepMotors();
  } else if (current_cmd == 'b') {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    stepMotors();
  } else if (current_cmd == 's') {
    // stop: do nothing
  }
}

void stepMotors(){
  digitalWrite(stepPin1, HIGH);
  digitalWrite(stepPin2, HIGH);
  delayMicroseconds(500);
  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);
  delayMicroseconds(500);
}
