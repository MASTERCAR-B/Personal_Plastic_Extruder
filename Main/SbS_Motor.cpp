// Fuck useless libraries, imma do it my own way
const int stepper1_stp = 45;
const int stepper1_dir = 47;
const int stepper1_ena = 43;
const int stepper2_stp = 23;
const int stepper2_dir = 41;
const int stepper2_ena = 37;
const int endstop1 = 23;
const int endstop2 = 25;

unsigned long lastStep1Time = 0;
unsigned long lastStep2Time = 0;
unsigned long stepDelay = 500; 
bool stepper2Direction = true;

void setup() {
  pinMode(stepper1_stp, OUTPUT);
  pinMode(stepper1_dir, OUTPUT);
  pinMode(stepper1_ena, OUTPUT);
  pinMode(stepper2_stp, OUTPUT);
  pinMode(stepper2_dir, OUTPUT);
  pinMode(stepper2_ena, OUTPUT);
  pinMode(endstop1, INPUT_PULLUP);
  pinMode(endstop2, INPUT_PULLUP);
  
  digitalWrite(stepper1_ena, LOW);  
  digitalWrite(stepper2_ena, LOW);
  digitalWrite(stepper1_dir, HIGH); 
  digitalWrite(stepper2_dir, HIGH);
}

void loop() {
  unsigned long currentTime = millis();
  
  if(currentTime - lastStep1Time >= stepDelay) {
    digitalWrite(stepper1_stp, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepper1_stp, LOW);
    lastStep1Time = currentTime;
  }
  
  if(currentTime - lastStep2Time >= stepDelay) {
    if(digitalRead(endstop1) == LOW || digitalRead(endstop2) == LOW) {
      stepper2Direction = !stepper2Direction;
      digitalWrite(stepper2_dir, stepper2Direction ? HIGH : LOW);
      delay(10);
    }
    
    digitalWrite(stepper2_stp, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepper2_stp, LOW);
    lastStep2Time = currentTime;
  }
}
