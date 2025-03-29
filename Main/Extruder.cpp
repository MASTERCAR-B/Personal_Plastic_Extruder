/*
    ______     __                                          __        ____  __               __  _          
   / ____/  __/ /________  ___________  _________ _   ____/ /__     / __ \/ /   ____ ______/ /_(_)________ 
  / __/ | |/_/ __/ ___/ / / / ___/ __ \/ ___/ __ `/  / __  / _ \   / /_/ / /   / __ `/ ___/ __/ / ___/ __ \
 / /____>  </ /_/ /  / /_/ (__  ) /_/ / /  / /_/ /  / /_/ /  __/  / ____/ /___/ /_/ (__  ) /_/ / /__/ /_/ /
/_____/_/|_|\__/_/   \__,_/____/\____/_/   \__,_/   \__,_/\___/  /_/   /_____/\__,_/____/\__/_/\___/\____/  

https://github.com/MASTERCAR-B/Personal_Plastic_Extruder

Cualquier modificacion deseada llamar a  @gmail.com
No modifique ninguna linea de codigo sin tener presente el conocimiento adecuado sobre los componentes de la maquina
Ante cualquier duda comuniquese con el autor


yeah dm me if you need any help through gmail
i am NOT writing comments... just one
*/


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h> 
#include <PID_v1.h> 


const int thermistorPin = A1;  
const int potPin = A4;        
const int selectButtonPin = 31;
const int ssrPin = 12;          
const int greenButtonPin = 33; 
const int redButtonPin = 35;   
const int signalPin = 11;      

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
const unsigned long stepDelay = 500;
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

void controlSteppers() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastStep1Time >= stepDelay) {
    digitalWrite(stepper1_stp, HIGH);
    delayMicroseconds(100); // Short pulse
    digitalWrite(stepper1_stp, LOW);
    lastStep1Time = currentTime;
  }
  
  if (currentTime - lastStep2Time >= stepDelay) {
    if (digitalRead(endstop1) == LOW || digitalRead(endstop2) == LOW) {
      stepper2Direction = !stepper2Direction; // Change direction
      digitalWrite(stepper2_dir, stepper2Direction ? HIGH : LOW);
      delay(10); // Small delay to ensure direction change
    }
    
    digitalWrite(stepper2_stp, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepper2_stp, LOW);
    lastStep2Time = currentTime;
  }
}

void start() {
  isRunning = true;
  lcd.clear();
  lcd.print("Starting...");

  Serial.println("--- Initial Values ---");
  Serial.print("Setpoint Temperature: ");
  Serial.println(temperatura_buscada);
  Serial.print("Offset: ");
  Serial.println(offset);
  Serial.print("Motor Speed (RPM): ");
  Serial.println(velocidad_motor);
  Serial.println("-----------------------");

  delay(5000);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, windowSize);

  bool temperatureStable = false;

  while (!temperatureStable) {
    Input = readThermistor(offset);
    Setpoint = temperatura_buscada;
    myPID.Compute();
    
    unsigned long now = millis();
    static unsigned long windowStartTime = millis();
    if (now - windowStartTime > 5000) {
      windowStartTime += 5000;
    }

    if (Output > (now - windowStartTime)) {
      digitalWrite(ssrPin, HIGH);
    } else {
      digitalWrite(ssrPin, LOW);
    }
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T.B: ");
    lcd.print(Setpoint);
    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.print(Input, 1);

    if (abs(Input - Setpoint) <= 5.0) {
      temperatureStable = true;
      delay(500);
    }
    
    delay(500);
  }
  
  delay(5000);
  
  while (isRunning) {
    controlSteppers(); // Run stepper motors
    analogWrite(signalPin, map(velocidad_motor, 0, 60, 0, 255));
    nivelar(offset, temperatura_buscada);

    if (digitalRead(redButtonPin) == LOW) {
      isRunning = false;
      stop();
    }

    delay(100);
  }
}

void stop() {
  lcd.clear();
  lcd.print("Parando");
  digitalWrite(ssrPin, LOW);
  analogWrite(signalPin, 0);
  
  // Disable stepper motors
  digitalWrite(stepper1_ena, HIGH);
  digitalWrite(stepper2_ena, HIGH);
  
  delay(2000);
}

void nivelar(int offset, int tb) {
  int temperatura_buscada = tb;
  Serial.println(tb);
  Input = readThermistor(offset);
  Setpoint = temperatura_buscada;
  myPID.Compute();

  unsigned long now = millis();
  static unsigned long windowStartTime = millis();
  if (now - windowStartTime > 5000) {
    windowStartTime += 5000; 
  }
  if (Output > (now - windowStartTime)) {
    digitalWrite(ssrPin, HIGH); 
  } else {
    digitalWrite(ssrPin, LOW); 
  }
  delay(500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T.B: ");
  lcd.print(Setpoint);
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(Input, 1);
}
