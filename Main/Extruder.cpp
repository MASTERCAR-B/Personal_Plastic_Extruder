/*
    ______     __                                          __        ____  __               __  _          
   / ____/  __/ /________  ___________  _________ _   ____/ /__     / __ \/ /   ____ ______/ /_(_)________ 
  / __/ | |/_/ __/ ___/ / / / ___/ __ \/ ___/ __ `/  / __  / _ \   / /_/ / /   / __ `/ ___/ __/ / ___/ __ \
 / /____>  </ /_/ /  / /_/ (__  ) /_/ / /  / /_/ /  / /_/ /  __/  / ____/ /___/ /_/ (__  ) /_/ / /__/ /_/ /
/_____/_/|_|\__/_/   \__,_/____/\____/_/   \__,_/   \__,_/\___/  /_/   /_____/\__,_/____/\__/_/\___/\____/  

https://github.com/MASTERCAR-B/Personal_Plastic_Extruder

Cualquier modificacion deseada llamar a ZAVALETA lucaszavaleta10@gmail.com
No modifique ninguna linea de codigo sin tener presente el conocimiento adecuado sobre los componentes de la maquina
Ante cualquier duda comuniquese con el autor

*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h> 
#include <PID_v1.h> 

LiquidCrystal_I2C lcd(0x27, 16, 2); 

const int thermistorPin = A1;  
const int potPin = A4;        
const int selectButtonPin = 31;
const int ssrPin = 12;          
const int greenButtonPin = 33; 
const int redButtonPin = 35;   
const int signalPin = 11;      

int screenIndex = 0;            
float setpointTemperature = 30.0; 
float temperatura_buscada = 25.0;
int velocidad_motor = 0;
int offset = 48;
bool isRunning = false;         

int potValueTemp = 0;
int potValueMotor = 0;
int potValueOffset = 0;

double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0; // PID tuning parameters

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const float A = 0.176323;
const float B = 3950.0;
const float resistorFijo = 4700.0;

unsigned long windowStartTime;
const unsigned long windowSize = 5000; // 1-second window

// PID variables from the first code
double dt, last_time;
double integral, previous, output = 0;

void setup() {
  lcd.init();
  lcd.backlight();

  pinMode(selectButtonPin, INPUT_PULLUP);
  pinMode(greenButtonPin, INPUT_PULLUP);
  pinMode(redButtonPin, INPUT_PULLUP);
  pinMode(ssrPin, OUTPUT);
  pinMode(signalPin, OUTPUT);

  Setpoint = setpointTemperature;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, windowSize); 
  myPID.SetSampleTime(1000);

  windowStartTime = millis();
  Serial.begin(9600);
}

void loop() {
  int selectButtonState = digitalRead(selectButtonPin);
  if (selectButtonState == LOW) {
    screenIndex++;  
    if (screenIndex > 5) {
      screenIndex = 0;
    }
    delay(200);  
  }

  int greenButtonState = digitalRead(greenButtonPin);
  if (greenButtonState == LOW && screenIndex == 4 && !isRunning) {
    isRunning = true;
    start();
    delay(200); 
  }

  int redButtonState = digitalRead(redButtonPin);
  if (redButtonState == LOW && isRunning) {
    isRunning = false;
    stop(); 
    delay(200); 
  }

  displayScreen();

  if (screenIndex == 1 || screenIndex == 2 || screenIndex == 3) {
    if (screenIndex == 1) {
      potValueTemp = analogRead(potPin); 
      temperatura_buscada = map(potValueTemp, 0, 1023, 0, 240); 
    }
    if (screenIndex == 2) {
      potValueMotor = analogRead(potPin);
      velocidad_motor = map(potValueMotor, 0, 1023, 0, 60);  
    }
    if (screenIndex == 3) {
      potValueOffset = analogRead(potPin);
      offset = map(potValueOffset, 0, 1023, 0, 80); 
    }
  }

  if (screenIndex == 1) {
    Input = readThermistor(offset); // Get temperature from thermistor
    Setpoint = temperatura_buscada; 
    output = pid(Setpoint - Input); // Use PID from first system for temperature control

    unsigned long now = millis();
    if (now - windowStartTime > windowSize) {
      windowStartTime += windowSize;
    }

    if (output > (now - windowStartTime)) {
      digitalWrite(ssrPin, HIGH); // Turn SSR on
    } else {
      digitalWrite(ssrPin, LOW);  // Turn SSR off
    }

    Serial.print("PID Output: ");
    Serial.println(output);
    Serial.print("Temperature: ");
    Serial.println(Input);
  }

  // Ensure the motor is only controlled in start() and stop()
  if (!isRunning) {
    analogWrite(signalPin, 0);
  }

  delay(1000); 
}

void displayScreen() {
  lcd.clear();
//dani si lo estas leyendo... ya sabes
  switch (screenIndex) {
    case 0:  
      lcd.print("Extrusora de");
      lcd.setCursor(0, 1);
      lcd.print("Plastico");
      break;

    case 1:  
      lcd.print("T.A: ");
      lcd.print(readThermistor(offset), 1);  
      lcd.setCursor(0, 1);
      lcd.print("T.B: ");
      lcd.print(temperatura_buscada, 1); 
      break;

    case 2: 
      lcd.print("Velocidad: ");
      lcd.print(velocidad_motor); 
      break;

    case 3:  
      lcd.print("Offset: ");
      lcd.print(offset); 
      break;

    case 4: 
      lcd.print("Comenzar?");
      lcd.setCursor(0, 1);
      lcd.print("Presionar Verde");
      break;

    case 5:  
      lcd.print("Enrollar?");
      break;
  }
}

double readThermistor(int offset) {
  int reading = analogRead(thermistorPin);
  float voltage = reading * (5.0 / 1023.0);
  float temperature = -223.15 + (B / log((voltage * resistorFijo) / (A * (5.0 - voltage)))) - offset;
  return temperature;
}

// PID function from the first system
double pid(double error) {
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  return output;
}

// Start function
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

  // Stabilize temperature before starting the motor
  while (!temperatureStable) {
    Input = readThermistor(offset);
    Setpoint = temperatura_buscada;
    myPID.Compute();
    
    unsigned long now = millis();
    static unsigned long windowStartTime = millis();
    if (now - windowStartTime > 5000) {
      windowStartTime += 5000; // Reset the 5-second window
    }

    if (Output > (now - windowStartTime)) {
      digitalWrite(ssrPin, HIGH); // Turn SSR on
    } else {
      digitalWrite(ssrPin, LOW);  // Turn SSR off
    }
    delay(500);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T.B: ");
    lcd.print(Setpoint);
    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.print(Input, 1);

    if (abs(Input - Setpoint) <= 5.0) { // Adjusted error margin
      temperatureStable = true;
      delay(500);
    }
  }
  delay(5000);
  while (isRunning) {
    delay(1000);
    analogWrite(signalPin, map(velocidad_motor, 0, 60, 0, 255));
    nivelar(offset, temperatura_buscada);

    if (digitalRead(redButtonPin) == LOW) {
      isRunning = false;
      stop();
    }

    delay(100);
  }
}

// Stop function
void stop() {
  lcd.clear();
  lcd.print("Parando");
  digitalWrite(ssrPin, LOW);
  analogWrite(signalPin, 0);
  delay(2000);
}

void nivelar(int offset, int tb) {
  int temperatura_buscada = tb;
  Serial.println(tb);
  Input = readThermistor(offset);
  Setpoint = temperatura_buscada;
  output = pid(Setpoint - Input); 
  myPID.Compute();
}
