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

const int redButtonPin = 27;        // Pin 0 for the red button
const int greenButtonPin = 29;      // Pin 1 for the green button
const int selectionButtonPin = 31;  // Pin 3 for the selection button
const int motorPin = 11;             // Pin 2 for the motor PWM output
const int potPin = A2;
const int relayPin = 12;
const int termistorPin = A1; // Pin where the thermistor is connected
const float resistenciaNominal = 100000; // 100k ohm at 25 degrees Celsius
const float resistorFijo = 4700; // 4700 ohms
const float B = 3950; // B constant of the thermistor
const float A = 0.176323; // A constant of the thermistor
byte lastButtonState = LOW;    // Previous button state
byte screenIndex = 0;          // Tracks current screen
float temperatura_actual;  // Example temperature A value
float temperatura_buscada; // Example temperature B value
int velocidad_motor;        // Example motor speed
int final1 = 23;
int final2 = 25;
int diametro_carrete;
int diametro_filamento;
int offset = -250;
int DIR = 5;
int PUL =  4;
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

LiquidCrystal_I2C lcd(0x27, 16, 2); // Change the address if needed
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup(){


  lcd.init();
  lcd.backlight();
  pinMode(redButtonPin, INPUT_PULLUP);
pinMode(greenButtonPin, INPUT_PULLUP);
pinMode(selectionButtonPin, INPUT_PULLUP);
pinMode(final1, INPUT_PULLUP);
pinMode(final2, INPUT_PULLUP);
pinMode(motorPin, OUTPUT);
pinMode(relayPin, OUTPUT);
pinMode(6, OUTPUT);
pinMode(DIR, OUTPUT);
pinMode(PUL, OUTPUT);
}


void loop() {
  byte currentButtonState = digitalRead(selectionButtonPin);
 
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    screenIndex = (screenIndex + 1) % 4;
    displayScreen();
  }
  lastButtonState = currentButtonState;
  if (screenIndex == 4 && digitalRead(greenButtonPin) == LOW) {
    empezar_programa(temperatura_buscada, velocidad_motor, offset);
  }
  if (screenIndex == 5 && digitalRead(greenButtonPin) == LOW) {
    enrollar(diametro_carrete, diametro_filamento);
  }
  delay(50);
}

void displayScreen() {
  lcd.clear();
  float temperatura_actual = leer_temperatura(offset);

  switch (screenIndex) {
    case 0:
      lcd.print("  Extrusora de"); // First screen
      lcd.setCursor(0, 1);
      lcd.print("    Plastico");
      break;

    case 1:
      int potValue = analogRead(potPin);
      temperatura_buscada = map(potValue, 0, 1023, 0, 100);
      lcd.print("T.A: "); // Second screen
      lcd.print(temperatura_actual, 1); // Display current temperature with 1 decimal
      lcd.setCursor(0, 1);
      lcd.print("T.B: ");
      lcd.print(temperatura_buscada);
      break;

    case 2:
      int potValueMotor = analogRead(potPin);
      velocidad_motor = map(potValueMotor, 0, 1023, 0, 66);

      lcd.print("Velocidad M.P:"); // Third screen
      lcd.setCursor(0, 1);
      lcd.print(velocidad_motor); // Display the adjusted motor speed
      break;

    case 3:
      lcd.print("Offset: "); // Fourth screen (previously "Empezar" screen)
      lcd.setCursor(0, 1);
      lcd.print(offset, 1); // Display current offset with 1 decimal
     
      // Adjust offset with potentiometer
      int potValueOffset = analogRead(potPin);
      offset = map(potValueOffset, 0, 1023, -300, 0); // Map potentiometer value to offset range
      break;

    case 4:
      lcd.print("Comenzar ?"); // Fifth screen
      lcd.setCursor(0, 1);
      lcd.print("Presionar Verde");

    case 5:
      lcd.print("Enrollar?");

  }
}

float leer_temperatura(float offset) {
  int reading = analogRead(termistorPin);
  float voltaje = reading * (5.0 / 1023.0);
  float temperatura = offset + (B / log((voltaje * resistorFijo) / (A * (5.0 - voltaje))));
  return temperatura;
}

void empezar_programa(float temp_buscada, int velocidad_motor, int offset) {
  // Setpoint is the desired temperature
  int Setpoint = temp_buscada;
 
  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // Output range for PID (on/off or duty cycle)

  // Set relay pin as output
  pinMode(relayPin, OUTPUT);

  while (true) {
    temperatura_actual = leer_temperatura(offset);
   
    int Input = temperatura_actual;
    myPID.Compute();

    if (Output > 127) {
      digitalWrite(relayPin, HIGH); // Turn relay on
    } else {
      digitalWrite(relayPin, LOW); // Turn relay off
    }

    if (abs(Setpoint - temperatura_actual) < 1.0) {
      int pwmValue = map(velocidad_motor, 0, 66, 0, 255);
      analogWrite(relayPin, pwmValue);
      break;
    }

    delay(1000);
  }

  digitalWrite(relayPin, LOW);
  Serial.println("Target temperature reached!");
}






void enrollar(int diametro_carrete, int diametro_filamento){
  digitalWrite(6, HIGH);

  // Main loop for alternating signals to pin 4 and pin 5
  while (true) {
    // Signal to pin 4 until a pulse is detected on pin 23
    Serial.println("Sending signal to pin 4...");
    while (true) {
      digitalWrite(4, HIGH);
      delay(100); // Adjust timing as needed
      digitalWrite(4, LOW);
      delay(100);

      if (digitalRead(23) == LOW) { // Detect pulse (LOW state)
        delay(50); // Debounce delay
        if (digitalRead(23) == LOW) { // Confirm pulse
          break; // Exit loop on pulse detection
        }
      }
    }

    Serial.println("Stopping pin 4, moving to pin 5...");

    // Signal to pin 5 until a pulse is detected on pin 25
    while (true) {
      digitalWrite(5, HIGH);
      delay(100); // Adjust timing as needed
      digitalWrite(5, LOW);
      delay(100);

      if (digitalRead(25) == LOW) { // Detect pulse (LOW state)
        delay(50); // Debounce delay
        if (digitalRead(25) == LOW) { // Confirm pulse
          break; // Exit loop on pulse detection
        }
      }
    }

    Serial.println("Stopping pin 5, moving back to pin 4...");
  }
}
