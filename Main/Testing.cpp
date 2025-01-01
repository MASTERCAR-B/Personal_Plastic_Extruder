//Im not sure this code works or not since at the moment i do not have access to the main machine, will update whenever im able to check it. This code uses PID system instead of the Bang-Bang one.


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <PID_v1.h>

const int botonRojo = 0;
const int botonVerde = 1;
const int botonSeleccion = 3;
const int motorPin = 2;
const int potPin = A2;
const int relePin = 9;
const int termistorPin = A1;

const float resistenciaNominal = 100000;
const float resistorFijo = 4700;
const float B = 3950;
const float A = 0.176323;

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2.0, 5.0, 1.0, DIRECT);

float temperatura_actual = 25.5;
float temperatura_buscada = 30.0;
int velocidad_motor = 120;
float offset = 0.0;
unsigned long ultimaActualizacionPID = 0;
const unsigned long intervaloPID = 1000;

LiquidCrystal_I2C lcd(0x27, 16, 2);

byte estadoBoton = LOW;
byte ultimoEstadoBoton = LOW;
byte indicePantalla = 0;

void setup() {
  lcd.init();
  lcd.backlight();
  pinMode(botonSeleccion, INPUT_PULLUP);
  pinMode(botonVerde, INPUT_PULLUP);
  pinMode(potPin, INPUT);
  pinMode(relePin, OUTPUT);
  mostrarPantalla();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  byte estadoActualBoton = digitalRead(botonSeleccion);
  byte estadoBotonRojo = digitalRead(botonRojo);

  if (estadoActualBoton == LOW && ultimoEstadoBoton == HIGH) {
    indicePantalla = (indicePantalla + 1) % 5;
    mostrarPantalla();
  }

  if (estadoBotonRojo == LOW) {
    reiniciarTodo();
  }

  if (indicePantalla == 4 && digitalRead(botonVerde) == LOW) {
    empezar(temperatura_buscada, velocidad_motor);
  }

  ultimoEstadoBoton = estadoActualBoton;
  delay(50);
}

void mostrarPantalla() {
  lcd.clear();
  float temperatura_actual = leerTemperatura(offset);

  switch (indicePantalla) {
    case 0:
      lcd.print("  Extrusora de");
      lcd.setCursor(0, 1);
      lcd.print("    Plastico");
      break;

    case 1:
      int valorPot = analogRead(potPin);
      temperatura_buscada = map(valorPot, 0, 1023, 0, 100);
      lcd.print("T.A: ");
      lcd.print(temperatura_actual, 1);
      lcd.setCursor(0, 1);
      lcd.print("T.B: ");
      lcd.print(temperatura_buscada);
      break;

    case 2:
      int valorPotMotor = analogRead(potPin);
      velocidad_motor = map(valorPotMotor, 0, 1023, 0, 66);
      lcd.print("Velocidad M.P:");
      lcd.setCursor(0, 1);
      lcd.print(velocidad_motor);
      break;

    case 3:
      lcd.print("Offset: ");
      lcd.setCursor(0, 1);
      lcd.print(offset, 1);
      int valorPotOffset = analogRead(potPin);
      offset = map(valorPotOffset, 0, 1023, 0, 300);
      break;

    case 4:
      lcd.print("Comenzar ?");
      lcd.setCursor(0, 1);
      lcd.print("Presionar Verde");
      break;
  }
}

float leerTemperatura(float offset) {
  int lectura = analogRead(termistorPin);
  float voltaje = lectura * (5.0 / 1023.0);
  float temperatura = offset + (B / log((voltaje * resistorFijo) / (A * (5.0 - voltaje))));
  return temperatura;
}

void empezar(float temp_buscada, int velocidad_motor) {
  Setpoint = temp_buscada;
  unsigned long millisActual = millis();

  while (true) {
    temperatura_actual = leerTemperatura(offset);
    Input = temperatura_actual;

    if (millis() - ultimaActualizacionPID >= intervaloPID) {
      myPID.Compute();
      ultimaActualizacionPID = millis();
    }

    analogWrite(relePin, Output);

    if (abs(Setpoint - temperatura_actual) < 1.0) {
      break;
    }

    delay(100);
  }

  analogWrite(relePin, 0);
  Serial.println("Se alcanzo la temperatura deseada");
}

void reiniciarTodo() {
  velocidad_motor = 0;
  offset = 0.0;
  indicePantalla = 0;
  Setpoint = 0;
  Input = 0;
  Output = 0;

  myPID.SetMode(MANUAL);
  analogWrite(relePin, 0);
  lcd.clear();
  lcd.print("  Reiniciando...");
  delay(1000);
  mostrarPantalla();
}
