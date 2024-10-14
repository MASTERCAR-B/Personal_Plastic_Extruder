#include <AccelStepper.h>  // Librería para controlar motores paso a paso

// Definir los pines de los motores
int motor1StepPin = 2;   // Pin para el motor del carrete
int motor1DirPin = 3;
int motor2StepPin = 4;   // Pin para el motor de la varilla roscada
int motor2DirPin = 5;

AccelStepper motor1(AccelStepper::DRIVER, motor1StepPin, motor1DirPin);  // Motor para el carrete
AccelStepper motor2(AccelStepper::DRIVER, motor2StepPin, motor2DirPin);  // Motor para la varilla roscada

// Variables para la velocidad y el diámetro del filamento
float diametroFilamento = 1.75;  // Inicialmente en mm, cambiar en función del valor ingresado
float radioCarrete = 50;         // Radio del carrete en mm
float anchoCarrete = 100;        // Ancho del carrete en mm
float pasoRosca = 0.3125;        // Paso de la rosca en mm
float velocidadFilamento = 10;   // Velocidad de salida del filamento en mm/s

void setup() {
  Serial.begin(9600);  // Inicializar el monitor serial para ingresar el diámetro
  motor1.setMaxSpeed(1000);  // Establecer la velocidad máxima de ambos motores
  motor2.setMaxSpeed(1000);

  pinMode(motor1StepPin, OUTPUT);
  pinMode(motor1DirPin, OUTPUT);
  pinMode(motor2StepPin, OUTPUT);
  pinMode(motor2DirPin, OUTPUT);

  // Aceleración de los motores
  motor1.setAcceleration(500);
  motor2.setAcceleration(500);
  
  Serial.println("Ingrese el diámetro del filamento en mm:");
}

void loop() {
  // Revisar si se ingresó un nuevo valor de diámetro desde el monitor serial
  if (Serial.available() > 0) {
    diametroFilamento = Serial.parseFloat();  // Leer el valor de diámetro ingresado
    Serial.print("Nuevo diámetro de filamento: ");
    Serial.println(diametroFilamento);

    // Calcular las velocidades de los motores en función del diámetro ingresado
    calcularVelocidades();
  }

  // Hacer que los motores giren a las velocidades calculadas
  motor1.runSpeed();
  motor2.runSpeed();
}

void calcularVelocidades() {
  // Calcular la velocidad del motor 1 (Carrete)
  float velocidadMotor1 = velocidadFilamento / (2 * 3.1416 * (radioCarrete + diametroFilamento / 2));
  motor1.setSpeed(velocidadMotor1);  // Establecer la velocidad del motor 1

  // Calcular la velocidad del motor 2 (Varilla roscada)
  float tiempoPasada = (anchoCarrete / velocidadFilamento);  // Tiempo que toma hacer una pasada completa
  float velocidadMotor2 = anchoCarrete / (pasoRosca * tiempoPasada);
  motor2.setSpeed(velocidadMotor2);  // Establecer la velocidad del motor 2

  Serial.print("Velocidad Motor 1: ");
  Serial.println(velocidadMotor1);
  Serial.print("Velocidad Motor 2: ");
  Serial.println(velocidadMotor2);
}
