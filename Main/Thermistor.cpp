//Make sure to use a 103 ceramic capacitor of 10μf and a 4.7k ohm resistor for each 100k thermistor you have 

#include <math.h>
const int termistorPin = A0; // Pin donde se conecta el termistor
const float resistenciaNominal = 100000; // 100k ohm a 25 grados Celsius
const float resistorFijo = 4700; // 4700 ohmios
const float B = 3950; // Constante B del termistor
const float A = 0.176323; // Constante B del termistor

void setup() {
  Serial.begin(9600); // Iniciar comunicación serie
}

void loop() {
  int reading = analogRead(termistorPin); // Leer valor analógico
  //Serial.print(reading);
  float voltaje = reading * (5.0 / 1023.0); // Convertir a voltaje
  //float resistenciaTermistor = (resistorFijo * (5.0 - voltaje)) / voltaje; // Calcular resistencia del termistor

  float temperatura = -223.15+(B/log((voltaje*resistorFijo)/(A*(5.0-voltaje))));
 
  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.println(" °C");
  Serial.println(voltaje);
  delay(1000);}
