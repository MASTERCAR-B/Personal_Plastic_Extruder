const int ventiladorPin = 14; // Pin digital donde se conecta el ventilador

void setup() {
  pinMode(ventiladorPin, OUTPUT); // Configura el pin del ventilador como salida
  Serial.begin(9600); // Iniciar la comunicación serie para monitoreo
}

void loop() {
  // Encender el ventilador
  digitalWrite(ventiladorPin, HIGH); // Activa el ventilador
  Serial.println("Ventilador encendido");
  delay(5000); // Mantener el ventilador encendido por 5 segundos

  // Apagar el ventilador
  digitalWrite(ventiladorPin, LOW); // Desactiva el ventilador
  Serial.println("Ventilador apagado");
  delay(5000); // Mantener el ventilador apagado por 5 segundos
}
