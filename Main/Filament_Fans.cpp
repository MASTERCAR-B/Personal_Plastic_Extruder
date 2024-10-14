const int ventiladorPin = 14; // Pin digital donde se conecta el ventilador

void setup() {
  pinMode(ventiladorPin, OUTPUT); // Configura el pin del ventilador como salida
  digitalWrite(ventiladorPin, HIGH); // Enciende el ventilador al inicio
  Serial.begin(9600); // Iniciar la comunicación serie para monitoreo
}

void loop() {
  // Mantiene el ventilador encendido
  Serial.println("Ventilador encendido");
  delay(10000); // Espera 10 segundos antes de enviar el siguiente mensaje
}
