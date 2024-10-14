#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Inicializa la LCD (dirección I2C: 0x27, 16 columnas, 2 filas)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variable creada por el usuario
int miVariable = 42; // Puedes cambiar este valor según lo necesites

void setup() {
  lcd.init(); // Inicializa la pantalla LCD
  lcd.backlight(); // Enciende la luz de fondo de la LCD
  
  // Mostrar un mensaje inicial en la LCD
  lcd.setCursor(0, 0); // Coloca el cursor en la primera fila, primera columna
  lcd.print("Valor: ");
}

void loop() {
  // Muestra el valor de la variable en la LCD
  lcd.setCursor(6, 0); // Coloca el cursor en la primera fila, columna 6
  lcd.print(miVariable); // Muestra el valor de la variable
  
  // O puedes actualizar la variable en cada iteración
  miVariable++; // Incrementa la variable para mostrar un nuevo valor cada vez
  
  delay(1000); // Espera un segundo antes de la siguiente actualización
}
