#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int selectButtonPin = 31;

int screenIndex = 0;
unsigned long lastButtonPress = 0;
const int debounceDelay = 200; 

void setupLCD() {
  lcd.init();
  lcd.backlight();
  pinMode(selectButtonPin, INPUT_PULLUP);
}

void checkScreenButton() {
  if (digitalRead(selectButtonPin) == LOW) {
    unsigned long currentTime = millis();
    if (currentTime - lastButtonPress > debounceDelay) {
      screenIndex++;
      if (screenIndex > 5) {
        screenIndex = 0;
      }
      lastButtonPress = currentTime;
    }
  }
}

void updateScreen(float currentTemp, float targetTemp, int motorSpeed, int offsetValue, bool isRunning) {
  lcd.clear();

  switch (screenIndex) {
    case 0:  
      lcd.print("Extrusora de");
      lcd.setCursor(0, 1);
      lcd.print("Plastico");
      break;

    case 1: 
      lcd.print("T.A: ");
      lcd.print(currentTemp, 1);
      lcd.setCursor(0, 1);
      lcd.print("T.B: ");
      lcd.print(targetTemp, 1);
      break;

    case 2:  
      lcd.print("Velocidad: ");
      lcd.print(motorSpeed);
      break;

    case 3: 
      lcd.print("Offset: ");
      lcd.print(offsetValue);
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

void showStarting() {
  lcd.clear();
  lcd.print("Starting...");
}

void showStopping() {
  lcd.clear();
  lcd.print("Parando");
}

void showTempStabilization(float setpoint, float currentTemp) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T.B: ");
  lcd.print(setpoint);
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(currentTemp, 1);
}
