//this is the overall code for the extruder

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h> // Include math library for logarithm calculations
#include <PID_v1.h> // Include the PID library

// LCD configuration
// Initialize the LCD (I2C address: 0x27, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2); // Change to 0x3f if 0x27 doesn't work

// Pin definitions
const int thermistorPin = A3; // Analog pin where the thermistor is connected
const int potPin = A2; // Analog pin connected to the potentiometer
const int selectButtonPin = 3; // Digital pin connected to the select button
const int ssrPin = 9; // Digital output pin to control the SSR (or LED for testing)

// Variables
float setpointTemperature = 25.0; // Initial setpoint temperature value
bool editMode = false; // Edit mode flag

// Variables to store the last button state (for debouncing)
int lastSelectButtonState = HIGH;

// Thermistor constants and variables
const float resistenciaNominal = 100000.0; // 100k ohm at 25 degrees Celsius
const float resistorFijo = 4700.0; // 4.7k ohms
const float B = 3950.0; // B constant of the thermistor
const float A = 0.176323; // A constant for calculations

// PID control variables
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0; // PID tuning parameters

// PID object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Time-proportional control variables
unsigned long windowStartTime;
const unsigned long windowSize = 1000; // 1-second window (adjusted for testing)

void setup() {
  lcd.init();
  lcd.backlight();

  // Set up the button pin with internal pull-up resistor
  pinMode(selectButtonPin, INPUT_PULLUP);

  // Set up the SSR control pin (or LED for testing)
  pinMode(ssrPin, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize PID
  Setpoint = setpointTemperature; // Initial setpoint
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, windowSize); // Output limits correspond to window size
  myPID.SetSampleTime(1000); // Sample time in milliseconds

  // Initialize window start time
  windowStartTime = millis();
}

void loop() {
  // Read the current state of the select button
  int selectButtonState = digitalRead(selectButtonPin);

  // Print the state of the button
  if (selectButtonState == LOW) {
    Serial.println("Button Pressed");
  } else {
    Serial.println("Button Not Pressed");
  }

  // Check if the select button was pressed
  if (selectButtonState == LOW && lastSelectButtonState == HIGH) {
    editMode = !editMode; // Toggle edit mode

    if (editMode) {
      // Entering edit mode
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Adjust Setpoint");
    } else {
      // Exiting edit mode, clear the screen
      lcd.clear();
    }
    delay(200); // Debounce delay
  }
  lastSelectButtonState = selectButtonState;

  if (editMode) {
    // In edit mode, read the potentiometer and update setpoint temperature
    int potValue = analogRead(potPin); // Read potentiometer value (0-1023)
    // Map the potentiometer value to the desired temperature range (adjust as needed)
    setpointTemperature = map(potValue, 0, 1023, 0, 100);

    // Update the LCD to show the setpoint temperature
    lcd.setCursor(0, 1);
    lcd.print("Setpoint: ");
    lcd.print(setpointTemperature, 1);
    lcd.print(" C ");
    delay(100); // Small delay to prevent flickering
  } else {
    // Not in edit mode, proceed with temperature control

    // Read the actual temperature from the thermistor
    float temperature_read = readThermistor();

    // Update PID variables
    Input = temperature_read; // Actual temperature
    Setpoint = setpointTemperature; // Desired temperature

    // Compute PID output
    myPID.Compute();

    // Time-proportional control logic for SSR (or LED)
    unsigned long now = millis();
    if (now - windowStartTime > windowSize) {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }

    // Control the SSR (or LED) based on the PID output
    if (Output > now - windowStartTime) {
      digitalWrite(ssrPin, HIGH); // SSR (or LED) ON
    } else {
      digitalWrite(ssrPin, LOW); // SSR (or LED) OFF
    }

    // Print PID Output and temperature readings to Serial Monitor for debugging
    Serial.print("PID Output: ");
    Serial.println(Output);
    Serial.print("Temperature: ");
    Serial.println(temperature_read);

    // Update LCD display with current temperature and setpoint
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature_read, 2);
    lcd.print(" C ");
    lcd.setCursor(0, 1);
    lcd.print("Set: ");
    lcd.print(setpointTemperature, 1);
    lcd.print(" C ");

    delay(1000); // Update every second
  }
}

// Function to read the thermistor and calculate temperature
double readThermistor() {
  int reading = analogRead(thermistorPin); // Read analog value from thermistor
  float voltage = reading * (5.0 / 1023.0); // Convert reading to voltage

  // Calculate resistance of thermistor
  float resistance = (voltage * resistorFijo) / (5.0 - voltage);

  // Calculate temperature in Kelvin using the Steinhart-Hart equation or Beta coefficient method
  float temperatureK = (B * 298.15) / (B + (298.15 * log(resistance / resistenciaNominal)));

  // Convert Kelvin to Celsius
  float temperatureC = temperatureK - 273.15;

  return temperatureC;
}
