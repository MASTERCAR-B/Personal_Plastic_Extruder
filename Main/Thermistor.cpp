#include <math.h>

const int thermistorPin = A1;
const float seriesResistor = 220.0;
const float nominalResistance = 100000.0;
const float nominalTemperature = 25.0;
const float bCoefficient = 3950.0;

float readThermistor() {
    int adcValue = analogRead(thermistorPin);
    float resistance = seriesResistor * ((1023.0 / adcValue) - 1.0);
    float steinhart;
    steinhart = resistance / nominalResistance;
    steinhart = log(steinhart);
    steinhart /= bCoefficient;
    steinhart += 1.0 / (nominalTemperature + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;
    return steinhart;
}

void setup() {
    Serial.begin(9600);
}

void loop() {
    float temperature = readThermistor();
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    delay(1000);
}
