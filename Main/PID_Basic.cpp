#include <PID_v1.h> // Include the PID library

// PID control variables
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0; // PID tuning parameters

// PID object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setupPID() {
  // Initialize PID
  Setpoint = setpointTemperature; // Initial setpoint
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, windowSize); // Output limits correspond to window size
  myPID.SetSampleTime(1000); // Sample time in milliseconds
}

void loopPID() {
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
}
