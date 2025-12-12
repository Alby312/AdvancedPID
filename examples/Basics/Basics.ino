/*
 * AdvancedPID Library - Basic Heater Example
 * * This example simulates a simple thermal process (like a 3D printer hotend).
 * It demonstrates the basic setup and the "Clamping" anti-windup method.
 * * HOW TO USE:
 * 1. Upload the sketch.
 * 2. Open Tools -> Serial Plotter.
 * 3. Watch how the Output adjusts to bring Input to the Setpoint.
 */

#include "AdvancedPID.h"

// Initialize PID: Kp=2.0, Ki=0.5, Kd=1.0
// Kb = 0.0 means we are using "Clamping" Anti-Windup (Standard)
AdvancedPID myPID(2.0, 0.5, 1.0, 0.0); 

float setpoint = 100.0; // Target temperature
float input = 25.0;     // Current temperature (Ambient start)
float output = 0.0;     // PWM output (0-255)

void setup() {
  Serial.begin(115200);

  // Configure Output limits (e.g., PWM range for Arduino)
  myPID.setOutputLimits(0, 255);

  // Optional: Set a small deadband. If error is < 0.5, output won't change jitterily.
  myPID.setDeadband(0.5); 
  
  Serial.println("Time(ms)\tInput\tSetpoint\tOutput");
}

void loop() {
  // --- 1. SIMULATION (Replace this with real sensor reading) ---
  // Simple thermal model: Temperature rises with output, cools down over time.
  // input = analogRead(THERMISTOR_PIN); 
  input += (output * 0.1) - ((input - 25.0) * 0.05); 


  // --- 2. PID CALCULATION ---
  // run(Input, Setpoint)
  output = myPID.run(input, setpoint);


  // --- 3. ACTUATOR (Replace with real hardware) ---
  // analogWrite(HEATER_PIN, output);


  // --- 4. TELEMETRY ---
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(input);
  Serial.print("\t");
  Serial.print(setpoint);
  Serial.print("\t");
  Serial.println(output);

  delay(50); // Run loop at approx 20Hz
}