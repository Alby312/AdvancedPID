/*
 * AdvancedPID Library - FeedForward & Filter Example
 * * This example simulates a Velocity Control loop for a DC motor.
 * * FEATURES DEMONSTRATED:
 * 1. Feed-Forward: Predicting the voltage needed for a specific speed to improve response time.
 * 2. Derivative Filter: Smoothing out noisy speed readings (common with Encoders).
 * 3. Back-Calculation Anti-Windup: Useful for systems with slow response.
 */

#include "AdvancedPID.h"

// PID for Velocity
// Kp, Ki, Kd
// Kb = 1.0 -> Enable Back-Calculation Anti-Windup (Dynamic un-winding)
AdvancedPID velocityPID(1.5, 8.0, 0.05, 1.0);

float targetSpeed = 100.0; // RPM or Rad/s
float currentSpeed = 0.0;
float voltageCmd = 0.0;

// Simulation variables
float simulatedLoad = 0.0;

void setup() {
  Serial.begin(115200);

  velocityPID.setOutputLimits(-12.0, 12.0); // Voltage limits (e.g., 12V motor)
  
  // --- DERIVATIVE FILTER ---
  // Real sensors (encoders) are noisy. A D-term on raw data causes "jitters".
  // 1.0 = No Filter
  // 0.1 = Heavy Filter (Very smooth, more lag)
  // 0.8 = Good balance
  velocityPID.setDerivativeFilter(0.8);
}

void loop() {
  // Change target speed every 3 seconds to see response
  if (millis() % 6000 < 3000) targetSpeed = 150.0;
  else targetSpeed = 50.0;

  // --- 1. SIMULATE NOISY SENSOR ---
  // In real life: currentSpeed = encoder.getVelocity();
  // We add random noise to simulate a real encoder
  float noise = (random(-10, 10) / 10.0); 
  simulatedLoad += (voltageCmd * 2.0 - simulatedLoad) * 0.1; // Motor physics lag
  currentSpeed = simulatedLoad + noise; 


  // --- 2. CALCULATE FEED-FORWARD ---
  // Feed-Forward (FF) guesses the output based on physics, bypassing the PID loop.
  // If we know our motor needs 6V to spin at 100RPM, we can just give it 6V immediately
  // and let the PID fix the remaining error.
  float kV = 0.05; // Motor constant (Volts per RPM)
  float feedForward = targetSpeed * kV;


  // --- 3. RUN PID ---
  // run(Input, Setpoint, FeedForward, ExtDerivative)
  // We pass NAN for ExtDerivative to let the PID calculate it internally (with the filter applied).
  voltageCmd = velocityPID.run(currentSpeed, targetSpeed, feedForward, NAN);


  // --- 4. PLOT ---
  Serial.print("Target:");
  Serial.print(targetSpeed);
  Serial.print("\tSpeed:");
  Serial.print(currentSpeed);
  Serial.print("\tVoltage:");
  Serial.println(voltageCmd);

  delay(20);
}