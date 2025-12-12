/*
 * AdvancedPID Library - External Derivative Source Example
 * * Ideally, a PID calculates the Derivative term (D) by looking at the change in error over time.
 * However, this is noisy and mathematically delayed.
 * * If you have a Gyroscope (like BNO055 or MPU6050), you *already know* the rate of change!
 * Passing the Gyro rate directly to the PID results in incredibly smooth stabilization
 * with zero mathematical latency.
 * * Hardware Required: Adafruit BNO055 (or similar IMU)
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "AdvancedPID.h"

// Check I2C address (0x28 or 0x29)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// Stabilization PID
// Kp: Corrects angle error
// Ki: Corrects drift
// Kd: Dampens motion (D-Term). Since we use Gyro rate, this can be pushed higher!
// Kb: 0.0 (Clamping)
AdvancedPID pidStab(2.5, 0.0, 0.15, 0.0);

float targetAngle = 0.0; // Keep robot/gimbal flat
float motorOutput = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Initialize Sensor
  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // Setup PID
  pidStab.setOutputLimits(-255, 255); // Motor range
  pidStab.setIntegralZone(10.0);      // Only integrate when close to target (prevents windup when fallen over)
}

void loop() {
  sensors_event_t orientationData, gyroData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  // 1. Get Inputs
  // Current Angle (Pitch)
  float currentPitch = orientationData.orientation.y;
  
  // Gyro Rate (deg/s or rad/s depending on config)
  // NOTE: The derivative of position is velocity. The Gyro gives us velocity directly.
  float gyroRate = gyroData.gyro.y; 


  // 2. Run PID with EXTERNAL DERIVATIVE
  // Usage: run(Input, Setpoint, FeedForward, ExtDerivative)
  // We pass 0.0 for FeedForward.
  // We pass gyroRate for ExtDerivative.
  
  motorOutput = pidStab.run(currentPitch, targetAngle, 0.0, gyroRate);


  // 3. Drive Motors
  // driveMotors(motorOutput);


  // 4. Debug
  Serial.print("Angle:");
  Serial.print(currentPitch);
  Serial.print("\tGyroRate:"); // This acts as the D-Term
  Serial.print(gyroRate);
  Serial.print("\tOutput:");
  Serial.println(motorOutput);
  
  delay(10);
}