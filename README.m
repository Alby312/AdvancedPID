# AdvancedPID Library for Arduino

A PID controller for Arduino that implements advanced features often missing from standard libraries. Designed for advanced control strategies.

## Key Features

* **Dual Anti-Windup:** Supports both *Clamping* (conditional) and *Back-Calculation* (dynamic).
* **Derivative Filter:** Low-Pass Filter on the derivative term to handle sensor noise.
* **Derivative on Measurement:** Calculates derivative on the Input rather than the Error to prevent "Derivative Kick" on setpoint changes.
* **External Derivative Source:** Allows passing a direct derivative value (e.g., from a Gyroscope) instead of calculating it, reducing latency and noise.
* **Feed-Forward Capability:** Supports adding a pre-calculated feed-forward term to the output to improve response to known disturbances.
* **Output Ramp Rate:** Limits the rate of change of the output (Slew Rate) to protect mechanical actuators from mechanical stress.
* **Bumpless Transfer:** Smooth transition between Manual and Auto modes.
* **Trapezoidal Integration:** More precise than the standard rectangular integration.
* **Proportional on Measurement (P-on-M):** Option to soften the response to setpoint changes (Proportional Kick reduction).

## Theory & References

This library does not reinvent anything; it implements established control strategies. Here are the theoretical references behind the functions used:

### 1. Anti-Windup (Clamping & Back-Calculation)
"Windup" occurs when the integral term grows indefinitely while the actuator is saturated (at max limit).
* **Clamping (Default):** Integration stops if the output is saturated and the error is trying to push it further into saturation.
* **Back-Calculation (If `Kb > 0`):** Dynamically reduces the integral term based on the difference between the calculated output and the actual saturated output.
* *Source:* [MathWorks - Anti-Windup Control Using a PID Controller](https://it.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html)
* *Web Ref:* [ControlGuru - Integral Reset Windup](https://controlguru.com/integral-reset-windup/)

### 2. Derivative on Measurement (D-on-M)
Standard PID calculates `d(Error)/dt`. This library calculates `-d(Input)/dt`. Mathematically, these are identical while the Setpoint is constant, but D-on-M eliminates the violent spike ("Kick") in output when the Setpoint is changed instantly.
* *Source:* [Brett Beauregard - Improving the Arduino PID Library (Derivative Kick)](http://brettbeauregard.com/blog/2011/04/improving-the-beginner-pid-derivative-kick/)

### 3. Derivative Filter (Low Pass Filter)
The derivative term is highly sensitive to high-frequency noise. An exponential smoothing filter (Alpha filter) is applied to the derivative term.
* **Formula:** `D_term = (alpha * D_raw) + ((1 - alpha) * last_D)`
* *Source:* [Wikipedia - Low-pass filter (Discrete-time implementation)](https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter)

### 4. Trapezoidal Integration
Most basic libraries use rectangular integration (`I += error * dt`). This library uses the trapezoidal rule (`I += (error + lastError) * 0.5 * dt`) for a better approximation of the area under the curve.
* *Source:* [Tustin's Method / Trapezoidal Rule](https://en.wikipedia.org/wiki/Trapezoidal_rule)

### 5. Proportional on Measurement (P-on-M)
Moves the proportional term to feedback path. The controller reacts only to changes in the process variable, not the setpoint. This creates a smoother response to setpoint changes.
* *Source:* [Expert Tune - P-on-M vs P-on-E](http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/)

## Installation

Arduino IDE:
1. Download this repository as a .ZIP file.
2. In the Arduino IDE, go to: **Sketch** -> **Include Library** -> **Add .ZIP Library**.

PlatformIO:
1. Open platformio.ini
2. add `lib_deps = Alby312/AdvancedPID` or `lib_deps = https://github.com/Alby312/AdvancedPID`

## Usage Example

```cpp
#include "AdvancedPID.h"

// Kp, Ki, Kd, Kb (Back-calculation coefficient)
AdvancedPID myPID(2.0, 5.0, 1.0, 0.0); 

void setup() {
  myPID.setOutputLimits(0, 255);
  // Enable derivative filter (0.8 = strong smoothing, 1.0 = OFF)
  myPID.setDerivativeFilter(0.8);
  
  // Protect actuator: Output can change max 100 units per second
  myPID.setOutputRampRate(100.0);
}

void loop() {
  float input = analogRead(A0);
  float setpoint = 512.0; // Target value
  
  // Optional: Read feed-forward (e.g. battery voltage compensation)
  float feedForward = 10.0; 

  // Optional: Read external derivative (e.g. from gyroscope)
  float gyroRate = 0.0; // Read from sensor...

  // Run PID with advanced features
  float output = myPID.run(input, setpoint, feedForward, gyroRate);
  
  analogWrite(3, output);
  delay(10); // PID automatically handles dt calculation
}

## Contributing

This library is an ongoing project to advanced control strategies to the Arduino ecosystem.
If you have ideas for new features and optimizations **pull requests and suggestions are highly welcome!**

## License
MIT License.