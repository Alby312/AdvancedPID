#ifndef AdvancedPID_h
#define AdvancedPID_h

#include "Arduino.h"
#include <math.h> 

enum PIDMode { MODE_MANUAL, MODE_AUTO };
enum PIDDirection { DIR_DIRECT, DIR_REVERSE };

class AdvancedPID {
  public:
    // Constructor
    // Kb > 0 enables Back-Calculation. Kb = 0 (default) uses Clamping.
    AdvancedPID(float Kp, float Ki, float Kd, float Kb = 0.0);

    // --- Main Core ---
    
    // Calculates PID output. Call every loop cycle.
    // feedForward: (Optional) Pre-calculated value added directly to output.
    // extDerivative: (Optional) Gyro rate etc. NAN = use internal calculation.
    float run(float input, float setpoint, float feedForward = 0.0, float extDerivative = NAN);

    // Resets integral, filter history, and timer.
    void reset();

    // Mode switching. 'currentOutput' ensures bumpless transfer (Manual -> Auto).
    void setMode(PIDMode mode, float currentOutput = 0.0);

    // --- Configuration ---

    void setOutputLimits(float min, float max);
    void setTunings(float Kp, float Ki, float Kd, float Kb = 0.0);
    
    // Set alpha < 1.0 to enable filter (e.g., 0.8). Default 1.0 (OFF).
    void setDerivativeFilter(float alpha);
    
    // true = Proportional on Measurement (soft), false = P on Error (aggressive).
    void setPOnM(bool pOnM);

    // Sets a threshold for the error. If abs(error) < threshold, PID computes as if error is 0.
    // Use to prevent oscillation due to sensor noise. Default 0.0 (OFF).
    void setDeadband(float threshold);

    // Sets the Integral Zone. If abs(error) > zone, the Integral term is NOT updated.
    // Complements anti-windup by preventing buildup during large setpoint changes.
    // Set to 0.0 to disable (Default).
    void setIntegralZone(float zone);

    // Sets the direction of the controller.
    // DIR_DIRECT: Like a Heater (Input < Setpoint -> Output Increases)
    // DIR_REVERSE: Like a Cooler (Input > Setpoint -> Output Increases)
    void setControllerDirection(PIDDirection direction);

    // Limits the rate of change of the output (Slew Rate).
    // rate: Maximum change in output units per Second.
    // 0.0 = Disable (Default)
    void setOutputRampRate(float rate);

    // --- Getters ---
    float getKp();
    float getKi();
    float getKd();
    float getKb();
    PIDMode getMode();
    PIDDirection getDirection();

  private:
    float kp, ki, kd, kb;
    float outMin, outMax;
    float dFilterAlpha; 
    bool pOnE;
    PIDMode mode;
    PIDDirection direction;

    // Advanced features vars
    float deadband;
    float iZone;
    float outputRampRate; // Units per second

    unsigned long lastTime;
    float integralTerm;
    float lastError;
    float lastInput;
    float lastDTerm;
    float lastOutput;     // Needed for Ramp Rate
};

#endif