#include "AdvancedPID.h"

AdvancedPID::AdvancedPID(float Kp, float Ki, float Kd, float Kb)
{
    setTunings(Kp, Ki, Kd, Kb);

    // Defaults
    outMin = 0.0;
    outMax = 255.0;
    dFilterAlpha = 1.0; // 1.0 = No Filter
    pOnE = true;        // Default: P on Error
    mode = MODE_AUTO;
    direction = DIR_DIRECT;

    deadband = 0.0;
    iZone = 0.0;
    outputRampRate = 0.0;

    reset();
}

float AdvancedPID::run(float input, float setpoint, float feedForward, float extDerivative)
{
    // If Manual, update history to prevent jumps when switching back, then return 0
    if (mode == MODE_MANUAL)
    {
        lastInput = input;
        lastOutput = 0.0; // Or keep tracking? Ideally 0 or external set.
        lastTime = micros();
        return 0.0;
    }

    // 1. Time Calculation (Dynamic dt)
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;

    if (dt <= 0.000001)
    { // Safety check for very fast loops or timer overflow
        dt = 0.000001;
    }
    lastTime = now;

    // 2. Error Calculation & Deadband
    float error;
    if (direction == DIR_DIRECT)
        error = setpoint - input;
    else
        error = input - setpoint; // Reverse action

    if (deadband > 0.0 && fabs(error) < deadband)
    {
        error = 0.0;
    }

    // 3. Proportional Term
    float pTerm;
    if (pOnE)
    {
        pTerm = kp * error; // Standard P on Error
    }
    else
    {
        // P on Measurement (reduced overshoot)
        // Must respect direction
        if (direction == DIR_DIRECT)
            pTerm = -kp * input;
        else
            pTerm = kp * input;
    }

    // 4. Integral Term Step
    float iIncrement = ki * 0.5f * (error + lastError) * dt;

    // Integral Zone: If error is too large, disable integration
    if (iZone > 0.0 && fabs(error) > iZone)
    {
        iIncrement = 0.0;
    }

    // 5. Derivative Term (with optional External Source)
    float dTermRaw;
    if (isnan(extDerivative))
    {
        // Internal Calc: Derivative on Measurement (avoids Setpoint kick)
        float dInput = input - lastInput;
        // Direction matters for D term too
        if (direction == DIR_DIRECT)
            dTermRaw = -kd * (dInput / dt);
        else
            dTermRaw = kd * (dInput / dt);
    }
    else
    {
        // Use External Source (e.g., Gyro)
        // Ensure user passes correct sign, or we adapt?
        // Usually ExtDerivative is Rate. For Direct, we subtract rate.
        if (direction == DIR_DIRECT)
            dTermRaw = -kd * extDerivative;
        else
            dTermRaw = kd * extDerivative;
    }

    // Low Pass Filter
    float dTerm = (dFilterAlpha * dTermRaw) + ((1.0 - dFilterAlpha) * lastDTerm);
    lastDTerm = dTerm;

    // 6. Compute Raw Output
    float outCalc = pTerm + (integralTerm + iIncrement) + dTerm + feedForward;

    // 7. Output Ramp Rate (Slew Rate Limiting)
    if (outputRampRate > 0.0)
    {
        float maxChange = outputRampRate * dt;
        if (outCalc > lastOutput + maxChange)
        {
            outCalc = lastOutput + maxChange;
        }
        else if (outCalc < lastOutput - maxChange)
        {
            outCalc = lastOutput - maxChange;
        }
    }

    // 8. Saturation
    float outReal = outCalc;
    if (outReal > outMax)
        outReal = outMax;
    else if (outReal < outMin)
        outReal = outMin;

    // 9. Anti-Windup Strategy
    // The difference (outReal - outCalc) includes both saturation AND ramp limiting.
    // This allows Back-Calc to prevent winding up if we are Ramp Limited!
    if (kb > 0.0)
    {
        // --- Back-Calculation ---
        // Dynamically reduce integral based on saturation difference
        float satDiff = outReal - outCalc;
        integralTerm += iIncrement + (kb * satDiff * dt);
    }
    else
    {
        // --- Clamping ---
        // Only integrate if not saturated OR if error opposes saturation
        bool saturated = (outReal != outCalc);
        bool sameSign = (error > 0 && outReal >= outMax) || (error < 0 && outReal <= outMin);

        if (!saturated || !sameSign)
        {
            integralTerm += iIncrement;
        }

        // Hard clamp the storage variable
        if (integralTerm > outMax)
            integralTerm = outMax;
        else if (integralTerm < outMin)
            integralTerm = outMin;
    }

    lastError = error;
    lastInput = input;
    lastOutput = outReal; // Store for Ramp Rate next cycle
    return outReal;
}

void AdvancedPID::reset()
{
    integralTerm = 0.0;
    lastDTerm = 0.0;
    lastInput = 0.0;
    lastOutput = 0.0;
    lastTime = micros();
}

void AdvancedPID::setMode(PIDMode newMode, float currentOutput)
{
    // Bumpless Transfer: Manual -> Auto
    if (newMode == MODE_AUTO && mode == MODE_MANUAL)
    {
        integralTerm = currentOutput; // Pre-load integral

        // Clamp initial integral
        if (integralTerm > outMax)
            integralTerm = outMax;
        else if (integralTerm < outMin)
            integralTerm = outMin;

        lastOutput = currentOutput;
        lastDTerm = 0.0;
        lastTime = micros();
    }
    mode = newMode;
}

void AdvancedPID::setOutputLimits(float min, float max)
{
    if (min < max)
    {
        outMin = min;
        outMax = max;
    }
}

void AdvancedPID::setTunings(float Kp, float Ki, float Kd, float Kb)
{
    if (Kp < 0 || Ki < 0 || Kd < 0 || Kb < 0)
        return;
    kp = Kp;
    ki = Ki;
    kd = Kd;
    kb = Kb;
}

void AdvancedPID::setDerivativeFilter(float alpha)
{
    dFilterAlpha = constrain(alpha, 0.0, 1.0);
}

void AdvancedPID::setPOnM(bool val) { pOnE = !val; }
void AdvancedPID::setDeadband(float threshold) { deadband = fabs(threshold); }
void AdvancedPID::setIntegralZone(float zone) { iZone = fabs(zone); }

void AdvancedPID::setControllerDirection(PIDDirection dir)
{
    direction = dir;
}

void AdvancedPID::setOutputRampRate(float rate)
{
    outputRampRate = fabs(rate);
}

// Getters
float AdvancedPID::getKp() { return kp; }
float AdvancedPID::getKi() { return ki; }
float AdvancedPID::getKd() { return kd; }
float AdvancedPID::getKb() { return kb; }
PIDMode AdvancedPID::getMode() { return mode; }
PIDDirection AdvancedPID::getDirection() { return direction; }