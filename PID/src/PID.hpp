#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float interval);

    void setInputLimits(float inMin, float inMax);
    void setOutputLimits(float outMin, float outMax);

    void setSetPoint(float sp);
    void setProcessValue(float pv);

    float compute();

private:
    float kp, ki, kd;
    float interval;
    float setPoint;
    float processValue;

    float integral, previousError;

    float outMin, outMax;
};

#endif