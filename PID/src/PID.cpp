#include <PID.hpp>

PID::PID(float kp, float ki, float kd, float interval) 
    : kp(kp), ki(ki), kd(kd), interval(interval), integral(0), previousError(0) {
}

void PID::setInputLimits(float inMin, float inMax) {
    // 未使用。必要に応じて追加可能。
}

void PID::setOutputLimits(float outMin, float outMax) {
    this->outMin = outMin;
    this->outMax = outMax;
}

void PID::setSetPoint(float sp) {
    this->setPoint = sp;
}

void PID::setProcessValue(float pv) {
    this->processValue = pv;
}

float PID::compute() {
    float error = setPoint - processValue;
    integral += error * interval;
    float derivative = (error - previousError) / interval;

    float output = (kp * error) + (ki * integral) + (kd * derivative);

    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;

    previousError = error;

    return output;
}