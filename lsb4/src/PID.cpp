#include "PID.hpp"

PID::PID(double k_p, double k_i, double k_d, double minValue, double maxValue)
{
    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;
    this->integral = 0;
    this->lastError = 0;
    this->integralLimit = 300;
    this->minValue = minValue;
    this->maxValue = maxValue;
}

double PID::update(double inputValue, double targetValue, double dt)
{
    double res = 0;
    PIDValue pidData = {0};
    double error = targetValue - inputValue;

    this->integral = saturation( (this->integral + (error * dt)), 0, this->integralLimit);
    // this->integral += error * dt;
    pidData.P = this->k_p * error;
    pidData.I = this->k_i * this->integral;
    pidData.D = this->k_d * (error - this->lastError)/dt;

    this->lastError = error;
    double test = pidData.P + pidData.I + pidData.D;
    res = saturation((pidData.P + pidData.I + pidData.D), this->minValue, this->maxValue);

    return res;
}

double PID::saturation(double data, double min, double max)
{
    if (data > max)
        return max;
    else if (data < min)
        return min;
    else
        return data;    
}
