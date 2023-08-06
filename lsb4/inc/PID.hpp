#ifndef PID_HPP
#define PID_HPP

#pragma pack(push,1) // выравнивание в памяти
struct PIDValue
{
    double P;
    double I;
    double D;
};

#pragma pack(pop) // выравнивание в памяти

class PID
{
    public:
        PID(){};
        PID(double k_p, double k_i, double k_d);
        void setIntegralLimit(int limit) {this->integralLimit = limit;}
        void setSaturationLimit(int min, int max) {this->minValue = min; this->maxValue = max;}
        double update(double inputValue, double targetValue, double dt);
        double saturation(double data, double min, double max);
    private:
        double k_p;
        double k_i;
        double k_d;
        int integralLimit;
        double integral;
        double lastError;
        double minValue;
        double maxValue;
};

#endif