#pragma once
#include <iostream>

using namespace std;

class PID_controller {
private:
    double Kp, Ki, Kd, target, upper, lower;
    double error = 0.0, pre_error = 0.0, sum_error = 0.0;
public:
    PID_controller(double Kp, double Ki, double Kd, double target, double upper, double lower);

    void setTarget(double target);

    void setK(double Kp, double Ki, double Kd);

    void setBound(double upper, double lower);

    double calOutput(double state);

    void reset();

    void setSumError(double sum_error);
};
