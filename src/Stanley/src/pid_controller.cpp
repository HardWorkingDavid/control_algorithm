#include "pid_controller.h"

PID_controller::PID_controller(double Kp, double Ki, double Kd, double target, double upper, double lower) : Kp(Kp),
                                                                                                             Ki(Ki),
                                                                                                             Kd(Kd),
                                                                                                             target(target),
                                                                                                             upper(upper),
                                                                                                             lower(lower) {}
void PID_controller::setTarget(double target) {
    PID_controller::target = target;
}

void PID_controller::setK(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID_controller::setBound(double upper, double lower) {
    this->upper = upper;
    this->lower = lower;
}

double PID_controller::calOutput(double state) {
    this->error = this->target - state;
    double u = this->error * this->Kp + this->sum_error * this->Ki + (this->error - this->pre_error) * this->Kd;
    if (u < this->lower) u = this->lower;
    else if (u > this->upper) u = this->upper;
    this->pre_error = this->error;
    this->sum_error = this->sum_error + this->error;
    return u;
}

void PID_controller::reset() {
    error = 0.0;
    pre_error = 0.0;
    sum_error = 0.0;
}

void PID_controller::setSumError(double sum_error) {
    this->sum_error = sum_error;
}
