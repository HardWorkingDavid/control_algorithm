#include "KinematicModel.h"

KinematicModel::KinematicModel(double x, double y, double psi, double v, double L, double dt) : x(x), y(y), psi(psi),
                                                                                                v(v), L(L), dt(dt) {}

void KinematicModel::updateState(double a, double delta)
{
    x = x + v * cos(psi) * dt;
    y = y + v * sin(psi) * dt;
    psi = psi + v / L * tan(delta) * dt;
    v = v + a * dt;
}    

vector<double> KinematicModel::getState()
{
    return {x, y, psi, v};
}

// 将模型离散化后的状态空间表达
vector<MatrixXd> KinematicModel::stateSpace(double ref_delta, double ref_yaw)
{
    MatrixXd A(3, 3), B(3, 2);
    A << 1.0, 0.0, -v * sin(ref_yaw) * dt,
         0.0, 1.0, v * cos(ref_yaw) * dt,
         0.0, 0.0, 1.0;
    B << cos(ref_yaw) * dt, 0,
         sin(ref_yaw) * dt, 0,
         tan(ref_delta) * dt / L, v * dt / (L * cos(ref_delta) * cos(ref_delta));
    return {A, B};
}
