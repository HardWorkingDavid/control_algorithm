#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

class KinematicModel {
public:
    KinematicModel();
    KinematicModel(double x, double y, double psi, double v, double L, double dt);

    vector<double> getState();
    void updateState(double a, double delta);
    vector<MatrixXd> stateSpace(double ref_delta, double ref_yaw);
public:
    double x, y, psi, v, L, dt;
};