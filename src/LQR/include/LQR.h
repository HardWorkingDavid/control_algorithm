#pragma once

#define EPS 1.0e-4
#include <Eigen/Dense>
#include <vector>
#include <iostream>
using namespace std;
using namespace Eigen;

class LQR {
private:
    int N;
public:
    LQR(int n);

    MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
    double LQRControl(vector<double> robot_state, vector<vector<double>> ref_path, double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
};