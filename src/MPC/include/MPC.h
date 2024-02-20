#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "KinematicModel.h"
#include "OsqpEigen/OsqpEigen.h"
#include <osqp/osqp.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;
using namespace OsqpEigen;

namespace {
    const double MAX_STEER = M_PI / 6.0;
    const double MAX_VEL = 10.0;
}
class MPC {
private:
    int NX, NU, T;
    MatrixXd R = MatrixXd::Identity(NU, NU);
    MatrixXd Rd = MatrixXd::Identity(NU, NU);
    MatrixXd Q = MatrixXd::Identity(NX, NX);
    MatrixXd Qf = Q;
public:
    MPC(int nx, int nu, int t);
    vector<double> LMPC(MatrixXd xref, Vector3d x0, MatrixXd ref_delta, KinematicModel model);
};