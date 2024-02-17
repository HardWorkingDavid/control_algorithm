#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define PI 3.1415926

struct refTraj
{
    MatrixXd xref, dref;
    int ind;
};

struct parameters
{
    int L;
    int NX, NU, T;
    double dt;
};

class ReferencePath
{
public:
    ReferencePath();
    vector<double> calcTrackError(vector<double> robot_state);
    double normalizeAngle(double angle);

    // 计算参考轨迹点，统一化变量数组，便于后面MPC优化使用.
    refTraj calc_ref_trajectory(vector<double> robot_state, parameters param, double dl = 1.0);

public:
    vector<vector<double>> ref_path; // x, y, 切线方向, k
    vector<double> ref_x, ref_y;
};