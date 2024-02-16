#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;

#define PI 3.1415926
class Stanley {
private:
    double k;
public:
    Stanley(double k);
    double calTargetIndex(vector<double> robot_state, vector<vector<double>> ref_path);
    double normalizeAngle(double angle);
    vector<double> stanleyControl(vector<double> robot_state, vector<vector<double>> ref_path);
};