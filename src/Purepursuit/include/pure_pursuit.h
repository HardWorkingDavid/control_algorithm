#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;
class PurePursuit {
public:
    double calTargetIndex(vector<double>robot_state, vector<vector<double>> ref_path, double l_d);

    double Pure_Pursuit_Control(vector<double> robot_state, vector<double> current_ref_point, double l_d, double psi, double L);
};
