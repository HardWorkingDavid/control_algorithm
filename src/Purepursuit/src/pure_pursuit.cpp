#include "pure_pursuit.h"

// 计算邻近路点 (G_x, G_y)
// robot_state:当前机器人位置 ref_path:参考轨迹 l_d:前向距离
double PurePursuit::calTargetIndex(vector<double> robot_state, vector<vector<double>> ref_path, double l_d)
{
    vector<double> dists;
    for (vector<double> xy : ref_path)
    {
        double dist = sqrt(pow(xy[0] - robot_state[0], 2) + pow(xy[1] - robot_state[1], 2));
        dists.push_back(dist);
    }
    double min_index = min_element(dists.begin(), dists.end()) - dists.begin();
    double delta_l = sqrt(pow(ref_path[min_index][0] - robot_state[0], 2) + pow(ref_path[min_index][1] - robot_state[1], 2));

    while (l_d > delta_l && min_index < ref_path.size() - 1)
    {
        delta_l = sqrt(pow(ref_path[min_index + 1][0] - robot_state[0], 2) + pow(ref_path[min_index + 1][1] - robot_state[1], 2));
        min_index += 1;
    }
    return min_index;
}

// Pure Pursuit Control
// robot_state 当前机器人位置; current_ref_point 参考轨迹点 ; l_d 前向距离 ; psi 机器人航向角 ; L 轴距 ; return 转角控制量
double PurePursuit::Pure_Pursuit_Control(vector<double> robot_state, vector<double> current_ref_point, double l_d, double psi, double L)
{
    double alpha = atan2(current_ref_point[1] - robot_state[1], current_ref_point[0] - robot_state[0]) - psi;
    double delta = atan2(2 * L * sin(alpha), l_d);
    return delta;
}
