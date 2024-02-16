#include "stanley.h"

Stanley::Stanley(double k)
{
    this->k = k;
}

// 搜索目标邻近路点
// robot_state 当前机器人位置 ref_path 参考轨迹(数组)
double Stanley::calTargetIndex(vector<double> robot_state, vector<vector<double>> ref_path)
{
    vector<double> dists;
    for (vector<double> xy : ref_path)
    {
        double dist = sqrt(pow(xy[0] - robot_state[0], 2) + pow(xy[1] - robot_state[1], 2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(), dists.end()) - dists.begin();
}

double Stanley::normalizeAngle(double angle)
{
    while (angle > PI)
    {
        angle -= 2.0 * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0 * PI;
    }
    return angle;
}

// stanley 控制
// robot_state：x,y,yaw,v ref_path：x,y,theta
// return 控制量 + 目标点索引
vector<double> Stanley::stanleyControl(vector<double> robot_state, vector<vector<double>> ref_path)
{
    double current_target_index = calTargetIndex(robot_state, ref_path);
    vector<double> current_ref_point;

    if (current_target_index >= ref_path.size())
    {
        current_target_index = ref_path.size() - 1;
        current_ref_point = ref_path[ref_path.size() - 1];
    } else {
        current_ref_point = ref_path[current_target_index];
    }
    double e_y;
    double psi_t = current_ref_point[2];

    if ((robot_state[0] - current_ref_point[0]) * psi_t - (robot_state[1] - current_ref_point[1]) > 0)
    {
        e_y = sqrt(pow(current_ref_point[0]-robot_state[0],2)+pow(current_ref_point[1]-robot_state[1],2));
    }
    else 
    {
        e_y = -sqrt(pow(current_ref_point[0]-robot_state[0],2)+pow(current_ref_point[1]-robot_state[1],2));
    }
    double psi = robot_state[2];
    double v = robot_state[3];
    double theta_e = psi_t - psi;
    double delta_e = atan2(k * e_y, v);
    double delta = normalizeAngle(delta_e + theta_e);

    // 限制车轮转角[-30, 30]
    if (delta > PI / 6.0) delta = PI / 6.0;
    else if (delta < -PI / 6.0) delta = -PI / 6.0;

    return {delta, current_target_index};
}
