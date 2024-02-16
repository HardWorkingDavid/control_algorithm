#include "KinematicModel.h"
#include "pid_controller.h"
#include "matplotlibcpp.h"
#include <iostream>
#include <cmath>
#include "stanley.h"

namespace plt = matplotlibcpp;
#define PI 3.1415926

int main()
{
    double Target_speed = 12.0 / 3.6; // m / s
    KinematicModel model(0.0, 1.0, 0.74, 0.0, 2.2, 0.1); // x,y,psi,v,L,dt
    vector<vector<double>> ref_path(1000, vector<double>(3));
    vector<double> ref_x, ref_y;
    // 生成参考轨迹
    for (int i = 0; i < 1000; i++)
    {
        ref_path[i][0] = 0.1 * i;
        ref_path[i][1] = 2 * sin(ref_path[i][0] / 3.0);
        for (int i = 0; i < 999; i++) {
            ref_path[i][2]= atan2((ref_path[i+1][1]-ref_path[i][1]),(ref_path[i+1][0]-ref_path[i][0]));
        }
        ref_x.push_back(ref_path[i][0]);
        ref_y.push_back(ref_path[i][1]);
    }
    vector<double> x_, y_;
    double k = 6.99; // 增益参数
    vector<double> robot_state(4);
    Stanley stanley(k);
    PID_controller PID(2, 0.001, 30, Target_speed, 16.0 / 3.6, 0.0);
    for (int i = 0; i < 600; i++)
    {
        plt::clf();
        robot_state = model.getState();
        double a = PID.calOutput(model.v);
        vector<double> delta_index = stanley.stanleyControl(robot_state, ref_path);
        
        model.updateState(a, delta_index[0]);
        cout << "Speed: " << model.v << " m/s" << endl;
        x_.push_back(model.x);
        y_.push_back(model.y);

        plt::plot(ref_x, ref_y, "b--");
        plt::plot(x_, y_, "r--");
        plt::grid(true);
        plt::ylim(-3, 3);
        plt::pause(0.01);
        if (delta_index[1] >= ref_path.size() - 1) break;
    }
    const char* filename = "./stanley.png";
    plt::save(filename);
    plt::show();
    return 0;
}