#include "matplotlibcpp.h"
#include <cmath>
#include <iostream>
#include "KinematicModel.h"
#include "pid_controller.h"
#include "pure_pursuit.h"

namespace plt = matplotlibcpp;
#define PI 3.1415926

int main()
{
    double lam = 0.1;
    double c = 2;
    KinematicModel model(0.0, 1.0, 0.74, 0.0, 2.2, 0.1); // x,y,psi,v,L,dt
    double Target_speed = 20.0 / 3.6; // m/s
    vector<vector<double>> ref_path(1000, vector<double>(2));
    vector<double> ref_x, ref_y;
    // 生成参考轨迹
    for (int i = 0; i < 1000; i++) {
        ref_path[i][0] = 0.1 * i;
        ref_path[i][1] = 2 * sin(ref_path[i][0] / 3.0);
        ref_x.push_back(ref_path[i][0]);
        ref_y.push_back(ref_path[i][1]);
    }

    vector<double> x_, y_;
    vector<double> robot_state(2);
    PurePursuit pp;
    PID_controller PID(3, 0.001, 30, Target_speed, 22.0 / 3.6, 0.0);
    for (int i = 0; i < 600; i++)
    {
        plt::clf();
        robot_state[0] = model.x;
        robot_state[1] = model.y;
        double a = PID.calOutput(model.v);
        
        double l_d = lam * model.v + c;
        double min_ind = pp.calTargetIndex(robot_state, ref_path, l_d);
        double delta = pp.Pure_Pursuit_Control(robot_state, ref_path[min_ind], l_d, model.psi, 2.2);
        model.updateState(a, delta);
        cout << "Speed: " << model.v << " m/s" << endl;
        x_.push_back(model.x);
        y_.push_back(model.y);

        plt::plot(ref_x, ref_y, "b--");
        plt::plot(x_, y_, "r--");
        plt::grid(true);
        plt::ylim(-5, 5);
        plt::pause(0.01);
    }
    const char* filename = "./pure_pursuit.png";
    plt::save(filename);
    plt::show();
    return 0;
}