#include "LQR.h"
#include "KinematicModel.h"
#include "matplotlibcpp.h"
#include "Reference_path.h"
#include "pid_controller.h"

namespace plt = matplotlibcpp;

int main()
{
    int N = 500; // 迭代范围
    double Target_speed = 7.2 / 3.6;
    MatrixXd Q(3,3);
    Q << 3,0,0,
         0,3,0,
         0,0,3;
    MatrixXd R(2,2);
    R << 2.0,0.0,
         0.0,2.0;
    //保存机器人（小车）运动过程中的轨迹
    vector<double> x_, y_;
    ReferencePath referencePath;
    KinematicModel model(0, 1.0, 0, 0, 2.2, 0.1);
    PID_controller PID(3, 0.001, 30, Target_speed, 15.0 / 3.6, 0.0);
    LQR lqr(N);
    vector<double> robot_state;

    for (int i = 0; i < 700; i++)
    {
        plt::clf();
        robot_state = model.getState();
        vector<double> one_trial = referencePath.calcTrackError(robot_state);
        double k = one_trial[1], ref_yaw = one_trial[2], s0 = one_trial[3];

        double ref_delta = atan2(k * model.L, 1); // L = 2.2
        vector<MatrixXd> state_space = model.stateSpace(ref_delta, ref_yaw);

        double delta = lqr.LQRControl(robot_state, referencePath.ref_path, s0, state_space[0], state_space[1], Q, R);
        delta = delta + ref_delta;
        double a = PID.calOutput(model.v);
        model.updateState(a, delta);
        cout << "Speed: " << model.v << " m/s" << endl;

        x_.push_back(model.x);
        y_.push_back(model.y);
        //画参考轨迹
        plt::plot(referencePath.ref_x, referencePath.ref_y, "b--");
        plt::grid(true);
        plt::ylim(-5, 5);
        plt::plot(x_, y_, "r");
        plt::pause(0.01);
    }
    const char* filename = "./LQR.png";
    plt::save(filename);
    plt::show();
    return 0;
}
