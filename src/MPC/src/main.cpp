#include "MPC.h"
#include "Reference_path.h"
#include "KinematicModel.h"
#include "matplotlibcpp.h"
#include "pid_controller.h"

namespace plt = matplotlibcpp;

int main()
{
    parameters param;
    param.NX = 3;
    param.NU = 2;
    param.T = 2;
    param.L = 2.2;
    param.dt = 0.1;
    double Target_speed = 8.0 / 3.6;
    MPC mpc(param.NX, param.NU, param.T);

    Eigen::VectorXd x0(param.NX);

    x0 << 0.0, -3.0, 0.0; // x y yaw
    vector<double> robot_state = {0.0, -3.0, 0.0, 0.0};
    double dt = 0.1;
    double L = 2.2;
    ReferencePath reference_path;
    auto reference_trajectory = reference_path.calc_ref_trajectory(robot_state, param, 1.0);
    KinematicModel model(x0(0), x0(1), x0(2), 0.0, L, dt);
    PID_controller PID(3, 0.001, 30, Target_speed, 15.0 / 3.6, 0.0);

    std::vector<double> x_;
    std::vector<double> y_;

    for (int i = 0; i < param.T; i++)
    {
        Eigen::MatrixXd xref = reference_trajectory.xref;
        Eigen::VectorXd xref_i = xref.col(i); // 3 x 1
        Eigen::VectorXd ref_delta = reference_trajectory.dref.col(i); // 2 x 1

        std::vector<double> control_input = mpc.LMPC(xref_i, x0, ref_delta, model);
        cout << control_input[1] << endl;
        double a = PID.calOutput(model.v);

        model.updateState(a, control_input[1]);
        cout << "Speed: " << model.v << " m/s" << endl;

        x_.push_back(model.getState()[0]);
        y_.push_back(model.getState()[1]);

        const auto state = model.getState();
        x0 << state[0], state[1], state[2];

        //画参考轨迹
        plt::plot(reference_path.ref_x, reference_path.ref_y, "b--");
        plt::grid(true);
        plt::ylim(-5, 5);
        plt::plot(x_, y_, "r");
        plt::pause(0.01);
    }
    const char* filename = "./MPC.png";
    plt::save(filename);
    plt::show();
    return 0;
}
