#include "LQR.h"

LQR::LQR(int n) : N(n) {}
// 解代数里卡提方程
MatrixXd LQR::calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R)
{
    MatrixXd Qf = Q;
    MatrixXd P_old = Qf;
    MatrixXd P_new;
    // P _{new} =Q+A ^TPA−A ^TPB(R+B ^T PB) ^{−1}B ^TPA
    for (int i = 0; i < N; i++)
    {
        P_new = Q + A.transpose() * P_old * A - A.transpose() * P_old * B * (R + B.transpose() * P_old * B).inverse() * B.transpose() * P_old * A;
        if ((P_new - P_old).maxCoeff() < EPS && (P_old - P_new).maxCoeff() < EPS) break;
        P_old = P_new;
    }
    return P_new;
}

double LQR::LQRControl(vector<double> robot_state, vector<vector<double>> ref_path, double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R)
{
    MatrixXd X(3, 1);
    X << robot_state[0] - ref_path[s0][0],
         robot_state[1] - ref_path[s0][1],
         robot_state[2] - ref_path[s0][2];
    MatrixXd P = calRicatti(A, B, Q, R);
    // K=(R + B^TP_{new}B)^{-1}B^TP_{new}A
    MatrixXd K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    MatrixXd u = -K * X; // [v - ref_v, delta - ref_delta]
    return u(1, 0);
}