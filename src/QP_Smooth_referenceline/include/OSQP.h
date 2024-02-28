#pragma once

#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"
#include <limits>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "math.h"
using namespace std;
extern std::vector<double> global_path_x;
extern std::vector<double> global_path_y;

namespace plt = matplotlibcpp;

class Trajectory_Smoothing {
public:
    double buff = 0.1;

    int Weight_smooth = 1;
    int Weight_length = 1;
    int Weight_ref = 1;
    int n; // 轨迹点

    //各个代价下的矩阵
    Eigen::MatrixXd A1;
    Eigen::MatrixXd A2;
    Eigen::MatrixXd A3;

    Eigen::VectorXd h;
    Eigen::MatrixXd H;
    Eigen::VectorXd f;

    //上下边界约束
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;

    int Write(); // 用于读取txt文档中的轨迹X Y坐标点
    //为矩阵赋予相应维度零矩阵
    Eigen::MatrixXd wei_du_MatrixXd(int i, int j, Eigen::MatrixXd& MatrixXd_);

    //为向量赋予相应维度零矩阵
    Eigen::VectorXd wei_du_VectorXd(int i, Eigen::VectorXd& VectorXd_);

    //得到各个矩阵维度
    Eigen::MatrixXd get_MatrixXd(int rows, int cols);

    //得到各个向量维度
    Eigen::VectorXd get_VectorXd(int rows, int cols);

    //构建各A1矩阵乘相关权重用以计算H
    Eigen::MatrixXd create_A1(Eigen::MatrixXd& A1);

    //构建各A2矩阵乘相关权重用以计算H
    Eigen::MatrixXd create_A2(Eigen::MatrixXd& A2);

    //构建各A3矩阵乘相关权重用以计算H
    Eigen::MatrixXd create_A3(Eigen::MatrixXd& A3);

    //构建h乘相关权重用以计算f
    Eigen::VectorXd create_h(Eigen::VectorXd& h);

    //构建约束下限lb
    Eigen::VectorXd create_lb(Eigen::VectorXd& lb);

    //构建约束上限lu
    Eigen::VectorXd create_ub(Eigen::VectorXd& lu);
};