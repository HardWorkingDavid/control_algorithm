#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <limits>
#include <vector>
#include <fstream>
#include <string>
#include "OSQP.h"

using namespace std;

int main()
{
    Trajectory_Smoothing Smoothing;
    Smoothing.Write();

    // 轨迹点
    unsigned int n;
    n = min(global_path_x.size(), global_path_y.size());
    plt::clf();

    plt::scatter(global_path_x, global_path_y);

    //计算H（hessian）
    Eigen::MatrixXd A1 = Smoothing.wei_du_MatrixXd(2*n, 2*n-4, Smoothing.A1);
    Eigen::MatrixXd A2 = Smoothing.wei_du_MatrixXd(2*n, 2*n-2, Smoothing.A2);
    Eigen::MatrixXd A3 = Smoothing.wei_du_MatrixXd(2*n, 2*n, Smoothing.A3);
    Eigen::MatrixXd H;

    A1 = Smoothing.create_A1(A1);
    A2 = Smoothing.create_A2(A2);
    A3 = Smoothing.create_A3(A3);
    H = (Smoothing.Weight_smooth * A1 * A1.transpose() + Smoothing.Weight_length * A2 * A2.transpose() + Smoothing.Weight_ref * A3 * A3.transpose());

    //计算f（gradient）
    Eigen::VectorXd h = Smoothing.wei_du_VectorXd(2*n, Smoothing.h);
    Eigen::VectorXd f = Smoothing.wei_du_VectorXd(2*n, Smoothing.f);
    h = Smoothing.create_h(h);
    f = Smoothing.Weight_ref * h; 

    //计算上下限
    Eigen::VectorXd lb = Smoothing.wei_du_VectorXd(2*n, Smoothing.lb);   
    Eigen::VectorXd ub = Smoothing.wei_du_VectorXd(2*n, Smoothing.ub); 
    lb = Smoothing.create_lb(lb);      
    ub = Smoothing.create_ub(ub);

    //OSQP矩阵赋值
    Eigen::SparseMatrix<double> hessian(2*n,2*n);        //P: n*n正定矩阵,必须为稀疏矩阵SparseMatrix
    Eigen::SparseMatrix<double> linearMatrix(2*n,2*n);   //A: m*n矩阵,必须为稀疏矩阵SparseMatrix
    for (int i = 0; i < 2 * n; i++){  
        for (int j = 0; j < 2 * n; j++){
            hessian.insert(i, j) = H(i,j); //注意稀疏矩阵的初始化方式,无法使用<<初始化
        }
    }

    for (int i = 0; i < 2 * n; i++){         //A（linearMatrix）矩阵
        linearMatrix.insert(i, i) = 1; //注意稀疏矩阵的初始化方式,无法使用<<初始化
    }

    Eigen::VectorXd gradient(2 * n);                    //Q: n*1向量
    Eigen::VectorXd lowerBound(2 * n);                  //L: m*1下限向量
    Eigen::VectorXd upperBound(2 * n);                  //U: m*1上限向量
    gradient << f;

    lowerBound << lb;
    upperBound << ub;

    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(2 * n);   //变量数n
    solver.data()->setNumberOfConstraints(2 * n); //约束数m
    if (!solver.data()->setHessianMatrix(hessian))
        return 1;
    if (!solver.data()->setGradient(gradient))
        return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
        return 1;
    if (!solver.data()->setLowerBound(lowerBound))
        return 1;
    if (!solver.data()->setUpperBound(upperBound))
        return 1;
 
    // instantiate the solver
    if (!solver.initSolver())
        return 1;
    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if (!solver.solve())
    {
        return 1;
    }    
    QPSolution = solver.getSolution();

    std::vector<double> X_solve_;
    X_solve_.reserve(QPSolution.size() / 2);

    for (int i = 0; i < QPSolution.size(); i += 2) {
        X_solve_.push_back(QPSolution(i));
    }

    std::vector<double> Y_solve_;
    Y_solve_.reserve((QPSolution.size() + 1)  / 2);

    for (int i = 1; i < QPSolution.size(); i += 2) {
        Y_solve_.push_back(QPSolution(i));
    }

    plt::plot(X_solve_, Y_solve_, "r");
    // 设置图形标题和轴标签
    plt::title("Lane Line");
    plt::xlabel("X");
    plt::ylabel("Y");

    // 显示图形
    plt::show();

    return 0;
}