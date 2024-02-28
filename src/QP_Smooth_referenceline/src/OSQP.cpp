#include "OSQP.h"

vector<double> global_path_x;
vector<double> global_path_y;

int Trajectory_Smoothing::Write() {
    std::ifstream input_x("/home/bigdavid/视频/Control_algorithm/src/QP_Smooth_referenceline/src/global_path_x.txt");
    if (!input_x.is_open()) {
        std::cout << "Failed to open the file." << std::endl;
        return 1;
    }

    std::string line_x;
    while (std::getline(input_x, line_x)) {
        double value = std::stod(line_x);
        global_path_x.push_back(value);
        if(global_path_x.size() > 500) break;
    }
    input_x.close();

    std::ifstream input_y("/home/bigdavid/视频/Control_algorithm/src/QP_Smooth_referenceline/src/global_path_y.txt");
    if (!input_y.is_open()) {
        std::cout << "Failed to open the file." << std::endl;
        return 1;
    }

    std::string line_y;
    while (std::getline(input_y, line_y)) {
        double value = std::stod(line_y);
        global_path_y.push_back(value);
        if(global_path_y.size() > 500) break;
    }
    input_y.close();
    return 0;
}

//为矩阵赋予相应维度零矩阵
Eigen::MatrixXd Trajectory_Smoothing::wei_du_MatrixXd(int i, int j, Eigen::MatrixXd& MatrixXd_){

    MatrixXd_ = Eigen::MatrixXd::Zero(i, j);
    return MatrixXd_;
}

//为向量赋予相应维度零矩阵
Eigen::VectorXd Trajectory_Smoothing::wei_du_VectorXd(int i, Eigen::VectorXd& VectorXd_){

    VectorXd_ = Eigen::VectorXd::Zero(i);
    return VectorXd_;
}

//填充矩阵和向量
Eigen::MatrixXd Trajectory_Smoothing::create_A1(Eigen::MatrixXd& A1){
    for (int i=0; i<A1.cols(); i+=2){  //取列
        A1(i,i)=1;A1(i,i+1)=0;
        A1(i+1,i)=0;A1(i+1,i+1)=1;
        A1(i+2,i)=-2;A1(i+2,i+1)=0;
        A1(i+3,i)=0;A1(i+3,i+1)=-2;
        A1(i+4,i)=1;A1(i+4,i+1)=0;
        A1(i+5,i)=0;A1(i+5,i+1)=1;
    }
    return A1;
}

Eigen::MatrixXd Trajectory_Smoothing::create_A2(Eigen::MatrixXd& A2){
    for (int i=0; i<A2.cols(); i+=2){   
        A2(i,i)=1;A2(i,i+1)=0;
        A2(i+1,i)=0;A2(i+1,i+1)=1;
        A2(i+2,i)=-1;A2(i+2,i+1)=0;
        A2(i+3,i)=0;A2(i+3,i+1)=-1;
    } 
    return A2;
}

Eigen::MatrixXd Trajectory_Smoothing::create_A3(Eigen::MatrixXd& A3){
    for (int i=0; i<A3.cols(); i++){   
        A3(i,i)=1;
    }
    return A3;
}

Eigen::VectorXd Trajectory_Smoothing::create_h(Eigen::VectorXd& h){
    for (int i=0; i<h.rows(); i+=2){
        h(i) = -2*global_path_x[i/2];
        h(i+1) = -2*global_path_y[i/2];
    }
    return h;
}

Eigen::VectorXd Trajectory_Smoothing::create_lb(Eigen::VectorXd& lb){
    for (int i=0; i<lb.rows(); i+=2){   
        lb(i)=(global_path_x[i/2]-buff);
        lb(i+1)=(global_path_y[i/2]-buff);
    }
    return lb;

}

Eigen::VectorXd Trajectory_Smoothing::create_ub(Eigen::VectorXd& ub){
    for (int i=0; i<ub.rows(); i+=2){   
        ub(i)=(global_path_x[i/2]+buff);
        ub(i+1)=(global_path_y[i/2]+buff);
    }
    return ub;
}
