#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    for (int j=0; j<estimations.size(); j++) {
        VectorXd d = estimations[j] - ground_truth[j];
        VectorXd d2 = d.cwiseProduct(d);
        rmse += d2;
    }
    rmse = rmse / (double)estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd jacobian(3,4);
    
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);
    
    double p2 = px*px + py*py;
    double p = sqrt(p2);
    double p3 = p2 * p;
    
    if (abs(p) < 0.000001) {
        cout << "error computing jacobian, division by zero" << endl;
        return jacobian;
    }
    
    jacobian(0,0) = px / p;
    jacobian(0,1) = py / p;
    jacobian(0,2) = 0.0;
    jacobian(0,3) = 0.0;
    
    jacobian(1,0) = -py / p2;
    jacobian(1,1) =  px / p2;
    jacobian(1,2) =  0.0;
    jacobian(1,3) =  0.0;
    
    jacobian(2,0) = py * (vx * py - vy * px) / p3;
    jacobian(2,1) = px * (vy * px - vx * py) / p3;
    jacobian(2,2) = px / p;
    jacobian(2,3) = py / p;
    
    return jacobian;
}
