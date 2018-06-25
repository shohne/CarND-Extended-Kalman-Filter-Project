#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    
    VectorXd rmse(4);
    for (int i=0; i<4; i++) {
        double z = 0;
        int t = 0;
        for (auto e : estimations) {
            z = e(i)*e(i);
            t++;
        }
        rmse(i) = sqrt(z/(double)(t > 0 ? t : 1));
    }
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd jacobian(4,4);
    jacobian(0,0) = 0.0;
    jacobian(0,1) = 0.1;
    jacobian(0,2) = 0.2;
    jacobian(0,3) = 0.3;
    jacobian(1,0) = 1.0;
    jacobian(1,1) = 1.1;
    jacobian(1,2) = 1.2;
    jacobian(1,3) = 1.3;
    jacobian(2,0) = 2.0;
    jacobian(2,1) = 2.1;
    jacobian(2,2) = 2.2;
    jacobian(2,3) = 2.3;
    jacobian(3,0) = 3.0;
    jacobian(3,1) = 3.1;
    jacobian(3,2) = 3.2;
    jacobian(3,3) = 3.3;
    return jacobian;
}
