#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
}

void KalmanFilter::Predict() {
    /**
     TODO:
     * predict the state
     */
    //    x_ = F_ * x_;
}

void KalmanFilter::Update(const VectorXd &z) {
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
}
