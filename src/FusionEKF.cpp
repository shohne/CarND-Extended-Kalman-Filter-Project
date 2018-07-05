


#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define PI 3.14159265

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    
    R_laser_ << 0.0225, 0,
    0, 0.0225;
    
    R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
//    cout << "x_ (bef) = " << ekf_.x_ << endl;
//    cout << "P_ (bef) = " << ekf_.P_ << endl;

    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        ekf_.x_ = VectorXd(4);

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        	float r = measurement_pack.raw_measurements_(0);
        	float theta = measurement_pack.raw_measurements_(1);
            ekf_.x_ << r*cos(theta), r*sin(theta), 0, 0;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
        }
        ekf_.P_ = Eigen::MatrixXd(4,4);
        ekf_.P_ <<
               1,    0,    0,    0,
               0,    1,    0,    0,
               0,    0,  100,    0,
               0,    0,    0,  100;

        is_initialized_ = true;
        previous_timestamp_ = measurement_pack.timestamp_;
        return;
    }
    
    /**
     TODO:
     * Update the state transition matrix F according to the new elapsed time.
     - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    
    double deltaT = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    double deltaT2 = deltaT  * deltaT;
    double deltaT3 = deltaT2 * deltaT;
    double deltaT4 = deltaT3 * deltaT;
    double noise_ax = 9;
    double noise_ay = 9;
    
    F_ = Eigen::MatrixXd(4,4);
    F_ <<
        1.0, 0.0, deltaT,    0.0,
        0.0, 1.0,    0.0, deltaT,
        0.0, 0.0,    1.0,    0.0,
        0.0, 0.0,    0.0,    1.0;

    Q_ = Eigen::MatrixXd(4,4);
    Q_ <<
           		deltaT4 * noise_ax / 4.0,                          0,  deltaT3 * noise_ax / 2.0,                             0,
                                       0,   deltaT4 * noise_ay / 4.0,                         0,      deltaT3 * noise_ay / 2.0,
                deltaT3 * noise_ax / 2.0,                          0,  deltaT2 * noise_ax / 2.0,                             0,
                                       0,   deltaT3 * noise_ay / 2.0,                         0,      deltaT2 * noise_ay / 2.0;

    ekf_.P_ = F_ * ekf_.P_ * F_.transpose() + Q_;
    ekf_.x_ = F_ * ekf_.x_;

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    
    /**
     TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     */
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
//        cout << "RADAR" << endl;

        Eigen::VectorXd z = Eigen::VectorXd(3);
        z << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), measurement_pack.raw_measurements_(2);
        Eigen::MatrixXd H = tools.CalculateJacobian(ekf_.x_);
        Eigen::VectorXd y = Eigen::VectorXd(3);
        y(0) = z(0) - sqrt(ekf_.x_(0)*ekf_.x_(0) + ekf_.x_(1)*ekf_.x_(1));

//        double theta = atan(ekf_.x_(1) / ekf_.x_(0));
        double theta = arctan_with_xy(ekf_.x_(0), ekf_.x_(1));

        /*
        if (z(1) < -PI || z(1) > PI) {
        	cout << endl << "ERROR IN measure " << measurement_pack.raw_measurements_(1);
        }
        */
//        cout << endl << measurement_pack.timestamp_ << "\tx: " << ekf_.x_(0) << "\ty: " << ekf_.x_(1) << "\tt: " << ekf_.x_(1) / ekf_.x_(0) << "\ttheta: " << theta << "\tmeasure: " << z(1);
        y(1) = z(1) - theta;
        y(2) = z(2) - (ekf_.x_(0)*ekf_.x_(2) + ekf_.x_(1)*ekf_.x_(3)) / sqrt(ekf_.x_(0)*ekf_.x_(0) + ekf_.x_(1)*ekf_.x_(1));
        
        Eigen::MatrixXd S = H * ekf_.P_ * H.transpose() + R_radar_;
        Eigen::MatrixXd K = ekf_.P_ * H.transpose() * S.inverse();
        ekf_.x_ = ekf_.x_ + K * y;
        ekf_.P_ = (MatrixXd::Identity(4, 4) - K * H) * ekf_.P_;
        
    } else {
//        cout << "LIDAR" << endl;

        Eigen::VectorXd z = Eigen::VectorXd(2);
        z << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1);
        
        Eigen::MatrixXd H = Eigen::MatrixXd(2,4);
        H <<
            1,0,0,0,
            0,1,0,0;
        Eigen::VectorXd y = Eigen::VectorXd(2);
        y = z - H * ekf_.x_;
        
        Eigen::MatrixXd S = H * ekf_.P_ * H.transpose() + R_laser_;
        Eigen::MatrixXd K = ekf_.P_ * H.transpose() * S.inverse();
        ekf_.x_ = ekf_.x_ + K * y;
        ekf_.P_ = (MatrixXd::Identity(4, 4) - K * H) * ekf_.P_;
    }
    
    // print the output
//    cout << "x_ (after) = " << ekf_.x_ << endl;
//    cout << "P_ (after) = " << ekf_.P_ << endl;
    previous_timestamp_ = measurement_pack.timestamp_;
}

double FusionEKF::arctan_with_xy(const double x, const double y) {
	double x_abs = abs(x);
	double y_abs = abs(y);

	double r = sqrt(x_abs*x_abs + y_abs*y_abs);

	double x_norm = x_abs / r;
	double y_norm = y_abs / r;

	double theta = 0;

	if (y_norm < x_norm) {
		theta = asin(y_norm);
	}
	else {
		theta = acos(x_norm);
	}

	if (x < 0 && y > 0) theta = PI - theta;
	if (x > 0 && y < 0) theta =    - theta;
	if (x < 0 && y < 0) theta = theta - PI;

	return theta;
}

