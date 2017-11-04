#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  
  ///* measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  ///* measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
  
  ///* Measurement Matrix Laser
  H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

  ///* Measurement Matrix RADAR
  Hj_ << 0, 0, 0, 0,
         0, 0, 0, 0,
		 0, 0, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	  float rho = measurement_pack.raw_measurements_[0];
	  float phi = measurement_pack.raw_measurements_[1];
	  float rho_d = measurement_pack.raw_measurements_[2];
	  
	  float x = rho * cos(phi);
	  float y = rho * sin(phi);
	  float v_x = rho_d * cos(phi);
	  float v_y = rho_d * sin(phi);
	  
	  ///* put the transformed values into the x vector
	  ekf_.x_ << x,y,v_x,v_x;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	  ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }
	
	///* Check for zero values like in course
	if (fabs(ekf_.x_(0)) < 0.001 and fabs(ekf_.x_(1)) < 0.001){
		ekf_.x_(0) = 0.001;
		ekf_.x_(1) = 0.001;
	}
	
	ekf_.P_ = MatrixXd(4,4);
	ekf_.P_ << 1,0,0,0,
	           0,1,0,0,
			   0,0,1000,0,
			   0,0,0,1000;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  float dt = measurement_pack.timestamp_ - previous_timestamp_;
  previous_timestamp_ = measurement_pack.timestamp_;
  ///* Convert to SI unitsdt
  dt = dt/1000000.0;
  
  ///* state transistion matrix
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1;

  float dt_2 = pow(dt,2);
  float dt_3 = pow(dt,3);
  float dt_4 = pow(dt,4);
  ///* set the acceleration noise components
  float noise_ax = 5.0;
  float noise_ay = 5.0;
  ///* process covariance matrix
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt_4/4.0 * noise_ax, 0, dt_3/2.0 * noise_ax, 0,
	    0, dt_4/4.0 * noise_ay, 0, dt_3/2.0 * noise_ay,
	    dt_3/2.0 * noise_ax, 0, dt_2 * noise_ax, 0,
 	    0, dt_3/2.0 * noise_ay, 0, dt_2 * noise_ay;
			 
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.H_ = H_laser_;
	ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
