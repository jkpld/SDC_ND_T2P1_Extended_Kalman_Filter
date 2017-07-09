#include "FusionEKF.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Constructor.
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;
}

// Destructor.
FusionEKF::~FusionEKF() {}

// State transition matrix.
inline MatrixXd FusionEKF::F(double dt) {
  MatrixXd F = MatrixXd::Identity(4,4);
  F(0,2) = dt;
  F(1,3) = dt;
  return F;
}

// Process covariance matrix.
inline MatrixXd FusionEKF::Q(double dt) {
  double noise_ax = 9;
  double noise_ay = 9;
  double dt2o2 = dt*dt/2;

  Eigen::DiagonalMatrix<double, 2> Qnu(noise_ax, noise_ay);

  MatrixXd G = MatrixXd::Zero(4,2);
  G(0,0) = dt2o2;
  G(1,1) = dt2o2;
  G(2,0) = dt;
  G(3,1) = dt;

  MatrixXd Q = G * Qnu * G.transpose();
  return Q;
}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // convert from measurment space to state space.
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      ekf_.x_ = radar_.measure_to_state(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ = lidar_.measure_to_state(measurement_pack.raw_measurements_);
    }

    // initialize state covariance matrix P
  	ekf_.P_ = MatrixXd::Zero(4, 4);
    ekf_.P_.diagonal() << 1, 1, 1000, 1000;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Update F and Q using dt
  double dt = measurement_pack.timestamp_ - previous_timestamp_;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = F(dt);
  ekf_.Q_ = Q(dt);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.Update(measurement_pack.raw_measurements_, radar_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_, lidar_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
