#include "kalman_filter.h"
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;

  I_ = MatrixXd::Identity(P_.rows(), P_.cols());

  cout << "Initialized Kalman Filter" << endl;
  cout << x_ << endl;
  cout << P_ << endl;
  cout << F_ << endl;
  cout << Q_ << endl;
  cout << I_ << endl;

}

void KalmanFilter::Predict() {
  // Predict new state and covariance
  x_ = F_*x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, Sensor &sensor) {

  cout << "In KalmanFilter::Update()" << endl << endl;
  cout << "Measurment:" << endl;
  cout << z;
  cout << "Prediction from state" << endl;

  // Compute filter gain
  MatrixXd H = sensor.H(z);
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + sensor.R;
  MatrixXd K = P_ * Ht * S.inverse();

  // difference between measurment and prediction
  VectorXd dz = z - sensor.state_to_measure(x_);

  // new state
  x_ = x_ + (K*dz);
  P_ = (I_ - K*H) * P_;
}
