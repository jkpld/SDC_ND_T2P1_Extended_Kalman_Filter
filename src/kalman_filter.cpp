#include "kalman_filter.h"
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;



KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::PrintInfo(string name) {
  cout << endl << "KalmanFilter::" << name << endl << endl;
  cout << "x:" << endl << x_ << endl;
  cout << "P:" << endl << P_ << endl;
  // cout << "F:" << endl << F_ << endl;
  // cout << "Q:" << endl << Q_ << endl;
  // cout << "I:" << endl << I_ << endl;
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;

  I_ = MatrixXd::Identity(P_.rows(), P_.cols());

  // PrintInfo("Init");
}

void KalmanFilter::Predict() {

  // PrintInfo("Predict-start");

  // Predict new state and covariance
  x_ = F_*x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

  // PrintInfo("Predict-end");
}

void KalmanFilter::Update(const VectorXd &z, Sensor &sensor) {

  // PrintInfo("Update-start");

  // Compute filter gain
  MatrixXd H = sensor.H(x_);
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + sensor.R;
  MatrixXd K = P_ * Ht * S.inverse();

  // difference between measurment and prediction
  VectorXd dz = sensor.meas_state_difference(z, x_);

  // new state
  x_ = x_ + (K*dz);
  P_ = (I_ - K*H) * P_;

  // PrintInfo("Update-end");
}
