#include "kalman_filter.h"

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

}

void KalmanFilter::Predict() {
  // Predict new state and covariance
  x_ = F_*x_;
  P_ = F_ * P * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const Sensor &sensor) {

  // Compute filter gain
  MatrixXd H = sensor.H(z);
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + sensor.R;
  MatrixXd K = P * Ht * S.inverse();

  // difference between measurment and prediction
  VectorXd dz = z - sensor.state_to_measure(x_);

  // new state
  x_ = x_ + (K*dz);
  P_ = (I_ - K*H) * P_;
}
