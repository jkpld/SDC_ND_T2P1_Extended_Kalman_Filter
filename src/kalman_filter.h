#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "Sensors.h"


class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // Constructor
  KalmanFilter();

  // Destructor
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance using the process
   * model
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   * @param sensor A derived Sensor class that defines sensor specific
   *   parameters: H, R, Jacobian
   */
  void Update(const Eigen::VectorXd &z, Sensor &sensor);

private:
  Eigen::MatrixXd I_;

};

#endif /* KALMAN_FILTER_H_ */
