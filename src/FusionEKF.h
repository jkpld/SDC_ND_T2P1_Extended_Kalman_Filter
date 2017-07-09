#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "Sensors.h"

class FusionEKF {
public:
  // Constructor.
  FusionEKF();

  // Destructor.
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /* State transition matrix
   * @param dt Time difference since last update
  */
  inline Eigen::MatrixXd F(double dt);

  /* Processes covariance matrix
   * @param dt Time difference since last update
  */
  inline Eigen::MatrixXd Q(double dt);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Radar radar_;
  Lidar lidar_;
  // Eigen::MatrixXd R_laser_;
  // Eigen::MatrixXd R_radar_;
  // Eigen::MatrixXd H_laser_;
  // Eigen::MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */
