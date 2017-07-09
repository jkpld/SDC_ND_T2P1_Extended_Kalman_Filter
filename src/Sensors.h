#ifndef Sensor_H_
#define Sensor_H_

#include "Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;

/********************************/
/*   Abstact Sensor class       */
class Sensor {
public:

  enum SensorType{
    linear,
    extended
  } sensor_type;

  // measurment noise covariance
  MatrixXd R;

  /* internal storage for projection matrix from internal state to measured
  state */
  MatrixXd H_;
  /* internal storage for jacobian of projection*/
  MatrixXd Hj_;

  // Constructor
  Sensor() {};

  // Destructor
  virtual ~Sensor() {};

  /* Methods to be implimented by subclasses:
    Convert state to sensor
    Compute Jacobian -- only required if sensor is extended
  */
  virtual VectorXd state_to_measure(const VectorXd &state) = 0;
  virtual VectorXd measure_to_state(const VectorXd &meas) = 0;
  virtual MatrixXd Jacobian(const VectorXd &state) = 0;

  // Method to return H matrix for covariance calculations
  MatrixXd H(const VectorXd &state) {
    if (sensor_type == Sensor::extended) {
      Hj_ = Jacobian(state);
    }
    return Hj_;
  };

};

/********************/
/*   Radar Sensor   */
class Radar : public Sensor
{
public:
  // Constructor
  Radar();
  // Destructor
  ~Radar();

  VectorXd state_to_measure(const VectorXd &state);
  VectorXd measure_to_state(const VectorXd &meas);
  MatrixXd Jacobian(const VectorXd &state);

};

/********************/
/*   Lidar Sensor   */
class Lidar : public Sensor
{
public:
  // Constructor
  Lidar();
  // Destructor
  ~Lidar();

  // These are not used
  VectorXd state_to_measure(const VectorXd &state);
  VectorXd measure_to_state(const VectorXd &meas);
  MatrixXd Jacobian(const VectorXd &state);

};


#endif /* Sensor_H_ */
