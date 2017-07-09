#ifndef Sensor_H_
#define Sensor_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Sensor {
public:

  enum SensorType{
    linear,
    extended
  } sensor_type;

  MatrixXd R;

  // Constructor
  Sensor();

  // Destructor
  virtual ~Sensor();

  /* Methods to be implimented by subclasses:
    Convert state to sensor
    Compute Jacobian -- only required if sensor is extended
  */
  virtual VectorXd state_projection(const VectorXd &state) = 0;
  virtual MatrixXd Jacobian(const VectorXd &state) = 0;

  // Method to return the predicted measurments from the state
  VectorXd z_predicted(const VectorXd &state);

  // Method to return H matrix for covariance calculations
  MatrixXd H(const VectorXd &state);

private:
  /* internal storage for projection matrix from internal state to measured
  state */
  MatrixXd H_;
  /* internal storage for jacobian of projection*/
  MatrixXd Hj_;

  /* internal storage for current state. Here we use two states so we don't have
   to keep track of the order z_predicted() and H() are called. */
};

class Radar : public Sensor
{
public:
  // Constructor
  Radar();
  // Destructor
  ~Radar();

  VectorXd state_projection(const VectorXd &state);
  MatrixXd Jacobian(const VectorXd &state);

};

class Lidar : public Sensor
{
public:
  // Constructor
  Lidar();
  // Destructor
  ~Lidar();

  // These are not used
  VectorXd state_projection(const VectorXd &state);
  MatrixXd Jacobian(const VectorXd &state);

};


#endif /* Sensor_H_ */
