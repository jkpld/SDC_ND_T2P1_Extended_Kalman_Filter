#include "Sensors.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/********************************/
/*   Sensor class definitions   */
Sensor::Sensor() {}
Sensor::~Sensor() {}

VectorXd Sensor::z_predicted(const VectorXd &state) {
  if (sensor_type == Sensor::linear) {
    z_pred_ = H_*state;
  } else {
    z_pred_ = state_projection(state);
  }
  return z_pred_
}

MatrixXd Sensor::H(const VectorXd &state) {
  if (sensor_type == Sensor::extended) {
    Hj_ = Jacobian(state);
  }
  return Hj_
}

/********************************/
/*   Radar class definitions    */
Radar::Radar() {
  sensory_type = Sensor::extended;

  R = MatrixXd(3, 3);
  R << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;
}
Radar::~Radar() {}

VectorXd Radar::state_projection(const VectorXd &state) {
  return state;
}
MatrixXd Radar::Jacobian(const VectorXd &state) {
  Hj_ = MatrixXd::Zeros(3,4);
  //recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

  //check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj_;
	}

  float cross = vx*py - vy*px;

	//compute the Jacobian matrix
	Hj_ << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*cross/c3, -px*cross/c3, px/c2, py/c2;

  return Hj_;
}

/********************************/
/*   Lidar class definitions    */
Lidar::Lidar()
{
  sensory_type = Sensor::linear;

  R = MatrixXd(2, 2);
  H_ = MatrixXd(2, 4);

  R << 0.0225, 0,
      0, 0.0225;

  H_ << 1, 0, 0, 0,
		    0, 1, 0, 0;
  Hj_ = H_;
}
Lidar::~Lidar() {}

// These are not used
VectorXd Lidar::state_projection(const VectorXd &state) {
  return H_*state;
}
MatrixXd Lidar::Jacobian(const VectorXd &state) {
  return Hj_;
}
