#include "Sensors.h"
#include "Eigen/Dense"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*******************************************************************************
/*   Radar class definitions
/******************************************************************************/
Radar::Radar() {
  sensor_type = Sensor::extended;

  R = MatrixXd::Zero(3,3);
  R.diagonal() << 0.09, 0.0009, 0.09;
}

Radar::~Radar() {}

VectorXd Radar::state_to_measure(const VectorXd &state) {
  double rx = state(0);
  double ry = state(1);
  double vx = state(2);
  double vy = state(3);

  double r = sqrt(rx*rx + ry*ry);
  double phi = atan2(ry,rx);
  double rdot = (rx*vx + ry*vy)/r;

  VectorXd meas = VectorXd(3);
  meas << r, phi, rdot;
  return meas;
}

VectorXd Radar::measure_to_state(const VectorXd &meas) {
  double r = meas(0);
  double phi = meas(1);

  VectorXd state = VectorXd::Zero(4);
  state(0) = r*cos(phi);
  state(1) = r*sin(phi);
  return state;
}

MatrixXd Radar::Jacobian(const VectorXd &state) {
  Hj_ = MatrixXd(3,4);
  //recover state parameters
	double rx = state(0);
	double ry = state(1);
	double vx = state(2);
	double vy = state(3);

  //pre-compute a set of terms to avoid repeated calculation
	double r2 = rx*rx + ry*ry;
	double r = sqrt(r2);
	double r3 = r2*r;

  //check division by zero
	if(fabs(r2) < 0.0001){
		std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
		return Hj_;
	}

  double cross = vx*ry - vy*rx;

	//compute the Jacobian matrix
	Hj_ << (rx/r), (ry/r), 0, 0,
		  -(ry/r2), (rx/r2), 0, 0,
		  ry*cross/r3, -rx*cross/r3, rx/r, ry/r;

  return Hj_;
}

/*******************************************************************************
/*   Lidar class definitions
/******************************************************************************/
Lidar::Lidar()
{
  sensor_type = Sensor::linear;

  R = MatrixXd::Zero(2,2);
  R.diagonal() << 0.0225, 0.0225;

  H_ = MatrixXd::Zero(2, 4);
  H_(0,0) = 1;
  H_(1,1) = 1;

  Hj_ = H_;
}
Lidar::~Lidar() {}

VectorXd Lidar::measure_to_state(const VectorXd &meas) {
  return H_.transpose() * meas;
}

VectorXd Lidar::state_to_measure(const VectorXd &state) {
  return H_*state;
}
MatrixXd Lidar::Jacobian(const VectorXd &state) {
  return Hj_;
}
