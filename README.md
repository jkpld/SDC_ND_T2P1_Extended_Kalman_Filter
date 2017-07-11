# Extended Kalman Filter Project

Self-Driving Car Engineer Nanodegree Program

**The goal of this project is to utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.**

## Code structure
The project code is broken up into several different classes:
* An abstract `Sensor` class.
* A `FusionEKF` class.
* A `KalmanFilter` class.

### `Sensor` class
The purpose of the sensor class is to hold data and methods that are unique to the sensor. The data the class holds are
* The measurement noise covariance matrix, `R`.
* The type of sensor: `linear` or `extended`.

The abstract methods include
* `meas_state_difference(measure, state)`: compute the difference between the measured state and the predicted measurement based on the state. _This function handles angle wrapping for the Radar sensor._
* `state_to_measure(state)`: Convert from state space to measurement space.
* `measure_to_state(meas)`: Convert from measurement space to state space.
* `Jacobian(state)`: Compute the Jacobian of the coordinate conversion from state to measurement space. _This is a private method._

Additionally, the `Sensor` class has a method `H(state)` that returns the H matrix to be used for computing the covariance matrix.

#### Concrete `Sensor`s
The abstract `Sensor` class is used to define two different concrete classes, `Radar` and `Lidar`. These classes implement the virtual functions listed above.

### `FusionEKF` class
This class handles taking in new measurements and initializing or updating the Kalman filter. The methods include
* `ProcessMeasurement(MeasurementPackage)`: primary method that initializes the Kalman filter using the first measurement, and then updates it using subsequent measurements.
* `F(dt)`: This method takes in the time since the last measurement and returns the state transition matrix `F`.
* `Q(dt)`: This method takes in the time since the last measurement and returns the process covariance matrix `Q`.

This class also contains instances of the `KalmanFilter`, `Radar`, and `Lidar` classes.

### `KalmanFilter` class
This class contains three methods:
* `Init(x,P,F,Q)`: Initialize the filter using a known state `x`, covariance `P`, state transition matrix `F`, and process covariance noise `Q`.
* `Predict()`: Propagate the state forward in time.
* `Update(meas, Sensor)`: Take in a new measurement and the `Sensor` that produced the measurement and update the filter's state and covariance.

## Notes
I modified the default CMakeLists.txt to include the Sensors.h and Sensors.cpp files.
