#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double vehicle_lf = 2.67;

// Target vehicle speed
const double target_speed = 100.0;

// Initial connection lag in ms (later it will be recalculated)
const double lag_connection_avg_ms = 100;

// Number of samples to calculate average connection lag
const int lag_connection_history_size = 20;

// Steering response lag in ms
const double lag_steering_ms = 300.0;

// Throttle response lag in ms
const double lag_throttle_ms = 300.0;

// Time length of single prediction step
const double prediction_step_ms = 100.0;

// Number of prediction steps calculated at once
const int prediction_size = 20;

// Steering angle for maximal steering control value
const double steering_angle_mult = 25.0 / 180.0 * M_PI;

// Acceleration approximation for maximum throttle value
const double acceleration_mult = 0.4;

class MPC
{
private:
	Eigen::VectorXd lastControl;
	double last_timestamp;
	double connection_lag_sum;
	double connection_lag_calc;
	vector<double> connection_history;

public:
	MPC();

	virtual ~MPC();

	// Solve the model given an initial state and polynomial coefficients
	// state - current state vector in vehicle coordinates { x, y, psi, speed, effective steering angle, effective throttle value }
	// coeffs - coefficients of path polynom (spline) in vehicle coordinates
	// control - returns control vector for external actuators scaled to simulator range { steering angle [-1, 1], throttle [-1, 1] }
	// target_path - target path points in format x1,y1,x2,y2,...
	// vehicle_path - predicted vehicle path points in format x1,y1,x2,y2,...
	void Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs,
		Eigen::VectorXd& control, Eigen::VectorXd& target_path, Eigen::VectorXd& vehicle_path);
};

#endif /* MPC_H */
