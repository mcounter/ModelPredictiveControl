#include "MPC.h"
#include <time.h>
#include <iostream>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

//Global functions

// Evaluate a polynomial.
double polyeval(const Eigen::VectorXd& coeffs, const double& x)
{
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++)
	{
		result += coeffs[i] * pow(x, i);
	}

	return result;
}

// Differentiation of polynom
Eigen::VectorXd polydif(const Eigen::VectorXd& coeffs)
{
	int sz = coeffs.size();
	Eigen::VectorXd dcoeffs((sz <= 1) ? 1 : sz - 1);

	if (sz <= 1)
	{
		dcoeffs[0] = 0.0;
	}
	else
	{
		for (int i = 1; i < sz; ++i)
		{
			dcoeffs[i - 1] = coeffs[i] * double(i);
		}
	}

	return dcoeffs;
}

// Variable vector definition
// NOTE: There are no separate variables for CTE and EPsi
// It can be calculated with help of X, Y, Psi and trajectory polinomial
const int x_start = 0;
const int y_start = x_start + prediction_size;
const int psi_start = y_start + prediction_size;
const int v_start = psi_start + prediction_size;
const int delta_start = v_start + prediction_size;
const int a_start = delta_start + prediction_size;
const int var_size = a_start + prediction_size;
const int constr_size = var_size;

class FG_eval
{
public:
	// Fitted 3-rd order polynomial coefficients
	Eigen::VectorXd coeffs;
	// Derivative of the polynomial
	Eigen::VectorXd dcoeffs;

	// Time length of single prediction step
	double dt;

	// Vehicle length
	double lf;

	// Target speed
	double target_speed;

	// Maxiaml steering angle
	double delta_mult;

	// Maximal acceleration
	double a_mult;

	// Connection lag
	int connection_lag;

	// Steering response lag
	int steering_lag;

	// Throttle response lag
	int acceleration_lag;

	// Class constructor. Initialize internal variables
	FG_eval(const Eigen::VectorXd& coeffs, const Eigen::VectorXd& dcoeffs,
		const double& dt, const double& lf, const double& target_speed, const double& delta_mult, const double& a_mult,
		const int& connection_lag, const int& steering_lag, const int& acceleration_lag)
	{
		this->coeffs = coeffs;
		this->dcoeffs = dcoeffs;
		this->dt = dt;
		this->lf = lf;
		this->target_speed = target_speed;
		this->delta_mult = delta_mult;
		this->a_mult = a_mult;
		this->connection_lag = connection_lag;
		this->steering_lag = steering_lag;
		this->acceleration_lag = acceleration_lag;
	}

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	void operator()(ADvector& fg, const ADvector& vars)
	{
		// Weight for each cost condition

		double weight_cte = 1.0;
		double weight_v = 0.01;
		double weight_ddelta = 3000.0;
		double weight_da = 0.0;

		double weight_epsi = 0.0;
		double weight_delta = 0.0;
		double weight_a = 0.0;

		// Cost conditions
		fg[0] = 0;

		for (int t = 1; t < prediction_size; ++t)
		{
			AD<double> x1 = vars[x_start + t];
			AD<double> y1 = vars[y_start + t];
			AD<double> psi1 = vars[psi_start + t];
			AD<double> v1 = vars[v_start + t];
			AD<double> delta1 = vars[delta_start + t];
			AD<double> a1 = vars[a_start + t];
			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0 = vars[a_start + t - 1];
			AD<double> f1 = coeffs[0] + coeffs[1] * x1 + coeffs[2] * CppAD::pow(x1, 2) + coeffs[3] * CppAD::pow(x1, 3);
			AD<double> df1 = dcoeffs[0] + dcoeffs[1] * x1 + dcoeffs[2] * CppAD::pow(x1, 2);
			AD<double> cte1 = CppAD::cos(CppAD::atan(df1)) *  (f1 - y1);
			AD<double> epsi1 = CppAD::sin(CppAD::atan(CppAD::tan(psi1)) - CppAD::atan(df1));

			fg[0] += weight_cte * CppAD::pow(cte1, 2) * CppAD::fabs(v1) * dt;
			fg[0] += weight_v * CppAD::pow(vars[v_start + t] - target_speed, 2);
			fg[0] += weight_ddelta * CppAD::pow(delta1 - delta0, 2);
			fg[0] += weight_da * CppAD::pow(a1 - a0, 2) * (1 + CppAD::sign(a1 - a0));
			
			fg[0] += weight_epsi * CppAD::pow(epsi1, 2);
			fg[0] += weight_delta * CppAD::pow(delta1, 2);
			fg[0] += weight_a * CppAD::pow(a1, 2);
		}

		// Vechicle model based constraints
		// Kinematic model is used with connection lag, steering and throttle response lag.
		// NOTE: There are no separate constraints for CTE and EPsi
		// It can be calculated with help of X, Y, Psi and trajectory polinomial

		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + delta_start] = vars[delta_start];
		fg[1 + a_start] = vars[a_start];

		int connection_lag_cnt = connection_lag;

		for (int t = 1; t < prediction_size; ++t)
		{
			AD<double> x0 = vars[x_start + t - 1];
			AD<double> y0 = vars[y_start + t - 1];
			AD<double> psi0 = vars[psi_start + t - 1];
			AD<double> v0 = vars[v_start + t - 1];
			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0 = vars[a_start + t - 1];

			AD<double> x1 = vars[x_start + t];
			AD<double> y1 = vars[y_start + t];
			AD<double> psi1 = vars[psi_start + t];
			AD<double> v1 = vars[v_start + t];
			AD<double> delta1 = vars[delta_start + t];
			AD<double> a1 = vars[a_start + t];

			AD<double> delta_active = (t < steering_lag) ? vars[delta_start] : vars[delta_start + t - steering_lag];
			AD<double> a_active = (t < acceleration_lag) ? vars[a_start] : vars[a_start + t - acceleration_lag];

			fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_start + t] = psi1 - (psi0 - v0 * CppAD::tan(delta_mult * delta_active) * dt / lf);
			fg[1 + v_start + t] = v1 - (v0 + a_mult * a_active * dt);

			if (connection_lag_cnt <= 0)
			{
				if ((t + steering_lag) < prediction_size)
				{
					fg[1 + delta_start + t] = 0;
				}
				else
				{
					fg[1 + delta_start + t] = delta1 - delta0;
				}

				if ((t + acceleration_lag) < prediction_size)
				{
					fg[1 + a_start + t] = 0;
				}
				else
				{
					fg[1 + a_start + t] = a1 - a0;
				}

				connection_lag_cnt = connection_lag;
			}
			else
			{
				fg[1 + delta_start + t] = delta1 - delta0;
				fg[1 + a_start + t] = a1 - a0;
			}

			--connection_lag_cnt;
		}
	}
};

// Initialize MPC class variables
MPC::MPC()
{
	lastControl = Eigen::VectorXd(2);
	lastControl << 0.0, 0.0; // steering angle, throttle

	last_timestamp = 0.0;
	connection_history.clear();
	connection_history.push_back(lag_connection_avg_ms / 1000.0);
	connection_lag_sum += connection_history[0];
	connection_lag_calc = connection_lag_sum;
}

MPC::~MPC() {}

void MPC::Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs,
	Eigen::VectorXd& control, Eigen::VectorXd& target_path, Eigen::VectorXd& vehicle_path)
{
	typedef CPPAD_TESTVECTOR(double) Dvector;

	// Calculate avegage connection lag based on difference between adjacent requests
	double timestamp = (double)clock() / CLOCKS_PER_SEC;
	if (last_timestamp <= 0.0)
	{
		last_timestamp = timestamp;
	}
	else
	{
		double dt = timestamp - last_timestamp;
		last_timestamp = timestamp;

		if (dt > 0)
		{
			connection_lag_sum += dt;
			connection_history.push_back(dt);

			if (connection_history.size() > lag_connection_history_size)
			{
				connection_lag_sum -= connection_history[0];
				connection_history.erase(connection_history.begin());
			}

			connection_lag_calc = connection_lag_sum / double(connection_history.size());
		}
	}

	// Convert lags to discrete prediction steps
	int connection_lag = max(1, int(round(connection_lag_calc / (prediction_step_ms / 1000.0))));
	int steering_lag = int(round(lag_steering_ms / prediction_step_ms));
	int acceleration_lag = int(round(lag_throttle_ms / prediction_step_ms));

	// Calculate derivative of the polynomial
	Eigen::VectorXd dcoeffs = polydif(coeffs);

	Eigen::VectorXd initials(6);
	initials <<
		state[0],
		state[1],
		state[2],
		state[3] * 1609.34 / 3600.00, // Speed must be converted from Miles/s to m/s
		state[4],
		state[5];


	// Initialize variables and constraints
	Dvector vars(var_size);

	for (int i = 0; i < prediction_size; i++)
	{
		vars[x_start + i] = initials[0];
		vars[y_start + i] = initials[1];
		vars[psi_start + i] = initials[2];
		vars[v_start + i] = initials[3];
		vars[delta_start + i] = initials[4];
		vars[a_start + i] = initials[5];
	}

	Dvector vars_lowerbound(var_size);
	Dvector vars_upperbound(var_size);
	for (int i = x_start; i < x_start + prediction_size; i++)
	{
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}

	for (int i = y_start; i < y_start + prediction_size; i++)
	{
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}

	for (int i = psi_start; i < psi_start + prediction_size; i++)
	{
		vars_lowerbound[i] = -M_PI;
		vars_upperbound[i] = M_PI;
	}

	for (int i = v_start; i < v_start + prediction_size; i++)
	{
		vars_lowerbound[i] = 0;
		vars_upperbound[i] = 1.0e19;
	}

	for (int i = delta_start; i < delta_start + prediction_size; i++)
	{
		vars_lowerbound[i] = -1.0;
		vars_upperbound[i] = 1.0;
	}

	for (int i = a_start; i < a_start + prediction_size; i++)
	{
		vars_lowerbound[i] = -1.0;
		vars_upperbound[i] = 1.0;
	}

	vars_lowerbound[x_start] = initials[0];
	vars_lowerbound[y_start] = initials[1];
	vars_lowerbound[psi_start] = initials[2];
	vars_lowerbound[v_start] = initials[3];
	vars_lowerbound[delta_start] = initials[4];
	vars_lowerbound[a_start] = initials[5];

	vars_upperbound[x_start] = initials[0];
	vars_upperbound[y_start] = initials[1];
	vars_upperbound[psi_start] = initials[2];
	vars_upperbound[v_start] = initials[3];
	vars_upperbound[delta_start] = initials[4];
	vars_upperbound[a_start] = initials[5];

	Dvector constraints_lowerbound(constr_size);
	Dvector constraints_upperbound(constr_size);
	for (int i = 0; i < constr_size; i++)
	{
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}

	constraints_lowerbound[x_start] = initials[0];
	constraints_lowerbound[y_start] = initials[1];
	constraints_lowerbound[psi_start] = initials[2];
	constraints_lowerbound[v_start] = initials[3];
	constraints_lowerbound[delta_start] = initials[4];
	constraints_lowerbound[a_start] = initials[5];

	constraints_upperbound[x_start] = initials[0];
	constraints_upperbound[y_start] = initials[1];
	constraints_upperbound[psi_start] = initials[2];
	constraints_upperbound[v_start] = initials[3];
	constraints_upperbound[delta_start] = initials[4];
	constraints_upperbound[a_start] = initials[5];

	// Object that computes objective and constraints
	FG_eval fg_eval(
		coeffs, dcoeffs,
		prediction_step_ms / 1000.0, vehicle_lf, target_speed * 1609.34 / 3600.00, steering_angle_mult, acceleration_mult,
		connection_lag, steering_lag, acceleration_lag);

	//
	// NOTE: You don't have to worry about these options
	//
	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";

	// Place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
		constraints_upperbound, fg_eval, solution);

	// Check some of the solution values
	bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Cost
	auto cost = solution.obj_value;

	if (ok)
	{
		std::cout << "Cost " << cost << std::endl;
	}
	else
	{
		std::cout << "Cannot predict values"<< std::endl;
	}

	// Next control values, taking in account connection lag
	control = Eigen::VectorXd(2);
	control <<
		solution.x[delta_start + connection_lag + 1],
		solution.x[a_start + connection_lag + 1];
	
	lastControl = Eigen::VectorXd(control);

	// Collect values for visualization
	int vis_vector_size = 2 * (prediction_size - connection_lag - 1);
	target_path = Eigen::VectorXd(vis_vector_size);
	vehicle_path = Eigen::VectorXd(vis_vector_size);

	for (int i = connection_lag + 1; i < prediction_size; ++i)
	{
		int pos = (i - connection_lag - 1) * 2;

		target_path[pos] = solution.x[x_start + i];
		target_path[pos + 1] = polyeval(coeffs, solution.x[x_start + i]);

		vehicle_path[pos] = solution.x[x_start + i];
		vehicle_path[pos + 1] = solution.x[y_start + i];
	}
}
