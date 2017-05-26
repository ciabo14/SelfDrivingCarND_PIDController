#include "PID.h"
#include <limits>
#include <algorithm>

// MIN_STEPS is used by the 

using namespace std;

PID::PID(double Kp, double Ki, double Kd){
	pid_coeff[0] = Kp;
	pid_coeff[1] = Ki; 
	pid_coeff[2] = Kd;
	
	p_error = i_error = d_error = 0.0;
}

PID::~PID() {}

void PID::Init(int min_steps_in, double dpp, double dpi, double dpd) {
	optimized = false;
	tol = 0.0001;
	dp[0] = dpp;
	dp[1] = dpi;
	dp[2] = dpd;
	
	min_steps = min_steps_in;
	best_opt_error = 100;
	twiddle_phase = 0;
	current_opt_par = 0;
	ResetTwiddleParams();

}

void PID::ResetTwiddleParams() {
	total_error = 0;
	p_error = i_error = d_error = 0.0;
	number_opt_steps = 0;
}

void PID::UpdateError(double cte) {

	p_error = cte;
	// we assume each timestemp is the unit. The d_error is then the difference between the last two errors
	d_error = cte - d_error;
	i_error += cte;
	if(number_opt_steps >= min_steps) total_error += pow(cte,2);
	number_opt_steps += 1;

}

double PID::TotalError() {
	return total_error/ (number_opt_steps- min_steps);
}

double PID::ComputeVal() {
	double steer_val = - pid_coeff[0] * p_error - pid_coeff[1] * i_error - pid_coeff[2] * d_error;
	return steer_val;
}

void PID::Twiddle() {

	if (!optimized) {

		double current_err = TotalError();
		switch (twiddle_phase) {
			// Case 0: used for the first iteration only
			case 0:
				best_opt_error = current_err;
				pid_coeff[current_opt_par] += dp[current_opt_par];
				twiddle_phase = 1;//(twiddle_phase + 1) % 3;
				break;
			/* 
			 * Case 1: check the error with the current adjusted parameter. If the current one is better,
			 * reduce the correction; otherwise, check in the other direction
			 */
			case 1:
				if (current_err < best_opt_error) {

					best_opt_error = current_err;
					ComputeNextStepParam(true, 1.1);
										
				}
				else { // try in the other direction
					if (2 * dp[current_opt_par] <= pid_coeff[current_opt_par]) {
						pid_coeff[current_opt_par] -= (2 * dp[current_opt_par]);
						twiddle_phase = 2; // (twiddle_phase + 1) % 3;
					}
					else {
						//Restore Original value for pid_coeff[current_opt_par]
						pid_coeff[current_opt_par] -= dp[current_opt_par];

						twiddle_phase = 1;
						ComputeNextStepParam(true, 0.9);
					}
				}
				break;
			/*
			 * Case 2: check the error with the current adjusted parameter (other direction than initally);
			 * Adjust then the correction parameter.
			 */
			case 2:
				if (current_err < best_opt_error) {
					best_opt_error = current_err;
					ComputeNextStepParam(true, 1.1);
				}
				else {
					pid_coeff[current_opt_par] += dp[current_opt_par];
					ComputeNextStepParam(true, 0.9);
				}
				twiddle_phase = 1;
				break;
		}
		double max_dp_coeff = *std::max_element(dp, dp + (sizeof(dp)/ sizeof(double)));
		if (max_dp_coeff < tol)
			optimized = true;
	}
}


void PID::ComputeNextStepParam(bool keepParam, double updateMulVal) {
	
	dp[current_opt_par] *= updateMulVal;

	//Switch to the next parameter and update its PID coefficient
	if (dp[(current_opt_par + 1) % 3] > tol)
		current_opt_par = (current_opt_par + 1) % 3;
	else
		if (dp[(current_opt_par + 2) % 3] > tol)
			current_opt_par = (current_opt_par + 2) % 3;

	pid_coeff[current_opt_par] += dp[current_opt_par];
}