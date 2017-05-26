#ifndef PID_H
#define PID_H

class PID {
public:
	/*
	* Errors
	*/
	double p_error;
	double i_error;
	double d_error;
	double total_error;
	double best_opt_error;

	/*
	* Coefficients
	*/ 
	double pid_coeff[3] = {};
	//double Kp;
	//double Ki;
	//double Kd;
	
	/*
	* Twiddle parameters
	*/
	double dp[3] = {};
	//double dp;
	//double di;
	//double dd;
	// used to define which is the current parameter to be optimized:
	// 0 --> means p coeff; 1 --> means i coeff; 2 --> means d coeff
	int current_opt_par;
	int twiddle_phase;
	int number_opt_steps;
	double tol;
	
	//min_steps is used to avoid the usage of the first N values for the optimization
	int min_steps;
	
	/*
	* Optimization parameters
	*/
	bool optimized;
	
	/*
	* Constructor
	*/
	PID(double Kp, double Ki, double Kd);
	
	/*
	* Destructor.
	*/
	virtual ~PID();
	
	/*
	* Initialize PID.
	*/
	void Init(int min_steps_in, double dpp = .0, double dpi = .0, double dpd = .0);
	
	/*
	 * Used to reset the parameters required by the twiddle procedure
	 */
	void ResetTwiddleParams();
	
	/*
	* Update the PID error variables given cross track error.
	*/
	void UpdateError(double cte);
	
	/*
	* Calculate the total PID error.
	*/
	double TotalError();
	
	/*
	* Exectute one step of the Twiddle procedure and check if the optimization requirements are satisfied
	* If yes, set the internal state "optimized" to true.
	*/
	void Twiddle();
	
	/*
	* Exectute the PID getting back the computed value
	*/
	double ComputeVal();
	
	private:
	/*
	* Compute the PID controller for the next Twiddle step, and adjust the correction coefficients dp.
	*/
	void ComputeNextStepParam(bool keepParam, double updateMulVal);
};

#endif /* PID_H */
