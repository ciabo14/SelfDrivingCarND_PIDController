#define _USE_MATH_DEFINES
#include "uWS/uWS.h"
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <iostream>
#include <fstream>
/* 
 * 0 --> Optimization phase
 * 1 --> Running Phase 
*/
#define PHASE 0

/* 
 * 0 --> traj PID Only
 * 1 --> traj & speed PIDs
*/
#define PID_VERSION 1

/*
 * Define the min number of steps to wait before to start the optimization phase and the max number
 * of steps per optimization step
 */
#define MIN_STEPS 200
#define MAX_OPT_STEPS 2200

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::stringstream hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return std::stringstream();
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    std::stringstream tmp = std::stringstream();
    tmp.str(s.substr(b1, b2 - b1 + 1));
    return tmp;
  }
  return std::stringstream();
}
void testErrorupdate() {

	PID p(1, 1, 1);
	p.Init((int)MIN_STEPS);
	double error = 1.0;
	p.UpdateError(error);
	assert(1.0 == p.p_error);
	assert(1.0 == p.d_error);
	assert(1.0 == p.i_error);

	p.Init((int)MIN_STEPS);

	for(int i = 0; i< 100; i++)
		p.UpdateError(error);
	
	assert(1.0 == p.p_error);
	assert(0.0 == p.d_error);
	assert(100.0 == p.i_error);

	p.Init((int)MIN_STEPS);

	for (int i = 0; i< 101; i++)
		p.UpdateError((double)i);

	assert(100.0 == p.p_error);
	assert(50.0 == p.d_error);
	assert(5050.0 == p.i_error);

	p.Init((int)MIN_STEPS);;

	p.UpdateError(-1.0);

	assert(-1.0 == p.p_error);
	assert(-1.0 == p.d_error);
	assert(-1.0 == p.i_error);

	p.Init((int)MIN_STEPS);
	for (int i = -50; i< 51; i++)
		p.UpdateError((double)i);

	assert(50.0 == p.p_error);
	assert(0.0 == p.d_error);
	assert(0.0 == p.i_error);

}

void testTwiddle() {

	PID p(1, 1, 1);
	p.Init((int)MIN_STEPS);

	for (int i = 0; i<MIN_STEPS; i++)
		p.UpdateError(0.0);

	p.UpdateError(1000.0);
	p.Twiddle();

	assert(p.twiddle_phase == 1);
	assert(p.current_opt_par == 0);
	
	p.ResetTwiddleParams();
	for (int i = 0; i<MIN_STEPS; i++)
		p.UpdateError(0.0);
	p.UpdateError(999.0);
	p.Twiddle();

	assert(p.twiddle_phase == 1);
	assert(p.current_opt_par == 1);

	p.ResetTwiddleParams();
	for (int i = 0; i<MIN_STEPS; i++)
		p.UpdateError(0.0);
	p.UpdateError(1000.0);
	p.Twiddle();

	assert(p.twiddle_phase == 2);
	assert(p.current_opt_par == 1);

	p.ResetTwiddleParams();
	for (int i = 0; i<MIN_STEPS; i++)
		p.UpdateError(0.0);
	p.UpdateError(900.0);
	p.Twiddle();

	assert(p.twiddle_phase == 1);
	assert(p.current_opt_par == 2);

	p.ResetTwiddleParams();
	for (int i = 0; i<MIN_STEPS; i++)
		p.UpdateError(0.0);
	p.UpdateError(1000.0);
	p.Twiddle();

	assert(p.twiddle_phase == 2);
	assert(p.current_opt_par == 2);

	p.ResetTwiddleParams();
	for (int i = 0; i<MIN_STEPS; i++)
		p.UpdateError(0.0);
	p.UpdateError(1100.0);
	p.Twiddle();

	assert(p.twiddle_phase == 1);
	assert(p.current_opt_par == 0);

	p.ResetTwiddleParams();
	for (int i = 0; i<MIN_STEPS; i++)
		p.UpdateError(0.0);
	p.UpdateError(2000.0);
	p.Twiddle();

	assert(p.twiddle_phase == 2);
	assert(p.current_opt_par == 0);

}

void WriteIntoFileResults(PID pid_pos, int currentOpt) {

	ofstream myfile;

	ostringstream fileName;
	fileName << "E:\\Self Driving Car Nanodegree\\Term 2\\CarND-PID-Control-Project-master\\PID_params_" << currentOpt << ".txt";

	myfile.open(fileName.str());

	myfile << "Current PID Parameters:\n";
	myfile << "p: " << pid_pos.pid_coeff[0] << " - i: " << pid_pos.pid_coeff[1] << " - d: " << pid_pos.pid_coeff[2] << "\n";
	myfile << "Best error: " << pid_pos.best_opt_error;

	myfile.close();

}
/*

inline void OptimizationAutomation(PID p, int &currentOpt) {

	if (p.optimized) {
		int stop = 1;
		
		/*
		WriteIntoFileResults(pid_pos, currentOpt);
		if (currentOpt >= 5)
		int stop = 1;
		switch (currentOpt) {
		case 0:
		pid_pos.current_opt_par = 1;
		pid_pos.ResetTwiddleParams();
		pid_pos.best_opt_error = 100;
		pid_pos.optimized = false;
		pid_pos.dp[0] = pid_pos.dp[2] = 0;
		pid_pos.dp[1] = 0.001;
		pid_pos.tol = 0.0001;

		break;

		case 1:
		pid_pos.current_opt_par = 2;
		pid_pos.ResetTwiddleParams();
		pid_pos.best_opt_error = 100;
		pid_pos.optimized = false;

		pid_pos.dp[0] = pid_pos.dp[1] = 0;
		pid_pos.dp[2] = 0.1;
		pid_pos.tol = 0.01;

		break;

		case 2:
		pid_pos.current_opt_par = 2;
		pid_pos.ResetTwiddleParams();
		pid_pos.best_opt_error = 100;
		pid_pos.optimized = false;
		pid_pos.current_opt_par = 100;

		pid_pos.pid_coeff[1] = pid_pos.pid_coeff[2] = 0.0;
		pid_pos.dp[0] = pid_pos.dp[1] = 0;
		pid_pos.dp[2] = 0.1;
		pid_pos.tol = 0.01;

		break;
		case 3:
		pid_pos.current_opt_par = 1;
		pid_pos.ResetTwiddleParams();
		pid_pos.best_opt_error = 100;
		pid_pos.optimized = false;
		pid_pos.pid_coeff[1] = pid_pos.pid_coeff[2] = 0.0;
		pid_pos.dp[0] = pid_pos.dp[2] = 0;
		pid_pos.dp[1] = 0.001;
		pid_pos.tol = 0.0001;

		break;
		case 4:
		pid_pos.current_opt_par = 0;
		pid_pos.ResetTwiddleParams();
		pid_pos.best_opt_error = 100;
		pid_pos.optimized = false;
		pid_pos.pid_coeff[0] = pid_pos.pid_coeff[1] = pid_pos.pid_coeff[2] = 0.0;
		pid_pos.dp[0] = pid_pos.dp[2] = 0.1;
		pid_pos.dp[1] = 0.001;
		pid_pos.tol = 0.0001;

		break;
		}
		currentOpt += 1;
		
}
*/
int main()
{
	
	uWS::Hub h;
	PID pid_pos(0.514387, 0.000472982,0.417457);
	PID pid_speed(.3, .0, .002);
	if(PHASE == 0){
		pid_pos.Init(MIN_STEPS, .1, .001, .1);
		pid_speed.Init(MIN_STEPS);
	}
	
	double currentCTE;
	currentCTE = 0.0 ;
	double oldCTE=0.0;
	int currentOpt = 1000;
	
	//testErrorupdate();
	//testTwiddle();
	h.onMessage([&pid_pos, &pid_speed, &currentOpt](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
		auto s = hasData(std::string(data));
		if (s.str() != "") {
		auto j = json::parse(s);
		std::string event = j[0].get<std::string>();
		if (event == "telemetry") {
			// j[1] is the data JSON object
			double cte = std::stod(j[1]["cte"].get<std::string>());
			double speed = std::stod(j[1]["speed"].get<std::string>());
			double angle = std::stod(j[1]["steering_angle"].get<std::string>());

			// compute the steering val to be appliyed and limit in in the range 0-1
			pid_pos.UpdateError(cte);
			double steer_value = pid_pos.ComputeVal();
			steer_value = steer_value < -1 ? -1 : (steer_value > 1 ? 1 : steer_value);
		  

			double target_speed = 5. * (1. - fabs(steer_value)) + 15.;
			pid_speed.UpdateError(speed - target_speed);

			double throttle = pid_speed.ComputeVal();
			throttle = throttle < -.5 ? -.5 : (throttle > .5 ? .5 : throttle);

			if (PHASE == 0) {

				bool stopOptimization = false;
			  
				/*
				* Optimization used during the Twiddle phase. Stop the execution of the current twiddle trial
				* id the current error already overcamed the best error found so far by the PID. This 
				* is not always true and usable for PID controller, but works good in this case
				*/
				if (pid_pos.TotalError() > pid_pos.best_opt_error && pid_pos.number_opt_steps >= MIN_STEPS * 2) {
					cout << "Interrupt optimization because error overcome the best" << endl;
					stopOptimization = true;
				}

				/*
				* Optimization used during the Twiddle phase. Stop the execution of the current twiddle trial
				* if the speed get stacked somewhere. This is very useful for very bad PID coefficients 
				* combination or for parameters too high
				*/
			  
				if (pid_pos.number_opt_steps >= 2*MIN_STEPS && speed < 4) {
					cout << "Interrupt optimization because too low speed" << endl;
					stopOptimization = true;
					pid_pos.total_error += 5000;
				}
				
				if ((stopOptimization || pid_pos.number_opt_steps >= MAX_OPT_STEPS) && !pid_pos.optimized) {
					cout << "==================================================" << endl;
					cout << "Current error:" << pid_pos.TotalError() << " - Best error: " << pid_pos.best_opt_error << endl;
					cout << "p: " << pid_pos.pid_coeff[0] << " - i: " << pid_pos.pid_coeff[1] << " - d: " << pid_pos.pid_coeff[2] << endl;
					cout << "dp: " << pid_pos.dp[0] << " - di: " << pid_pos.dp[1] << " - dd: " << pid_pos.dp[2] << endl;
					cout << "==================================================" << endl;

					pid_pos.Twiddle();

					pid_pos.ResetTwiddleParams();
					std::string msg = "42[\"reset\",{}]";
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
				if(pid_pos.optimized)
					WriteIntoFileResults(pid_pos, 0);

			}
			// DEBUG
			//std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
			
			json msgJson;
			msgJson["steering_angle"] = steer_value;
			msgJson["throttle"] = throttle;// (speed > 20) ? 0.0 : 0.2;
			auto msg = "42[\"steer\"," + msgJson.dump() + "]";
			(ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
	}
    else {
		// Manual driving
		std::string msg = "42[\"manual\",{}]";
		(ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
}
});

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    (ws).close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

