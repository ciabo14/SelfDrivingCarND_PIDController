# SelfDrivingCarND_UnscetedKalmanFilter

Implementation of a PID controller with the aim to control trajectory and speed of a car in the Simulation environent provided by Udacity in the scope of the Self driving car NanoDegree. Using the car information about speed, trajectory and The simulator provide for each timestamp the information about the car distance from the target trajectory, the current speed and the current steering angle, the PID controlloer adjust the speed and the trajectory, providing to the simulator a throttle value and a steering aangle.


## Repository structure

The repository is made of two folders and some files:

1.  /src folder: includes all the src files required for executable build and to execute the PID controller;
2.	/executables folder: contains the compiled files
2.  CMakeLists.txt, CMakeSettings.json and cmakepatch.txt are the configuration files used to make the system executable on windows. 
4.  install-mac.sh and install-ubuntu.sh: scripts for mac and ubuntu provided by udacity in order to make the configuration of the OS faster (e.g. uWebSockets installation) 

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

## Windows Install Instructions 

The system is completely configured in a windows machine, and then developed on a VS17 community environment. The installation instruction for windows are:

1. Install, in your root c:/ directory, vcpkg https://github.com/Microsoft/vcpkg. Keep attention to the installation instructions
2. Install python (min ver. 2.7) - dependency for libuv
3. cd to directory where you installed vcpkg (there should be placed the vcpkg.exe) and then type: ./vcpkg install uWebsockets 
4. Open CMakeSetting.json provided in the main root of this project, check if C:/vcpkg/scripts/buildsystems/vcpkg.cmake is pointing to the correct directory to your vcpkg and DCMAKE_TOOLCHAIN_FILE matches the output from vcpkg integrate.
5. Open in VS17 community edition, build pid.exe in x86 debug.
6. Enjoy with your PID controller

*Note* The simulator and the pid.exe will talk throught uWebsockets. So you need to start the simulator in the autonomous way to let the executable to work

## Editor Settings

The entire project is developed using VS17 community

## Controller Skeleton

The skeleton of the PID project is made by two main parts: 

1. *main.cpp* which include all the logic for the simulator connection as well as the heat to the optimization phase; 
2. PID.h and PID.cpp which includes the logic of the controller itself. 

### Optimization structure

The optimization structure is developed around the well known **Twiddle** (coordinate ascent) optimization algorithm . The algorithm, implemented within the class PID.cpp, sequentially adjust the controller coefficients in order to find the best solution. 

However, in order to make the procedure fast, three different sub procedures were adopted:

1. Avoid to check the PID behaviour with negative coefficients
2. Interrupt the optimization as soon as the error for the current trial is bigger than the optimum value
3. Stop the optimization trial as soon as the car got stacked.

Even if the first two are not true for each application, for the purpose of the projects appears to work properly and with a sensible reduction of optimization timings.

#### Avoid to check the PID behaviour with negative coefficients

Every time the PID is called for the Twiddle procedure, it implement this consistency check, which allow to avoid the optimization trial in case the parameter to be adjusted will be negative

Below the codes who implement this check.

```c++
...
if (2 * dp[current_opt_par] <= pid_coeff[current_opt_par]) {
	pid_coeff[current_opt_par] -= (2 * dp[current_opt_par]);
	twiddle_phase = 2; 
}
else {
	//Restore Original value for pid_coeff[current_opt_par]
	pid_coeff[current_opt_par] -= dp[current_opt_par];

	twiddle_phase = 1;
	ComputeNextStepParam(true, 0.9);
}
...
```
#### Interrupt the optimization as soon as the error for the current trial is bigger than the optimum value

A consistency check about the current error respect to the best is executed every movement step.


```c++
...
if (pid_pos.TotalError() > pid_pos.best_opt_error && pid_pos.number_opt_steps >= MIN_STEPS * 2) {
	cout << "Interrupt optimization because error overcome the best" << endl;
	stopOptimization = true;
}

```
The check start only if 2 conditions are met: 

1. the current total error overcame the best optimization error found so far;
2. the car as run at least 2 times the minimum values of steps required for otpimization

#### Stop the optimization trial as soon as the car got stacked.

When the car got stacked, its speed is constanstly = 0 for several steps. In this case the optimization step can be stopped and the procedure can go ahead with the next trial

```c++
...
if (pid_pos.number_opt_steps >= MIN_STEPS && speed < 4) {
	cout << "Interrupt optimization because too low speed" << endl;
	stopOptimization = true;
	pid_pos.total_error += 5000;
}

```
Also in this case the system takes into consideration also the min number of steps during the optimization phase.