# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

The purpose of the project was to implement a MPC (Model, Predictive and Control) controller and tune it so that the vehicle drives safely around the track.

## Rubric Tasks

### The Model 

The kinematic model contiains various  amounts of information including the vehicles x and y coordinates, orientation (psi) and velocity as well as cross-track error (CTE) and psi error (epsi). The outputs of the model inculdes actuators such as acceleration and delta (steering angle). 
The model takes into account the state and actuations from the previous timestep to calculate the state for the current timestep based on the equations below:

~~~

x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t-1] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t-1] / Lf * dt

~~~

### Timestep Length and Elapsed Duration (N & dt)

The values chosen for N and dt originally were 10 and 0.1 respectively to first give stable performance at lower speeds.

Once a benchmark performance was reached dt was reduced to 0.05 to allow for greater speeds. A N value of 12 was chosen so as to keep the horizon T to an OK level,  the values selected results in a T of 0.6s.
As the vehicle is travelling at higher speeds a lower T is more acceptable. It was found that when trying to increase N any further resulted in deteriorating performance. 

### Polynomial Fitting and MPC Preprocessing

The waypoints were transformed from the map co-oridinates to the vehicles co-ordinates using the following code:

~~~

Eigen::MatrixXd pts_veh(2, ptsx.size());
		  for (size_t i = 0; i < ptsx.size(); i++)
		  {
			  pts_veh(0, i) = (cos(-psi) * (ptsx[i] - px)) - (sin(-psi) * (ptsy[i] - py)); // X component
			  pts_veh(1, i) = (sin(-psi) * (ptsx[i] - px)) + (cos(-psi) * (ptsy[i] - py)); // Y component
		  }
		
~~~

Note: px = 0, py = 0 and psi = 0 because this is the center of car space which made the calculations easier.

### Model Predictive Control with Latency

The latency was mainly handled by predicting the next state based on the latency applied to the kinematic model as per the equations below. Note here that px, py and psi are all 0s as we have transformed about the center point of the car. 

Thus it results in the following block of code:

~~~

	// Predict state after latency using kinematic model
	// x, y and psi are all zero after transformation above
	double pred_px = v * dt;
	double pred_py = 0.0; 
	double pred_psi = -v * delta / Lf * dt;
	double pred_v = v + a * dt;
	double pred_cte = cte + v * sin(epsi) * dt;
	double pred_epsi = epsi - v * delta / Lf * dt;

	Eigen::VectorXd state(6);
	state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

~~~

### Demo recording

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/35GWdBZPK6Q/0.jpg)](https://www.youtube.com/watch?v=35GWdBZPK6Q)


---


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
