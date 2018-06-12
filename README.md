# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
[lap]: ./images/final_lap.gif
[kinematic]: ./images/kinematic_equ.png
[cte]: ./images/cte.png
[orientation]: ./images/orientation_error.png

## Introduction
This project involved developing a non-linear model predictive controller (MPC) to drive a car around the simulated track provided in the [Udacity Self-Driving Car Simulator](https://github.com/udacity/self-driving-car-sim/releases). For this project, the simulator will provide the cars current x,y position, steering angle and velocity along with a series of waypoints plotting the course the car should follow. The MPC is then used to update the steering angle and acceleration to drive the car along the plotted course.

![alt final lap][lap]

## The Model

The MPC uses the global kinematic model. The global kinematic madel defines the state, actuators and how the state changes over time based on the previous state and current accuator inputs provided by the simulator using the equations:

![alt kinematic model][kinematic]

The Cross Track Error (CTE) is the error between the desired track (waypoints provided by the simulator) and the vehicle's current position. The CTE update equation is:

![alt cross-track error][cte]

The Orientation Error (ePSI) is the difference between the desired orientation and the current orientation. The ePSI update equation is:

![alt orientation error][orientation]

Using This model a cost function is derived from the reference state, acuator state and the gap between sequential actions:

    // The part of the cost based on the reference state.
    for (t = 0; t < N; t++) {
      fg[0] += tuneCte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += tuneEpsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += tuneV * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (t = 0; t < N - 1; t++) {
      fg[0] += tuneDelta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += tuneA * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (t = 0; t < N - 2; t++) {
      fg[0] += tuneDeltaDiff * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += tuneADiff * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }


This is then used with an optimizer (Ipopt) to find this control inputs which minimize the cost function. 

The cost function is further tuned by multiplying different elements of the cost functions by constants:

    const double tuneCte = 1.0;
    const double tuneEpsi = 10.0;
    const double tuneV = 1.0;
    const double tuneDelta = 10.0;
    const double tuneA = 10.0;
    const double tuneDeltaDiff = 1000.0;
    const double tuneADiff = 1.0;


## Parameters (N, dt)

The prediction horizon (T) is defined by two parameters, the number of timesteps in the horizon (N) and the time elapsed between each timestep (dt). The final values used are N=15, dt=0.05. This means there are 15 timesteps with a half-second duration between each timestep. Other values tried were the provided N=10, dt=0.1. The final values had a more precise fit, but shorter overall duration (.75 seconds vs. 1 second).

## Polynomial Fitting and MPC Pre-processing

In order to calculate the CTE and ePSI, we need to fit the waypoints to a polynomial. However, the waypoints are provided from the simulator in a global coordinate system. These are first transformed to the vehicle's coordinate system using:

    void transformPoints(const std::vector<double> &ptsx, 
                        const std::vector<double> &ptsy,
                        const double &px,
                        const double &py,
                        const double &psi,
                        const int &size,
                        Eigen::VectorXd &t_ptsx,
                        Eigen::VectorXd &t_ptsy) {

      for (int i = 0; i < size; i++) {
        double dx = ptsx[i] - px;
        double dy = ptsy[i] - py;
        t_ptsx(i) = dx * cos(-psi) - dy * sin(-psi);
        t_ptsy(i) = dx * sin(-psi) + dy * cos(-psi);
      }
    }

The result of this equation is a set of waypoints in the vehicle's coordinate position. A 3rd order polynomial is then fit to these waypoints using the provided polyeval() function.

The CTE and ePSI are then calculated from the polynomial coefficients. The y and psi terms are dropped, because the waypoint coordinates have already been transformed to the vehicle's coordinate system (x,y,psi) = (0,0,0):

    double cte = polyeval(coeffs, 0);
    double epsi = -atan(coeffs[1]);

## Dealing with Latency

In a real system, there is a latency between the acuator inputs being issued and when those inputs take effect. This is modelled by introducing a 100ms delay before the acutator inputs calculated by the MPC are sent to the simulator. 

The output of the optimizer is a series of acuator inputs at each timestep of the prediction horizon. To account for the latency, I simply return the actuator inputs two timesteps in the future (2*dt where dt=50ms). This simple approach proves effective.

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
