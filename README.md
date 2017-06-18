# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

# Project Overview
Goal of this project is to implement Model Predictive Control (MPC) to drive the car around the track on a simulator. The simulator provides a feed of values containing the position, speed and heading direction of the car. Also it provides the coordinates of a serial waypoints along a reference trajectory. These coordinates are provided in a global coordinate system. 

Desigh Requirements are as follows: 

* Errors: Compute the errors between the reference trajectory and the vehicle’s actual path
* Actuator Constraints: 
* Actuator latency: The MPC controller should handle additional latency (saying 100ms) between commands.
* Test MPC controller on the simulator and make sure the vehicle is able to drive successfully around the track.

# Final Result
[Here](https://youtu.be/gDbm4EzFik8) is the video that demonstrates the vehicle controlled by MPC successfully drives around the track in the simulator.
![mpc](https://user-images.githubusercontent.com/24623272/27259729-4539316c-544c-11e7-8738-f385f02c34d3.png)

### The Model
The vehicle kinematic model is used in this project, which ignore all dynamical effects such as the complex interactions between the tires and the road determine the dynamics of the vehicle. 
The MPC system has three main components. 

#### 1. State vector and errors

[x,y,psi,v, cte, epsi] is the state of the vehicle, L​f​​ is a physical characteristic of the vehicle.

#### 2. Actuators
[δ,a] are the actuators, or control inputs,

#### 3. State update equations
```
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
Here, `x,y` denote the position of the car, `psi` the heading direction, `v` its velocity `cte` the cross-track error and `epsi` the orientation error. `Lf` is the distance between the center of mass of the vehicle and the front wheels and affects the maneuverability. `δ,a`,  which we denote as δ for steering angle and a for acceleration (throttle/brake combined). The vehicle model can be found in the class `FG_eval`. 

### Timestep Length and Elapsed Duration (N & dt)
The prediction horizon is the duration over which future predictions are made. We’ll refer to this as T.
T is the product of two other variables, N and dt. N is the number of timesteps in the horizon. dt is how much time elapses between actuations. N, dt, and T are hyperparameters you will need to tune for each model predictive controller you build. However, there are some general guidelines. T should be as large as possible, while dt should be as small as possible.

In the case,  I choose N = 16, dt= 0.05, such that drives the car smoothly around the track for velocities up to about 75mph.

### Polynomial Fitting and MPC Preprocessing

* MPC Preprocessing:  Use transformGlobalToVehicle to transform the given x and y coordinates of car position and representing waypoints, from global coordinate system to the vehicle coordinate system.
* Use polyfit to fit a 3rd order polynomial to the given x and y coordinates representing waypoints.
* Use evaluateCte and evaluateEpsi to get cross-track error and orientation error from fit.

The transformation used is 
```
      X =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
      Y =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);  
```
where `X,Y` denote coordinates in the vehicle coordinate system. Note that the initial position of the car and heading direction are always zero in this frame. Thus the state of the car in the vehicle cordinate system is 
```
          state << 0, 0, 0, v, cte, epsi;
```
initially. 

### Model Predictive Control with Latency
The additional latency (100ms) has significant impact on the result. When delays are not properly accounted for oscillations and/or bad trajectories can occur.

Two common approach to take delays into account:
1. First approach, When receiving states from the simulator, we first predict its state after 100 ms, and then feed this new state into the solver. The MPC trajectory is then determined by solving the control problem starting from that position. 

2. Another approach,  the control problem is solved from the current position and time onwards. Latency is taken into account by constraining the controls to the values of the previous iteration for the duration of the latency. Thus the optimal trajectory is computed starting from the time after the latency period. This has the advantage that the dynamics during the latency period is still calculated according to the vehicle model. 

Here, I chose the second approach . The actuations are forced to remain at their previous values for the time of the latency. This is implemented in 
`MPC::Solve` like so. 

```  
  // constrain delta to be the previous control for the latency time
  for (int i = delta_start; i < delta_start + latency_ind; i++) {
    vars_lowerbound[i] = delta_prev;
    vars_upperbound[i] = delta_prev;
  }
 ... 
  
  // constrain a to be the previous control for the latency time 
  for (int i = a_start; i < a_start+latency_ind; i++) {
    vars_lowerbound[i] = a_prev;
    vars_upperbound[i] = a_prev;
  }
```

Accordingly, the value that is fed to the simulator is then taken to be the first freely varying control parameter of the optimal trajectory:
```
          // compute the optimal trajectory          
          Solution sol = mpc.Solve(state, coeffs);

          double steer_value = sol.Delta.at(latency_ind);
          double throttle_value= sol.A.at(latency_ind);
```

 Note I set latency_idx = 2, which denote latency imply that the controls of the first two cycle steps are not used in the optimization. They are frozen to the values of the previous actuations. 


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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
