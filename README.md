# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Implementaiton


### The Model
Model Predictive Control (MPC) is a non-linear system and runs in the 
Unity car simulator. This provides the car's 2D location (x, y coordinates), 
car's orientation angle between x and y (psi) and 
a yellow projected reference line (waypoints) in the car's coordinates and 
the velocity of the car.

The waypoints are converted into car coordinate space. The converted waypoints are 
fit to a third order polynomial. 

The cross track error (cte) is the coefficients of the previous third order 
evaluated polynomial aka the car's tracking error to the trajectory polynomial.

The orientation error (epsi) is the change in error (derivate) of the car’s movement.

The state vector is defined by using the car coordinates x=0, y=0 and psi=0. 
Velocity (v) is provided by the simulator. cte and epsi are calculated using the 
Eigen header files, state << 0., 0., 0., v, cte, and epsi.

Next state is updated by the ipopt library Solve function. 
The inputs are the state and the coefficients. The car's coordinates will always be 
x=0, y=0 and psi=0 as it is the reference point. 

The following updated values are then sent to the simulator; 
steering angle, throttle, x, y, and the waypoints ptsx, ptsy. 

The updated waypoints are displayed as a green line. 

The goal is to have the yellow and green lines match, while ensuring smooth turning and acceleration.

### Timestep Length and Elapsed Duration (N and dt)
The prediction horizon (T = N * dt) is the duration over which future predictions 
are made.
It is recommended in real-world scenarios to have this horizon be no more than 
a few seconds as the environment will change enough that it doesn't make sense 
to predict any further into the future.

Given this I reduced T to 1 second (N = 10, dt = 0.1) from 
10 seconds (N = 25, dt = 0.05) of predicting a future continuous reference trajectory.


### Polynomnial Fitting and MPC Preprocessing

* Convert the waypoints into the car's coordinate space
    ```
    // Need Eigen vectors for polyfit
    Eigen::VectorXd ptsx_car(ptsx.size());
    Eigen::VectorXd ptsy_car(ptsy.size());
    
    // Transform the points to the vehicle's orientation
    for (int i = 0; i < ptsx.size(); i++) {
        double x = ptsx[i] - px;
        double y = ptsy[i] - py;
        ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
        ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
    }
    ```
* Fit a 3rd-order polynomial to the above x and y coordinates
    ```
    auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
    ```

### Model Predictive Control (MPC) with Latency
* This was extremely challenging for me and struggled with how to 
handle a 100 millisecond actuation delay correctly. Eventually after 
reading this recent [blog](https://medium.com/towards-data-science/dude-wheres-my-car-looking-back-at-udacity-sdcnd-term-2-e518fede30a1)
I decided to integrate that [solution](https://github.com/mvirgo/MPC-Project/blob/master/src/main.cpp) into my code base 
where the predicated state of the car, at the end of the latency 
period, is fed into the Solve function, making all resulting calculations
 appear to be real-time, based on the original prediction. Nice!
(credit to [mvirgo](https://github.com/mvirgo)).

### Successful Lap and PID Jerkiness versus MPC Smoothness
* [1 lap success](https://www.youtube.com/watch?v=TGU2ytNAFns)
* PID Jerkiness
    * !(here)[./pid_controller_5_sec.gif]
* MPC Smoothness
    * !(here_also)[./mpc_faster_5_sec.gif]
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
