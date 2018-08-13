# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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

## Project Rubric


### The Model

The vehicle model used for this project is the bicycle model which was presented in the course material. The assumptions for this model are:

* No tire slip
* No longitudinal/lateral forces
* The front wheels have the same speed and steer angle
* No drag

This model has four degrees of freedom and is represented by the following four state equations:

x[t+1] = x[t] + v[t] * cos(psi[t]) * dt

y[t+1] = y[t] + v[t] * sin(psi[t]) * dt

psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt

v[t+1] = v[t] + a[t] * dt

Where x is the vehicle's longitudinal position, y is the vehicle's lateral position, psi is the yaw rate of the vehicle, and v is the speed in the direction of travel. The two inputs to the system are the acceleration (a) and the steering angle (delta).

### Timestep Length and Elapsed Duration (N & dt)

For the simulation, my final selection for N and dt was 15 and 100 ms respectively.

As a starting point, I began with the values used in the "MPC Quizzes" (25, 50 ms). While running the simulation, it was clear that a timestep of 50 ms was much too frequent relative to the distance covered by the vehicle. This was causing undue processing overhead without contributing a performance increase, so I increased the timestep to 100 ms.

After increasing the timestep, I decided to fit the horizon to a value which would capture the upcoming road geometry while balancing processing costs. Starting with the MPC Quizzes value of 25 iterations, I realized that this was looking too far into the future for the the set speed that I was planning to operate the simulation at (80 mph). The first problem with this is the processing cost of looking that far ahead. The second is that there is an inverse correlation between the length of the horizon and the accuracy of a curve fit over that length. Working my way down form 25 iterations, I found that a horizon of 15 iterations fully captured a single curve on the test track.

### Polynomial Fitting and MPC Preprocessing

#### Waypoints

The waypoints were first converted from global coordinates to a local frame, then fitted to a 3rd degree polynomial. I selected a 3rd degree polynomial because it is the least complicated polynomial which is able to approximate an S-curve.

#### Vehicle State

The vehicle state is also given in a global reference system, and must be converted to a vehicle frame. In the vehicle frame x, y, and psi are all zero.

#### Actuators

The only preprocessing of the actuators was to convert the steer angle from degrees to radians.

### Model Predictive Control with Latency

#### Constraints and Weighting

I used the constraints formulation which was explored in the "MPC Quizzes" including cross track error and yaw error. This added two additional equations to my state:

cte[t+1] = cte[t] + (v[t] * sin(epsi[t]) * dt);

epsi[t+1] = epsi[t] - v[t] * delta[t] / Lf * dt;

In total, the variable constraints I used were cte, epsi, v, delta, accel, ddelta, and jerk. I prioritized a high weight on epsi, delta, and ddelta (the rate change of delta). This prioritized smooth steering over other parameters. Velocity, acceleration, and jerk were all minimally weighted so that the vehicle would tolerate changing speed to accommodate the other constraints. The CTE was tuned up just slightly so that the vehicle would try to stay centered.

#### Latency

I experimented with a method for delaying the actuators which was discussed on the slack forums. This technique involved applying actuator values which were 100 ms behind the rest of the vehicle state. This ultimately is not robust because it requires the MPC timestep and actuator latency to be multiples of one another. It was easy to implement, but ultimately did not achieve great results and did not make intuitive sense. Ultimately abandoned this method in favor of the method described in the Vehicle Model Lesson.

This method involves evaluating the vehicle state at a time equivalent to the actuator latency. So for an actuator latency of 100 ms, we predict where the vehicle will be in 100 ms and use that for the "current state" that we feed to the MPC.

The drawback of this method is that the kinematic bicycle model does not predict the future state with 100% accuracy. However, this approach does make sense holistically and can account for changing actuator latency.

### Conclusion

In the end I was able to drive the vehicle around the track using Model Predictive Control. If I were to further improve this process, I would include a module to adjust vehicle speed setpoint based on upcoming road curvature. Much like a normal driver does, when a curve appears ahead I would have the vehicle slow down ahead of the curve.

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.
