# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

### Discussion


## The Model

The state of the model has 6 components - the x-position, y-position, the angle `psi`, velocity, cross track error `cte`, and the orientation error `epsi`.
The actuator has two components - steering angle and throttle.

In the model, the state updates with the following equations:

x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v_[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt


## Timestep Length and Elapsed Duration (N & dt)

I set the timestep length, `N` to `10` and the duration, `dt` to `.1` . The timestep length affects how far into the future the model will consider when optimizing the cost function for the controller.  The `dt` affects the speed of the car, with a lower `dt` making the car faster and vice versa.  The car can complete a lap with a `dt` of `.3` or higher, but it does so much more slowly.

## Polynomial Fitting and MPC Preprocessing

First, I changed the waypoints from the global coordinate system to the car's coordinate system like this:

```double shift_x = ptsx[i]-px;
double shift_y = ptsy[i]-py;

ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
```

Then, I loaded the waypoints into a vector and fit them into a 3rd degree polynomial using `polyfit` like so:

```          
double* ptrx = &ptsx[0];
Eigen::Map<Eigen::VectorXd> ptsx_trans(ptrx,6);
double* ptry = &ptsy[0];
Eigen::Map<Eigen::VectorXd> ptsy_trans(ptry,6);          

auto coeffs = polyfit(ptsx_trans, ptsy_trans, 3);
```

## Model Predictive Control with Latency

The simulator mimics latency by sleeping the code thread for 100ms before actuating the controls.  To rememdy this, we need to always consider the state of the car 100ms in the future, instead of at its current position.  Using the help of the forum, I determined that each component of the vehicle state can be predicted using velocity `v`, the `latency` (.1 for 100ms), `steer_value`, the angle `psi` , cross track error `cte`, and the error-psi `epsi`:

```
double predicted_x = v * latency;
double predicted_y = 0;
double predicted_psi = v * -steer_value / Lf * latency;  
double predicted_v = v + throttle_value * latency;
double predicted_cte = cte +  (v * sin(epsi) * latency);
double predicted_epsi = epsi + v * -steer_value / Lf * latency;
```

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 