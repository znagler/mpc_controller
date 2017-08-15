# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

### Discussion


The PID controller uses a car's error signal, which is the perceived distance between the center of the lane and the car itself, along with 3 terms – proportional, integral and derivative – to control the car.  Each term has a logical purpose in making the controller work.  The 'P' proportional term is the most obvious; it says, if the car is to the right of the lane, aim left, and vice versa.  The 'I' integral term serves to correct wheel alignment issues.  It constantly sums up all accumulated error, and says, if the car is consistently to the right of the lane, aim a bit to the left.  The 'D' derivative term looks at the change in error from the previous signal, which is effectively the angle of the car relative to the road.  If the car is angled to the right relative to the road, turn to the left.  When used together and weighted properly, this system can output a desireable steering angle for a car.

Using the simulator, adjusting the terms had different effects on how the car drove.

## 1. Proportional
When the the proportional coefficient was negative and the other two coefficients set to zero, the car was able to drive fairly safely.  When it started to veer toward one side of the road it would correct itself and turn in the opposite direction.  The issue is that it would then overshoot the center of the road, and go too far in the other direction before repeating the same process.

## 2. Integral
The integral coefficient did not have a helpful effect in the simulator, probably because the car did not have wheel alignment issues.  While a successful tuning could surely be made with a non-zero integral coefficient, in this problem I found it easier to set it to zero.

## 3. Derivative
When set to a large (negative) value, the derivate coefficient could make the car drive somewhat safely even with the other two coefficients at zero.  This is because any time the car didn't think it was straight relative to the road, it immediately corrected.  The problem with only using this term is that the wheels jitter left and right extremely quickly as the car aimed to stay perfectly straight.

## Conclusion 
To achieve a ride that was 1- not swerving back and forth like a sine wave, and 2- not jittering back and forth, I set the 'P' and 'D' coefficients to small negative values and the 'I' coefficient to 0.  With manual tuning, the coefficients that worked well for me were:

 | Coefficient        | Value   | 
|:-------------:|:-------------:| 
| Kp   | -0.25     | 
| Ki       |  0      |
| Kd    | -2   |

To improve the driving of the simulator car, I also adjusted the car's throttle depending on the speed as follows:

| Speed    | Set - throttle   | 
|:-------------:|:-------------:| 
| < 25 mph   | 0.3     | 
| >= 25 mph       |  0      |

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