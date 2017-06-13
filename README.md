# Model Predictive Controller

![alt text](./documentation_images/mpc.png "Intro image")


## The Model

The model has a few different components. The state, actuators and update equations.
We are using a simple kinematic model with the following state variables:

#### State Variables
1. `x` The x position of the car.
2. `y` The y position of the car.
3. `psi` The current steering angle in radians.
4. `v` The current velocity in mph.

#### Error Variables
Note: We also added the following to our state representation (although technically not part of the state)

1. `cte` The cross track error is the difference between our desired position and actual position.
2. `epsi` The orientation error is the difference beween our desired heading and actual heading.

#### Actuator Variables

1. `delta` The steering angle. 
2. `a` This is the throttle, brake and reverse combined. Range is`[-1,1]`.

#### Update equations

The state and errors update equations are below.
```
Lf accounts for the turning radius differences in cars; distance from front of car to its Center-of-Gravity.

x' = x + v * cos(psi) * dt
y' = y + v * sin(psi) ( dt)
psi' = psi + v / Lf * delta * dt
v' = v + a * dt

cte' = cte - v * sin(epsi) * dt
epsi' = epsi +  v / Lf * delta * dt

where:
 cte  = f(x) - y
 epsi = psi - desired_psi
 desired_psi = arctan(f'(x))
 
 f(x) = referenced trajectory
```

## Timestep Length and Elapsed Duration (N & dt)

The hyperparameters N and dt can have dramatic effect of the model outcome.

* `N` The number of timesteps to predict
* `dt` The time between actuations. 
* `T = N*dt` The total time into the future to predict.

Having `T` span beyond one second seems to give me useless results.
Also having `dt` being too small was causing too many actuations and required much compute time.
I started with `N=15 and dt=.05` this was causing to much zig-zaging.
I finally landed on `N=10 and dt=0.1` to give me stable results. It was a manual tuning process.

## Polynomial Fitting and MPC Preprocessing

The simulator provides you with global (X,Y) way points on each update. I first
transformed them to the car coord. 
```
VectorXd x_way_points(ptsx.size());
VectorXd y_way_points(ptsy.size());
for (int i = 0; i < ptsx.size(); ++i) {
  const double dx = ptsx[i]-px;
  const double dy = ptsy[i]-py;

  x_way_points(i) = dx * cos(0-psi) - dy * sin(0-psi);
  y_way_points(i) = dx * sin(0-psi) + dy * cos(0-psi);
}
```

Finally, I fit a third order polynomial to these way points.
The coefficients from the polynomial is what the solver uses to 
make the predictions.

## Model Predictive Control with Latency

Dealing with latency is pretty simple because of MPC.
We can just use the update equation to progress the current
car state to the expected latency in time and use that as the
initial state for the solver.

```
const double time_delay_px    = 0.0 + v * time_delay;
const double time_delay_py    = 0.0;
const double time_delay_psi   = 0.0 + v * (-delta) / Lf * time_delay;
const double time_delay_v     = v + a * time_delay;
const double time_delay_cte   = cte + v * sin(epsi) * time_delay;
const double time_delay_epsi  = epsi + v * (-delta) / Lf * time_delay;
```

## MPC Solver: IPOPT/CPPAD and Objective Function

The objective function I used is below:

```
 x = state
 f(x) = w1 * cte^2 +            //this is the cross track error
        w2 * epsi^2 +           //error in heading
        w3 * (v - refv)^2 +     //error on matching reference velocity
        w4 * delta^2 +          //error for steering
        w5 * a^2 +              //error for throttle
        w6 * (a' - a)^2 +       // error for huge gaps between throttle changes.
        w7 * (delta`'- delta)^2 // error for huge gaps between steering changes.
```
Normally you'd need to provide the jacobian and hessian functions for your
objective function in order for the IPOPT solver to work, but 
we used CPPAD to do automatic differentiation!

## Objective Function Weights

Notice I added weights `w1-w7` which had to also be tuned. The weights I 
eventually landed on are below:

```
const double cte_weight           = 1800.0;
const double epsi_weight          = 1800.0;
const double v_weight             = 1.0;
const double delta_weight         = 20.0;
const double throttle_weight      = 10.0;
const double delta_diff_weight    = 250.0;
const double throttle_diff_weight = 15.0;
```

I gave high cost to important factors and low weights to less priority factors.
For example, my reference velocity was 99 mph, but it wasn't more important than
staying on the road.

## Final thoughts

This was fun to get working, but manual tuning was a pain. I'd be interested
in a more realistic simulator; also using a more dynamic models that 
take mass, forces on tires and gravity into account.

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

