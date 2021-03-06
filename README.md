CarND · T2 · P2 · Unscented Kalman Filter (UKF) Project
=======================================================

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="output/images/004 - Simulator Rotated.png" alt="Unscented Kalman Filter visualization on the simulator." />


Project Overview
----------------

In this project utilize an Unscented Kalman Filter (UKF) to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining `RMSE` values that are lower that the tolerance outlined in the [project rubric](https://review.udacity.com/#!/rubrics/783/view):  `px = 0.09, py = 0.10, vx = 0.40, vy = 0.30`, which are obviously lower than those required in the [previous project](https://github.com/Danziger/CarND-T2-P1-Extended-Kalman-Filter):  `px = 0.11, py = 0.11, vx = 0.52, vy = 0.52`.

To test it, [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) need to be used. The latest version of `main.cpp` used to run this project without the simulator can be found [here](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project/blob/10814755085e9e8b34050216baf73abaa6f1e9e9/src/main.cpp).

If you are looking for Udacity's started code project, you can find it [here](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project).


Dependencies
------------

- [Udacity's Self Driving Car Simulator](https://github.com/udacity/self-driving-car-sim/releases)
- [`cmake >= 3.5`](https://cmake.org/install/)
- `make >= 4.1` (Linux / [Mac](https://developer.apple.com/xcode/features/)), [`3.81` (Windows)](http://gnuwin32.sourceforge.net/packages/make.htm)
- `gcc/g++ >= 5.4` (Linux / [Mac](https://developer.apple.com/xcode/features/)), [`MinGW` (Windows)](http://www.mingw.org/)
- [`uWebSockets` commit `e94b6e1`](https://github.com/uWebSockets/uWebSockets). See the following section for installation instructions and additional details.
- [`Eigen`](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.


Installation
------------

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets):

- `install-mac.sh` for Mac.
- `install-ubuntu`for either Linux or [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) (**please, make sure it is updated**).

For Windows, Docker or VMware coulso also be used as explained in the [course lectures](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77). Details about enviroment setup can also be found there.

If you install from source, checkout to commit `e94b6e1`, as some function signatures have changed in `v0.14.x`:

    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1

See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.


Build & Run
-----------

Once the install is complete, the main program can be built and run by doing the following from the project top directory:

1. Create a build directory and navigate to it: `mkdir build && cd build`
2. Compile the project: `cmake .. && make`
3. Run it: `./UKF`

Or, all together (from inside the `build` directory): `clear && cmake .. && make && ./UKF`

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).


Relevant Changes
----------------

TODO


Data Flow
---------

TODO


Results
-------

TODO


Project Passing Criteria
------------------------

There are several criteria that must be fulfilled to pass the project.

- The overall processing chain (prediction, laser update or radar update depending on measurement type) must be correct.
- The student is not allowed to use values from the future to reason about the current state.
- It must be possible to run the project in three different modes: considering laser only, with considering radar only, or with using both sensors.
- For every mode, the overall RMSE (2d position only) may not be more than 10% increased to what the original solution is able to reach (this number depends on the individual measurement sequence)
- The RMSE of laser AND radar must be lower than radar only or laser only
- The NIS of radar measurements must be between 0.35 and 7.81 in at least 80% of all radar update steps.


Generating Additional Data
--------------------------

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.


Interesting Resources
---------------------

- [Detailed explanations (55 videos) about Kalman Filters by Michel van Biezen @ iLectureOnline.](https://www.youtube.com/watch?v=CaCcOwJPytQ)

- [A New Method for the Nonlinear Transformation of Means
and Covariances in Filters and Estimators](https://www3.nd.edu/~lemmon/courses/ee67033/pubs/julier-ukf-tac-2000.pdf)

- [Udacity Discussion Forum - Numerical instability of the implementation](https://discussions.udacity.com/t/numerical-instability-of-the-implementation/230449)

- [Udacity Discussion Forum - UKF getting stuck on second dataset](https://discussions.udacity.com/t/ukf-getting-stuck-on-second-dataset/240080)

- [Self-Driving Car Project Q&A | Unscented Kalman Filters](https://www.youtube.com/watch?v=gej_-2hxgck)


--------




Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]



## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.
