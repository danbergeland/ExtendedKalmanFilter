# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project, a mix of radar and laser measurements are read from a text file in to an Extended Kalman Filter.  The filter predicts and updates the position of the tracked object using the Extended Kalman Filter formulas.  The laser data is provided as X, Y coordinate data, and the radar is in polar coordinates (rho, phi, rho-dot).  

The extended Kalman Filter is able to improve the accuracy of localizing a tracked object by using statics to infer more precise coordinates!  To accomplish this, the data from the radar needs to be converted from polar to cartesian coordinates.  Since the arctan function of y/x is used to infer phi, it creates non-linearities in updates.  To accomodate the mathematical distortion of the different sensor coordinates, the Taylor series expansion is used. 

Recall that the Taylor series approximates functions as h(x) = h(u)+ d(x-u)/dx

The Jacobian matrix is calculated for the updated x,y positions at each additional data point.  The Jacobian provides the partial derivatives for polar to cartesian values (e.g. dphi/dx, dphi/dy), which allows for matrix multiplication with polar coordinates to result in projected cartesian values.

The code demonstrates the improved accuracy over either sensor.  Position accuracy using root mean square error (RMSE) goes from over .1 using only laser updates, and .15 using radar, down to .05.  Sensor fusion, therefor, can take two sensors with similar error and generate a more accurate sensing pipeline.

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

