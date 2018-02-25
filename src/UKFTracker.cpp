#include "UKFTracker.h"
#include "Eigen/Dense"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/*

TODO

Use a base abstract class and extend it here with a custom constructor for each
specific type of Kalman Filter, as they require different initialization.

The rest of the code can be common for all of them.

*/


UKFTracker::UKFTracker() {
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // State covariance matrix:
    MatrixXd P(5, 5);

    // TODO: Try to unitilize it using the noise values
    // See lesson 32

    P <<
        1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1000, 0, 0,
        0, 0, 0, 1000, 0,
        0, 0, 0, 0, 1000,

    // INITIALIZE UKF:

    ukf_.initStateCovarianceMatrix(P);
    ukf_.initNoise(30, 30, 0.15, 0.15, 0.3, 0.03, 0.3); 

    // NOISES LEGEND:

    // Process noise standard deviation longitudinal acceleration in m/s^2
    // Process noise standard deviation yaw acceleration in rad/s^2

    // DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.

    // Laser measurement noise standard deviation position1 in m
    // Laser measurement noise standard deviation position2 in m
    // Radar measurement noise standard deviation radius in m
    // Radar measurement noise standard deviation angle in rad
    // Radar measurement noise standard deviation radius change in m/s

    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
}


UKFTracker::~UKFTracker() {}


void UKFTracker::processMeasurement(const MeasurementPackage &pack) {

    // INITIALIZATION:

    if (!is_initialized_) {
        initialize(pack);

        return; // Done initializing. No need to predict or update.
    }


    // PREDICTION:

    // Compute the time elapsed between the current and previous measurements:
    const float dt = (pack.timestamp_ - previous_timestamp_) / 1000000.0;	// In seconds.

    previous_timestamp_ = pack.timestamp_;

    // Makes the prediction:
    ukf_.predict(dt);


    // UPDATE:

    if (pack.sensor_type_ == MeasurementPackage::RADAR) {
        ukf_.updateRadar(pack.raw_measurements_);
    } else {
        ukf_.updateLidar(pack.raw_measurements_);
    }

    // OUTPUT current state and state covariance:
    // cout << "x_ = " << ekf_.x_ << endl;
    // cout << "P_ = " << ekf_.P_ << endl;
}


VectorXd UKFTracker::getCurrentState() {
    return ukf_.getCurrentState();
}


void UKFTracker::initialize(const MeasurementPackage &pack) {
    const VectorXd measurements = pack.raw_measurements_;
    const MeasurementPackage::SensorType type = pack.sensor_type_;

    if (type == MeasurementPackage::RADAR) {
        // Convert radar from polar to cartesian coordinates and initialize state:

        const float rho = measurements[0]; // Range
        const float phi = measurements[1]; // Bearing

        // rho (range), phi (bearing), rho_dot (velocity)
        ukf_.initState(rho * cos(phi), rho * sin(phi), 0, 0, 0);
    } else if (type == MeasurementPackage::LASER) {
        // Set the state with the initial location and zero velocity:

        ukf_.initState(measurements[0], measurements[1], 0, 0, 0);
    }

    // OUTPUT initial value:
    // std::cout << "INITIAL x = " << ekf_.getCurrentState().transpose() << std::endl;

    previous_timestamp_ = pack.timestamp_;
    is_initialized_ = true;
}