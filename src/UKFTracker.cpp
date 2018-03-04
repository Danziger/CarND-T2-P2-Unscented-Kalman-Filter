#include "UKFTracker.h"
#include "Eigen/Dense"

#include <iostream>
#include <iomanip>
#include <limits>


/*

TODO

Use a base abstract class and extend it here with a custom constructor for each
specific type of Kalman Filter, as they require different initialization.

The rest of the code can be common for all of them.

*/


// PRIVATE:


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
    // cout << "INITIAL x = " << ekf_.getCurrentState().transpose() << endl;

    previous_timestamp_ = pack.timestamp_;
    is_initialized_ = true;
}


// PUBLIC:


UKFTracker::UKFTracker() {
    // Sensor measurement noises:
    // DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.

    // Laser measurement noise standard deviation position1 in m:
    const double std_laspx = 0.15;

    // Laser measurement noise standard deviation position2 in m:
    const double std_laspy = 0.15;

    // Radar measurement noise standard deviation radius in m:
    const double std_radr = 0.3;

    // Radar measurement noise standard deviation angle in rad:
    const double std_radphi = 0.03;

    // Radar measurement noise standard deviation radius change in m/s:
    const double std_radrd = 0.3;

    // DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.


    // Measurement covariance matrixes:

    MatrixXd R_lidar(2, 2);
    MatrixXd R_radar(3, 3);

    R_lidar <<
        std_laspx * std_laspx, 0,
        0, std_laspy * std_laspy;

    R_radar <<
        std_radr * std_radr, 0, 0,
        0, std_radphi * std_radphi, 0,
        0, 0, std_radrd * std_radrd;

    // State covariance matrix:
    MatrixXd P(5, 5);

    // TODO: Try to initilize it using the noise values
    // See lesson 32

    P <<
        2, 0, 0, 0, 0,
        0, 4, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 0.5, 0,
        0, 0, 0, 0, 0.5;

    // INITIALIZE UKF:

    ukf_.initMatrixes(P, R_lidar, R_radar);

    // Adjust these noises:
    // Process noise standard deviation longitudinal acceleration in m/s^2
    // Process noise standard deviation yaw acceleration in rad/s^2
    
    const double std_a = 1;
    const double std_yawdd = 0.5;

    ukf_.initNoise(std_a, std_yawdd);
                 
    cout
        << setprecision(2) << fixed
        << endl
        << "──────────────────────────────────────────────────────" << endl
        << endl
        << " STD DEV LNG ACC = " << setfill(' ') << setw(5) << std_a << " MET / S ^ 2" << endl
        << " STD DEV YAW ACC = " << setfill(' ') << setw(5) << std_yawdd << " RAD / S ^ 2" << endl
        << endl
        << "──────────────────────────────────────────────────────" << endl;

    // TODO: Add lambda here
}


UKFTracker::~UKFTracker() {}


vector<double> UKFTracker::processMeasurement(const MeasurementPackage &pack) {

    // INITIALIZATION:

    if (!is_initialized_) {
        initialize(pack);

        vector<double> empty(5);

        return empty; // Done initializing. No need to predict or update.
    }


    // PREDICTION:

    // Compute the time elapsed between the current and previous measurements:
    const double dt = (pack.timestamp_ - previous_timestamp_) / 1000000.0;	// In seconds.

    previous_timestamp_ = pack.timestamp_;

    // Makes the prediction:
    ukf_.predict(dt);


    // UPDATE:

    // OUTPUT current state and state covariance:
    // cout << "x_ = " << ekf_.x_ << endl;
    // cout << "P_ = " << ekf_.P_ << endl;

    if (pack.sensor_type_ == MeasurementPackage::RADAR) {
        const double NIS_radar = ukf_.updateRadar(pack.raw_measurements_);

        return updateNIS(NIS_radar, total_radar_, NIS_3_table_, radar_NIS_results_);
    } else {
        const double NIS_lidar = ukf_.updateLidar(pack.raw_measurements_);

        return updateNIS(NIS_lidar, total_lidar_, NIS_2_table_, lidar_NIS_results_);
    }
}


VectorXd UKFTracker::getCurrentState() {
    return ukf_.getCurrentState();
}


vector<double> UKFTracker::updateNIS(
    double current_NIS,
    int &total_measurements,
    vector<double> &NIS_table,
    vector<int> &NIS_results
) {
    vector<double> NIS_stats;

    NIS_stats.push_back(current_NIS);

    ++total_measurements;

    int i = -1;

    for (vector<double>::iterator it = NIS_table.begin(); it != NIS_table.end(); ++it) {
        if (current_NIS > *it) {                
            NIS_stats.push_back(100 * ++NIS_results[++i] / total_measurements);
        } else {
            NIS_stats.push_back(100 * NIS_results[++i] / total_measurements);
        }
    }

    return NIS_stats;
}