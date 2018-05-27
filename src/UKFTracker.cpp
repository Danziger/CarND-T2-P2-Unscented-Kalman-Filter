#include "UKFTracker.h"
#include "common/MeasurementPackage.h"
#include "common/tools.h"
#include "common/format.h"
#include "common/Eigen-3.3/Dense"

#include <iostream>
#include <iomanip>
#include <limits>


#define MAX_DT 2

#define RMSE_X_LIMIT 0.09
#define RMSE_Y_LIMIT 0.10
#define RMSE_VX_LIMIT 0.40
#define RMSE_VY_LIMIT 0.30


// DO NOT MODIFY sensor measurement noise values below these are provided by the sensor manufacturer.

// Laser measurement noise standard deviation position1 in m:
#define STD_X 0.15

// Laser measurement noise standard deviation position2 in m:
#define STD_Y 0.15

// Radar measurement noise standard deviation radius in m:
#define STD_R 0.3

// Radar measurement noise standard deviation angle in rad:
#define STD_PHI 0.03

// Radar measurement noise standard deviation radius change in m/s:
#define STD_RD 0.3

// DO NOT MODIFY sensor measurement noise values above these are provided by the sensor manufacturer.


/*

TODO

Use a base abstract class and extend it here with a custom constructor for each
specific type of Kalman Filter, as they require different initialization.

The rest of the code can be common for all of them.

*/


// PRIVATE:


void UKFTracker::initialize(const MeasurementPackage &pack) {

    // Execution time stats:
    time_ = 0;

    // Numerical inestability stats:
    inestability_ = 0;

    // Initialize RMSE:

    RMSE_sum_ = VectorXd(4);
    RMSE_ = VectorXd(4);

    RMSE_sum_ << 0, 0, 0, 0;
    RMSE_ << 0, 0, 0, 0;

    // Measurements count:

    total_ = 0;
    total_lidar_ = 0;
    total_radar_ = 0;

    // NIS stats:

    lidar_NIS_results_ = { 0, 0, 0, 0 };
    radar_NIS_results_ = { 0, 0, 0, 0 };


    // Initialize state covariance matrix P based on lidar noises:

    MatrixXd P(5, 5);

    P <<
        STD_X * STD_X, 0, 0, 0, 0,
        0, STD_Y * STD_Y, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, STD_PHI * STD_PHI, 0,
        0, 0, 0, 0, STD_RD * STD_RD;

    ukf_.setP(P);


    // Initialize state:

    const VectorXd measurements = pack.raw_measurements_;
    const MeasurementPackage::SensorType type = pack.sensor_type_;

    if (type == MeasurementPackage::RADAR) {
        // Convert radar from polar to cartesian coordinates and initialize state:

        const float rho = measurements[0]; // Range
        const float phi = measurements[1]; // Bearing

        // rho (range), phi (bearing), rho_dot (velocity)
        ukf_.setState(rho * cos(phi), rho * sin(phi), 0, 0, 0);
    } else if (type == MeasurementPackage::LASER) {
        // Set the state with the initial location and zero velocity:
        ukf_.setState(measurements[0], measurements[1], 0, 0, 0);
    }


    // PROMPT noise and lambda values:
    cout << setprecision(2) << fixed << endl;

    // Process noise standard deviation longitudinal acceleration in MET / S ^ 2
    cout << SEPARATOR << endl << endl;
    const double std_a = tools::prompt("STD DEV LNG ACC", "MET / S ^ 2", 1, 0, 100);

    // Process noise standard deviation yaw acceleration in RAD / S ^ 2
    cout << SEPARATOR << endl << endl;
    const double std_yawdd = tools::prompt("STD DEV YAW ACC", "RAD / S ^ 2", 0.5, 0, 100);

    // Set them in the UKF:
    ukf_.setNoise(std_a, std_yawdd);


    // TODO: Add lambda here


    // Log summary of params that will be used:

    cout
        << SEPARATOR << endl
        << endl
        << "   STD DEV LNG ACC = " << setfill(' ') << setw(5) << std_a << " MET / S ^ 2" << endl
        << "   STD DEV YAW ACC = " << setfill(' ') << setw(5) << std_yawdd << " RAD / S ^ 2" << endl
        << endl
        << SEPARATOR << endl;


    // Initialize previous_timestamp_:
    previous_timestamp_ = pack.timestamp_;
}


void UKFTracker::updateRMSE(VectorXd gt) {
    const VectorXd residual = getCurrentState() - gt;

    RMSE_sum_ += (residual.array().pow(2)).matrix();
    RMSE_ = (RMSE_sum_ / ++total_).array().sqrt();
}


vector<double> UKFTracker::updateNIS(
    double current_NIS,
    int total_sensor_measurements,
    vector<double> &NIS_table,
    vector<int> &NIS_results
) {
    vector<double> NIS_stats;

    NIS_stats.push_back(current_NIS);

    int i = -1;

    for (vector<double>::iterator it = NIS_table.begin(); it != NIS_table.end(); ++it) {
        if (current_NIS > *it) {                
            NIS_stats.push_back(100 * ++NIS_results[++i] / total_sensor_measurements);
        } else {
            NIS_stats.push_back(100 * NIS_results[++i] / total_sensor_measurements);
        }
    }

    return NIS_stats;
}


void UKFTracker::log(bool ok, char sensor, vector<double> NIS) {

    if (total_ % 10 == 1) {
        cout
            << "                     │                                    │" << endl
            << "     #  S  I    TIME │  RMSE X   RMSE Y  RMSE VX  RMSE VY │     NIS   NIS 95   NIS 90   NIS 10    NIS 5" << endl;
    }

    const double RMSE_X = RMSE_(0);
    const double RMSE_Y = RMSE_(1);
    const double RMSE_VX = RMSE_(2);
    const double RMSE_VY = RMSE_(3);

    cout
        << "  " << setfill(' ') << setw(4) << total_
        << "  " << sensor
        << "  " << (ok ? C_GREEN : C_RED ) << "■" << C_RST
        << setprecision(0) << fixed
        << setfill(' ') << setw(5) << 1000000 * time_ / total_ << " us"
        << " │ "
        << setprecision(3) << fixed
        << "  " << (RMSE_X > RMSE_X_LIMIT ? C_RED : C_GREEN) << RMSE_X
        << "    " << (RMSE_Y > RMSE_Y_LIMIT ? C_RED : C_GREEN) << RMSE_Y
        << "    " << (RMSE_VX > RMSE_VX_LIMIT ? C_RED : C_GREEN) << RMSE_VX
        << "    " << (RMSE_VY > RMSE_VY_LIMIT ? C_RED : C_GREEN) << RMSE_VY
        << C_RST << " │ "
        << setprecision(3) << fixed
        << setfill(' ') << setw(7) << NIS[0] << "  "
        << setprecision(1) << fixed
        << setfill(' ') << setw(5) << NIS[1] << " %  "
        << setfill(' ') << setw(5) << NIS[2] << " %  "
        << setfill(' ') << setw(5) << NIS[3] << " %  "
        << setfill(' ') << setw(5) << NIS[4] << " %" << endl;
}


// PUBLIC:


UKFTracker::UKFTracker() {

    // Initialize measurement covariance matrixes R_lidar and R_radar:

    MatrixXd R_lidar(2, 2);
    MatrixXd R_radar(3, 3);

    R_lidar <<
        STD_X * STD_X, 0,
        0, STD_Y * STD_Y;

    R_radar <<
        STD_R * STD_R, 0, 0,
        0, STD_PHI * STD_PHI, 0,
        0, 0, STD_RD * STD_RD;

    ukf_.setR(R_lidar, R_radar);
}


UKFTracker::~UKFTracker() {}


void UKFTracker::processMeasurement(const MeasurementPackage &pack) {

    // Compute the time elapsed between the current and previous measurements:
    const double dt = (pack.timestamp_ - previous_timestamp_) / 1000000.0;	// In seconds.

    // Update previous timestamp:
    previous_timestamp_ = pack.timestamp_;


    // INITIALIZATION:

    if (dt <= 0 || dt > MAX_DT) {
        initialize(pack);

        updateRMSE(pack.gt_);

        log(true, pack.sensor_type_ == MeasurementPackage::RADAR ? 'R' : 'L', { 0, 0, 0, 0, 0 });

        return; // Done initializing. No need to predict or update.
    }
    

    // PREDICTION:

    // Measure start time:
    clock_t begin = clock();

    // Try to make the prediction while keeping an eye on numerical inestability:
    // See https://discussions.udacity.com/t/numerical-instability-of-the-implementation/230449

    bool ok = true;

    try {
        ukf_.predict(dt);
    } catch (std::range_error e) {
        ok = false;

        ++inestability_; // TODO: IS THIS USERD?

        // cout << BEEP;

        // If convariance matrix is non positive definite (because of numerical
        // instability), reset it with the identiy matrix:
        ukf_.setP(MatrixXd::Identity(5, 5)); // TODO: Consistent naming
        
        // Redo prediction using the current measurement. Now P is definite, so we
        // should not get the error again.
        ukf_.predict(dt);
    }


    // UPDATE:
    // Update UKF, NIS and RSME:

    // OUTPUT current state and state covariance:
    // cout << "x_ = " << ekf_.x_ << endl;
    // cout << "P_ = " << ekf_.P_ << endl;

    char sensor;
    vector<double> NIS;

    if (pack.sensor_type_ == MeasurementPackage::RADAR) {
        const double NIS_radar = ukf_.updateRadar(pack.raw_measurements_);

        sensor = 'R';
        NIS = updateNIS(NIS_radar, ++total_radar_, NIS_3_table_, radar_NIS_results_);
    } else {
        const double NIS_lidar = ukf_.updateLidar(pack.raw_measurements_);

        sensor = 'L';
        NIS = updateNIS(NIS_lidar, ++total_lidar_, NIS_2_table_, lidar_NIS_results_);
    }

    updateRMSE(pack.gt_);

    // Measure end time:
    clock_t end = clock();

    // Update average time:
    time_ += (double)(end - begin) / CLOCKS_PER_SEC;

    // Log stuff. TODO: Check if log enabled, but always log all at the end!
    log(ok, sensor, NIS);
}


VectorXd UKFTracker::getCurrentState() {
    return ukf_.getCurrentState();
}


VectorXd UKFTracker::getCurrentRMSE() {
    return RMSE_;
}
