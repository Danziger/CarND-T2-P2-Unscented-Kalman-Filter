#ifndef EKFTracker_H_
#define EKFTracker_H_


#include "MeasurementPackage.h"
#include "UKF.h"
#include "Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


class UKFTracker {

    // The Unscented Kalman Filter (UKF) instance:
    UKF ukf_;

    // Indicates if the EKF state has been initialized (first measurement):
    bool is_initialized_ = false;

    // Previous timestamp:
    long long previous_timestamp_ = 0;

    // NIS stuff:
    int total_lidar_ = 0;
    int total_radar_ = 0;

    vector<int> lidar_NIS_results_{ 0, 0, 0, 0 };
    vector<int> radar_NIS_results_{ 0, 0, 0, 0 };

    vector<double> NIS_2_table_{ 0.103, 0.211, 4.605, 5.991 };
    vector<double> NIS_3_table_{ 0.352, 0.584, 6.251, 7.815 };

    /**
    * Initializes the state of the Kalman Filter using the first measurement.
    */
    void initialize(const MeasurementPackage &pack);


public:

    UKFTracker();

    virtual ~UKFTracker();

    /**
    * Run the whole flow of the KF/EKF from here and return the NIS for
    * lidar or radar, depending on the type of the input measurement.
    */
    vector<double> processMeasurement(const MeasurementPackage &pack);

    /**
    * Get the current filter state as [px, py, vx, vy]
    */
    VectorXd getCurrentState();
};


#endif /* EKFTracker_H_ */
