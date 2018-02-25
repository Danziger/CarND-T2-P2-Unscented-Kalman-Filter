#ifndef EKFTracker_H_
#define EKFTracker_H_


#include "MeasurementPackage.h"
#include "UKF.h"
#include "Eigen/Dense"

#include <vector>
#include <string>
#include <fstream>


using Eigen::MatrixXd;
using Eigen::VectorXd;


class UKFTracker {

    // The Unscented Kalman Filter (UKF) instance:
    UKF ukf_;

    // Indicates if the EKF state has been initialized (first measurement):
    bool is_initialized_;

    // Previous timestamp:
    long long previous_timestamp_;


    /**
    * Initializes the state of the Kalman Filter using the first measurement.
    */
    void initialize(const MeasurementPackage &pack);


public:

    UKFTracker();

    virtual ~UKFTracker();

    /**
    * Run the whole flow of the KF/EKF from here.
    */
    void processMeasurement(const MeasurementPackage &pack);

    /**
    * Get the current filter state = px, py, vx, vy
    */
    VectorXd getCurrentState();
};


#endif /* EKFTracker_H_ */
