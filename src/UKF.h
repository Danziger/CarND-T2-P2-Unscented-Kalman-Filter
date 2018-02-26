#ifndef UKF_H
#define UKF_H


#include "Tools.h"
#include "Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;


class UKF {

    // State vector: [px, py, v_abs, yaw_angle, yaw_rate] in SI units:
    VectorXd x_;

    // State covariance matrix:
    MatrixXd P_;

    // Measurement covariance matrixes:
    MatrixXd R_lidar_;
    MatrixXd R_radar_;

    // Predicted sigma points matrix:
    MatrixXd Xsig_pred_;

    // Process noise standard deviation longitudinal acceleration in m/s^2:
    double std_a_;

    // Process noise standard deviation yaw acceleration in rad/s^2:
    double std_yawdd_;

    // Weights of sigma points:
    VectorXd weights_;


    /**
    * A helper method to calculate augmented sigma points.
    */
    MatrixXd calculateAugmentedSigmaPoints();

    /**
    * A helper method to predict sigma points at timestep k + 1
    */
    MatrixXd predictSigmaPoints(const double dt, MatrixXd Xsig_aug);

    /**
    * Performs all the common update steps for lidar and radar (in measurement
    * space)
    */
    void update(
        const VectorXd z,
        const MatrixXd Zsig,
        const MatrixXd R,
        const double weight0,
        const double weightN,
        const int N_Z
    );

public:

    UKF();

    virtual ~UKF();

    /**
    * Initializes the state covariance matrix.
    * @param P_in State covariance matrix
    */
    void initMatrixes(
        const MatrixXd &P,
        const MatrixXd &R_lidar,
        const MatrixXd &R_radar
    );

    /**
    * Initializes the current state. 
    */
    void initState(
        const float px,
        const float py,
        const float v_abs,
        const float yaw_angle,
        const float yaw_rate
    );

    /**
    * Initializes the process and measurement noises.
    */
    void initNoise(const float std_a, const float std_yawdd);

    /**
    * Get the current filter state as [px, py, vx, vy]
    */
    VectorXd getCurrentState();

    /**
     * Predicts sigma points, the state, and the state covariance matrix.
     * @param delta_t Time between k and k+1 in s
     */
    void predict(const double dt);

    /**
     * Updates the state and the state covariance matrix using a laser measurement.
     * @param z The measurement at k+1
     */
    void updateLidar(const VectorXd &z);

    /**
     * Updates the state and the state covariance matrix using a radar measurement.
     * @param z The measurement at k+1
     */
    void updateRadar(const VectorXd &z);
};

#endif /* UKF_H */
