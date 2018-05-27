#ifndef UKF_H
#define UKF_H


#include "common/Eigen-3.3/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


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
    double update(
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
    * Initializes the measurement covariance matrixes.
    * @param R_lidar Measurement covariance matrix for lidar.
    * @param R_radar Measurement covariance matrix for radar.
    */
    void setR(
        const MatrixXd &R_lidar,
        const MatrixXd &R_radar
    );

    /**
    * Initializes the state covariance matrix.
    * @param P_in State covariance matrix
    */
    void setP(const MatrixXd &P);

    /**
    * Initializes the current state. 
    */
    void setState(
        const float px,
        const float py,
        const float v_abs,
        const float yaw_angle,
        const float yaw_rate
    );

    /**
    * Initializes the process and measurement noises.
    */
    void setNoise(const float std_a, const float std_yawdd);

    /**
    * Get the current filter state as [px, py, vx, vy]
    */
    VectorXd getCurrentState();

    /**
    * Get state covariance matrix P.
    * @return State covariance matrix P.
    */
    MatrixXd getP();

    /**
     * Predicts sigma points, the state, and the state covariance matrix.
     * @param delta_t Time between k and k+1 in s
     */
    void predict(const double dt);

    /**
     * Updates the state and the state covariance matrix using a laser measurement.
     * @param z The measurement at k+1
     * @return NIS value for current measurement.
     */
    double updateLidar(const VectorXd &z);

    /**
     * Updates the state and the state covariance matrix using a radar measurement.
     * @param z The measurement at k+1
     * @return NIS value for current measurement.
     */
    double updateRadar(const VectorXd &z);
};

#endif /* UKF_H */
