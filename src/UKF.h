#ifndef UKF_H
#define UKF_H


#include "Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;


class UKF {

    // State vector: [px, py, v_abs, yaw_angle, yaw_rate] in SI units:
    VectorXd x_;

    // State covariance matrix:
    MatrixXd P_;

    // Predicted sigma points matrix:
    MatrixXd Xsig_pred_;

    // Process noise standard deviation longitudinal acceleration in m/s^2:
    double std_a_;

    // Process noise standard deviation yaw acceleration in rad/s^2:
    double std_yawdd_;

    // Laser measurement noise standard deviation position1 in m:
    double std_laspx_;

    // Laser measurement noise standard deviation position2 in m:
    double std_laspy_;

    // Radar measurement noise standard deviation radius in m:
    double std_radr_;

    // Radar measurement noise standard deviation angle in rad:
    double std_radphi_;

    // Radar measurement noise standard deviation radius change in m/s:
    double std_radrd_;

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

public:

    UKF();

    virtual ~UKF();

    /**
    * Initializes the state covariance matrix.
    * @param P_in State covariance matrix
    */
    void initStateCovarianceMatrix(const MatrixXd &P_in);

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
    void initNoise(
        const float std_a,
        const float std_yawdd,
        const float std_laspx,
        const float std_laspy,
        const float std_radr,
        const float std_radphi,
        const float std_radrd
    );

    /**
    * Get the current filter state = px, py, vx, vy
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
